#include "user_comm.h"
#include "user_params.h"
#include "main.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>

// ---------------------------------------------------------------------------
// USART2 @ 115200 8N1 — polled, no HAL_UART (module disabled in CubeMX).
// PCLK1 = 45 MHz → BRR ≈ 391 (oversampling by 16).
// ---------------------------------------------------------------------------

static constexpr uint32_t USART_BAUD = 115200u;
static constexpr uint32_t USART_PCLK = 45000000u;

static constexpr size_t RX_LINE_MAX = 96;
static constexpr size_t TX_RING_SIZE = 256;
static constexpr uint32_t TELEMETRY_PERIOD_MS = 16;  // ~62.5 Hz

static char     g_rx_line[RX_LINE_MAX];
static size_t   g_rx_len = 0;

static char     g_tx_ring[TX_RING_SIZE];
static size_t   g_tx_head = 0;  // write
static size_t   g_tx_tail = 0;  // read
static uint32_t g_telem_last_ms = 0;

// Soft limits for P-line validation (vq still hard-clamped by g_amp).
static constexpr float KP_MAX = 1.0f;
static constexpr float KD_MAX = 0.05f;
static constexpr float ANGLE_ABS_MAX = 1080.0f;

static void usart2_init_regs(void)
{
  __HAL_RCC_USART2_CLK_ENABLE();

  // Ensure peripheral is off while programming baud / frame.
  USART2->CR1 = 0;
  USART2->CR2 = 0;
  USART2->CR3 = 0;

  // BRR = round(PCLK / baud) for OVER8 = 0 (matches HAL UART_DIV_SAMPLING16).
  USART2->BRR = (USART_PCLK + USART_BAUD / 2u) / USART_BAUD;

  // 8N1, oversampling by 16, TE + RE + UE.
  USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static size_t tx_used(void)
{
  if (g_tx_head >= g_tx_tail) {
    return g_tx_head - g_tx_tail;
  }
  return TX_RING_SIZE - (g_tx_tail - g_tx_head);
}

static size_t tx_free(void)
{
  // Leave one slot empty so head==tail means empty.
  return (TX_RING_SIZE - 1u) - tx_used();
}

static bool tx_push(char c)
{
  const size_t next = (g_tx_head + 1u) % TX_RING_SIZE;
  if (next == g_tx_tail) {
    return false;
  }
  g_tx_ring[g_tx_head] = c;
  g_tx_head = next;
  return true;
}

static bool tx_push_str(const char *s)
{
  while (*s) {
    if (!tx_push(*s++)) {
      return false;
    }
  }
  return true;
}

/** Drain TX ring into USART2 DR when TXE is set (non-blocking). */
static void tx_drain(void)
{
  while (g_tx_tail != g_tx_head) {
    if ((USART2->SR & USART_SR_TXE) == 0u) {
      break;
    }
    USART2->DR = static_cast<uint8_t>(g_tx_ring[g_tx_tail]);
    g_tx_tail = (g_tx_tail + 1u) % TX_RING_SIZE;
  }
}

/** Append integer (signed) without printf. */
static bool fmt_i32(char *dst, size_t dst_sz, size_t *pos, int32_t v)
{
  char tmp[12];
  size_t n = 0;
  uint32_t u;

  if (v < 0) {
    if (*pos + 1u >= dst_sz) {
      return false;
    }
    dst[(*pos)++] = '-';
    u = static_cast<uint32_t>(-v);
  } else {
    u = static_cast<uint32_t>(v);
  }

  if (u == 0u) {
    tmp[n++] = '0';
  } else {
    while (u > 0u && n < sizeof(tmp)) {
      tmp[n++] = static_cast<char>('0' + (u % 10u));
      u /= 10u;
    }
  }

  if (*pos + n >= dst_sz) {
    return false;
  }
  while (n > 0u) {
    dst[(*pos)++] = tmp[--n];
  }
  return true;
}

/**
 * Fixed-point float: one decimal place (e.g. 123.4).
 * Enough for angle/velocity telemetry; avoids nano.specs float printf.
 */
static bool fmt_f1(char *dst, size_t dst_sz, size_t *pos, float v)
{
  if (!std::isfinite(v)) {
    v = 0.f;
  }
  // Round to 0.1
  if (v >= 0.f) {
    v += 0.05f;
  } else {
    v -= 0.05f;
  }
  const int32_t scaled = static_cast<int32_t>(v * 10.f);
  int32_t whole = scaled / 10;
  int32_t frac = scaled % 10;
  if (frac < 0) {
    frac = -frac;
  }

  if (!fmt_i32(dst, dst_sz, pos, whole)) {
    return false;
  }
  if (*pos + 2u >= dst_sz) {
    return false;
  }
  dst[(*pos)++] = '.';
  dst[(*pos)++] = static_cast<char>('0' + frac);
  return true;
}

static void skip_ws(const char *&p)
{
  while (*p == ' ' || *p == '\t') {
    ++p;
  }
}

static bool parse_i32(const char *&p, int32_t *out)
{
  skip_ws(p);
  char *end = nullptr;
  // strtol is available with newlib-nano.
  const long v = strtol(p, &end, 10);
  if (end == p) {
    return false;
  }
  *out = static_cast<int32_t>(v);
  p = end;
  return true;
}

static bool parse_f32(const char *&p, float *out)
{
  skip_ws(p);
  char *end = nullptr;
  const float v = strtof(p, &end);
  if (end == p) {
    return false;
  }
  *out = v;
  p = end;
  return true;
}

static bool expect_comma(const char *&p)
{
  skip_ws(p);
  if (*p != ',') {
    return false;
  }
  ++p;
  return true;
}

/**
 * P,<type>,<num_detents>,<kp>,<kd>,<min_deg>,<max_deg>,<center_deg>
 * On success: atomically update haptic params and set g_mode = MODE_HAPTIC.
 * Does NOT set g_motor_en (enable remains a separate safety interlock).
 */
static bool handle_profile_line(const char *line)
{
  const char *p = line;
  skip_ws(p);
  if (*p != 'P' && *p != 'p') {
    return false;
  }
  ++p;
  if (!expect_comma(p)) {
    return false;
  }

  int32_t type_i = 0;
  int32_t detents_i = 0;
  float kp = 0.f, kd = 0.f, min_d = 0.f, max_d = 0.f, center = 0.f;

  if (!parse_i32(p, &type_i) || !expect_comma(p)) {
    return false;
  }
  if (!parse_i32(p, &detents_i) || !expect_comma(p)) {
    return false;
  }
  if (!parse_f32(p, &kp) || !expect_comma(p)) {
    return false;
  }
  if (!parse_f32(p, &kd) || !expect_comma(p)) {
    return false;
  }
  if (!parse_f32(p, &min_d) || !expect_comma(p)) {
    return false;
  }
  if (!parse_f32(p, &max_d) || !expect_comma(p)) {
    return false;
  }
  if (!parse_f32(p, &center)) {
    return false;
  }

  // Trailing junk (except whitespace) → reject.
  skip_ws(p);
  if (*p != '\0') {
    return false;
  }

  // --- validate ---
  if (type_i < 1 || type_i > 4) {
    return false;
  }
  if (detents_i < 1 || detents_i > 72) {
    return false;
  }
  if (!std::isfinite(kp) || !std::isfinite(kd) ||
      !std::isfinite(min_d) || !std::isfinite(max_d) || !std::isfinite(center)) {
    return false;
  }
  if (kp < 0.f || kp > KP_MAX || kd < 0.f || kd > KD_MAX) {
    return false;
  }
  if (std::fabs(min_d) > ANGLE_ABS_MAX || std::fabs(max_d) > ANGLE_ABS_MAX ||
      std::fabs(center) > ANGLE_ABS_MAX) {
    return false;
  }

  if (type_i == static_cast<int32_t>(HAPTIC_RANGE)) {
    // Span on the circle must be in (0, 360].
    float span = max_d - min_d;
    while (span <= 0.f) {
      span += 360.f;
    }
    while (span > 360.f) {
      span -= 360.f;
    }
    if (span <= 0.f || span > 360.f) {
      return false;
    }
  }

  // --- atomic commit (TIM1 tick must not see a half-written profile) ---
  __disable_irq();
  g_haptic_type       = static_cast<uint8_t>(type_i);
  g_haptic_detents    = static_cast<uint8_t>(detents_i);
  g_haptic_kp         = kp;
  g_haptic_kd         = kd;
  g_haptic_min_deg    = min_d;
  g_haptic_max_deg    = max_d;
  g_haptic_center_deg = center;
  g_mode              = MODE_HAPTIC;
  __enable_irq();

  return true;
}

static void rx_feed_byte(char c)
{
  if (c == '\r') {
    return;  // ignore CR; wait for LF
  }
  if (c == '\n') {
    if (g_rx_len > 0u) {
      g_rx_line[g_rx_len] = '\0';
      (void)handle_profile_line(g_rx_line);
    }
    g_rx_len = 0;
    return;
  }

  if (g_rx_len + 1u >= RX_LINE_MAX) {
    // Overflow: discard line.
    g_rx_len = 0;
    return;
  }
  g_rx_line[g_rx_len++] = c;
}

static void rx_poll(void)
{
  while (USART2->SR & USART_SR_RXNE) {
    const char c = static_cast<char>(USART2->DR & 0xFFu);
    rx_feed_byte(c);
  }
  // Clear overrun if it happened so RX keeps working.
  if (USART2->SR & USART_SR_ORE) {
    (void)USART2->DR;
  }
}

static void telemetry_maybe_queue(void)
{
  const uint32_t now = HAL_GetTick();
  if ((now - g_telem_last_ms) < TELEMETRY_PERIOD_MS) {
    return;
  }
  g_telem_last_ms = now;

  // Snapshot volatiles once (may tear slightly; telemetry is soft).
  const float angle = g_angle_deg;
  const int16_t detent = g_detent_index;
  const float vel = g_vel_deg_s;
  const uint8_t fault = g_fault;

  char line[64];
  size_t pos = 0;
  line[pos++] = 'A';
  line[pos++] = ',';
  if (!fmt_f1(line, sizeof(line), &pos, angle)) {
    return;
  }
  line[pos++] = ',';
  if (!fmt_i32(line, sizeof(line), &pos, detent)) {
    return;
  }
  line[pos++] = ',';
  if (!fmt_f1(line, sizeof(line), &pos, vel)) {
    return;
  }
  line[pos++] = ',';
  if (!fmt_i32(line, sizeof(line), &pos, fault)) {
    return;
  }
  line[pos++] = '\n';
  line[pos] = '\0';

  // Drop frame if TX ring cannot take the whole line (never block).
  if (tx_free() < pos) {
    return;
  }
  (void)tx_push_str(line);
}

void user_comm_init(void)
{
  g_rx_len = 0;
  g_tx_head = 0;
  g_tx_tail = 0;
  g_telem_last_ms = HAL_GetTick();
  usart2_init_regs();
}

void user_comm_poll(void)
{
  rx_poll();
  telemetry_maybe_queue();
  tx_drain();
}
