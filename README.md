# BLDC_FOC

STM32 firmware for BLDC motor bring-up and sensor-aligned **voltage FOC** on a **NUCLEO-F446RE** with a **SimpleFOC Mini v1.0** (DRV8313) power stage and an **AS5600** magnetic encoder.

## Hardware

| Piece | Role |
|-------|------|
| NUCLEO-F446RE | MCU |
| SimpleFOC Mini v1.0 (DRV8313) | 3-PWM + EN gate driver |
| AS5600 | Magnetic angle sensor (I2C @ 400 kHz) |
| BLDC motor | 3-phase load |

### Pin map

| Function | MCU pin | Mini / sensor |
|----------|---------|---------------|
| Phase U PWM | PA8 (TIM1_CH1) | IN1 |
| Phase V PWM | PA9 (TIM1_CH2) | IN2 |
| Phase W PWM | PA10 (TIM1_CH3) | IN3 |
| Driver enable | PC7 | EN |
| Driver fault (optional) | PC8 | nFAULT (active-low) |
| I2C1 SCL | PB6 | AS5600 SCL |
| I2C1 SDA | PB7 | AS5600 SDA |
| USART2 TX (VCP) | PA2 | ST-Link virtual COM |
| USART2 RX (VCP) | PA3 | ST-Link virtual COM |

Motor phases → Mini **M1 / M2 / M3**. Motor supply goes to the Mini (not USB alone). Share GND with the Nucleo.

Wire Mini **nFAULT** to **PC8** if you want hardware fault shutdown. PC8 is pulled up; leave it disconnected if unused (`g_fault_en` can be set to `0`).

## What’s implemented

- CubeMX / CMake project for STM32F446RE
- TIM1 center-aligned 3-phase PWM (~25 kHz)
- Driver EN GPIO + optional nFAULT on PC8
- AS5600 I2C bus scan + raw 12-bit angle read
- Open-loop rotating cosine voltage on U/V/W
- Encoder → electrical angle (`g_pole_pairs` + `g_zero_offset`)
- Alignment calibration (lock d-axis, sample offset)
- Sensor-aligned **voltage FOC** (dq → inverse Park/Clarke → PWM)
- Outer **speed PI** and cascaded **position PI**
- **Haptic force-feedback** (mode 4): detents, bounded range, return-to-center, viscous damper
- Non-blocking **USART2 VCP** @ 115200: profile commands in, ~60 Hz telemetry out
- Flash-persisted calibration (pole pairs + zero offset)
- Fixed-rate control in TIM1 update ISR (~2 kHz)
- Live Watch variables for mode, enable, gains, and telemetry

Application entry: `UserCode/Src/user_main.cpp` (thin glue). Modules: `as5600`, `motor_pwm`, `foc_math`, `control`, `user_params`, `user_flash`, `user_comm`. Control runs in `HAL_TIM_PeriodElapsedCallback`; serial + flash save from `user_loop`.

## Build & flash

1. Open / configure the CMake project (STM32Cube / VS Code / CLI as you prefer).
2. Build for the NUCLEO-F446RE target.
3. Flash over ST-Link.
4. Power the Mini motor supply and connect the motor before enabling drive.

Flash layout: code uses the first **384 KB**; **sector 7** (`0x08060000`) stores calibration.

## Bring-up

Watch these globals in the debugger Live Expressions / Watch window:

| Variable | Meaning |
|----------|---------|
| `g_enc_found` | `1` if AS5600 (`0x36`) answered the bus scan |
| `g_enc_raw` | Raw angle `0..4095` |
| `g_enc_status` | Last I2C read status |
| `g_motor_en` | `0` = off (EN low); `1` = run selected mode |
| `g_mode` | `0` open-loop · `1` torque · `2` speed · `3` position · `4` haptic |
| `g_amp` | Voltage amplitude / \|vq\| clamp (start ~`0.05`–`0.20`) |
| `g_omega` | Open-loop elec. speed, or speed **reference** in mode 2 [rad/s elec.] |
| `g_vd` / `g_vq` | dq voltages (mode 1; mode 2/3 override `vq`) |
| `g_pole_pairs` | Motor pole pairs |
| `g_zero_offset` | Mechanical angle [rad] at electrical 0 |
| `g_align_req` | Set `1` to run alignment |
| `g_align_busy` / `g_align_done` | Alignment status |
| `g_align_amp` | DC lock voltage during align |
| `g_save_req` / `g_save_ok` | Set `1` to store calib; `g_save_ok` after write |
| `g_calib_loaded` | `1` if flash calib loaded at boot |
| `g_kp_speed` / `g_ki_speed` | Speed PI gains |
| `g_pos_ref` | Position setpoint [mech rad, multi-turn] |
| `g_omega_max` | Max \|ω\| from position loop [elec rad/s] |
| `g_kp_pos` / `g_ki_pos` | Position PI gains |
| `g_fault_en` / `g_fault` / `g_fault_clr` | nFAULT monitor / latch / clear |
| `g_theta_e` | Electrical angle [rad] |
| `g_mech_angle` | Wrapped mechanical angle [rad] |
| `g_mech_unwrapped` | Continuous mechanical angle [rad] |
| `g_speed_e` | Measured electrical speed [rad/s] |
| `g_omega_cmd` | Speed command after position PI |
| `g_vq_out` | Commanded `vq` after PI / clamp |
| `g_haptic_type` | `1` detents · `2` range · `3` spring · `4` damper |
| `g_haptic_detents` / `g_haptic_kp` / `g_haptic_kd` | Profile gains |
| `g_haptic_min_deg` / `g_haptic_max_deg` / `g_haptic_center_deg` | Range / spring angles [deg] |
| `g_haptic_capture_deg` | Detent capture width (`<=0` = auto half-sector) |
| `g_angle_deg` / `g_detent_index` / `g_vel_deg_s` | Haptic telemetry |

### 1) Encoder check

1. Flash with `g_motor_en = 0` (default).
2. Confirm `g_enc_found == 1` and `g_enc_raw` moves when you turn the shaft.

### 2) Open-loop (optional sanity check)

1. `g_mode = 0`, set `g_amp` ~`0.08`, `g_omega` ~`30`.
2. `g_motor_en = 1` → motor should spin.

Mechanical RPM ≈ `(g_omega / 2π) × 60 / pole_pairs`.

### 3) Align encoder (required once before FOC)

1. Set `g_pole_pairs` to your motor.
2. Shaft free; supply on.
3. `g_align_req = 1` → lock ~1 s, then sample.
4. Wait for `g_align_done == 1`.
5. Optional: `g_save_req = 1` and confirm `g_save_ok == 1` so reboot keeps the offset.

### 4) Voltage FOC (torque)

1. `g_mode = 1`, `g_vd = 0`, small `g_vq`, then `g_motor_en = 1`.

### 5) Speed mode

1. `g_mode = 2`, set `g_omega` (electrical rad/s), tune `g_kp_speed` / `g_ki_speed`.
2. `g_motor_en = 1`.

### 6) Position mode

1. Note `g_mech_unwrapped`, set `g_pos_ref` nearby (or `g_pos_ref = g_mech_unwrapped` first).
2. `g_mode = 3`, modest `g_amp` / `g_omega_max`, then `g_motor_en = 1`.
3. Step `g_pos_ref` by small amounts (e.g. ±0.5 rad) while tuning `g_kp_pos`.

### 7) Haptic mode (force feedback)

Requires prior alignment (same as modes 1–3). Haptic laws run in the ~2 kHz TIM1 tick with `vd = 0` and `|vq| ≤ g_amp`.

1. Align and set a modest `g_amp` (~`0.08`–`0.20`).
2. Either set `g_mode = 4` and tune `g_haptic_*` in the Watch window, **or** send a profile line over the ST-Link VCP (below).
3. `g_motor_en = 1` — enable is never set by serial; you must arm it yourself.

| `g_haptic_type` | Feel |
|-----------------|------|
| `1` | Discrete detents around the circle |
| `2` | Free twist in `[min, max]` with hard end-stops |
| `3` | Return-to-center spring |
| `4` | Viscous damper only (`kp` ignored) |

### Serial (ST-Link VCP @ 115200 8N1)

Open the Nucleo’s virtual COM port. RX/TX are polled in `user_loop` so the control ISR is never blocked by string I/O.

**Host → MCU** (newline-terminated; invalid lines are ignored):

```
P,<profile_type>,<num_detents>,<kp>,<kd>,<min_deg>,<max_deg>,<center_deg>
```

Example spring: `P,3,12,0.08,0.0005,-45,45,0`

If the spring feels weak, raise `g_haptic_kp` and/or `g_amp` in the Watch window (no rebuild needed). If it oscillates, raise `g_haptic_kd` slightly.

A valid `P` line updates haptic parameters and sets `g_mode = 4` atomically. It does **not** set `g_motor_en`.

**MCU → host** (~60 Hz):

```
A,<angle_deg>,<detent_index>,<vel_deg_s>,<fault_flag>
```

### Fault pin

- With Mini **nFAULT → PC8**: a low level latches `g_fault = 1`, forces EN off, and clears `g_motor_en`.
- Clear with `g_fault_clr = 1` once the pin is high again.
- No wire? Leave disconnected (internal pull-up) or set `g_fault_en = 0`.

## Project layout

```
Core/              CubeMX-generated HAL init (GPIO, I2C1, TIM1)
UserCode/Inc|Src/  Application modules:
  user_main        setup / loop / TIM callback glue
  user_params      Watch globals (g_*)
  as5600           encoder I2C
  motor_pwm        TIM1 duties + EN
  foc_math         Park/Clarke helpers
  control          modes, PI, haptic, align, fault, save
  user_flash       calib sector R/W
  user_comm        USART2 VCP profile RX + telemetry TX
Drivers/           CMSIS + STM32F4 HAL
BLDC_FOC.ioc       CubeMX configuration
```

## Future implementations

- [x] Encoder → electrical angle (pole pairs + zero-offset / alignment)
- [x] Sensor-aligned **voltage FOC**
- [x] Outer **speed** and **position** loops
- [x] TIM ISR control (no `HAL_Delay` in the loop)
- [x] Persist `g_zero_offset` / `g_pole_pairs` to flash
- [x] Optional Mini `nFAULT` handling (PC8)
- [x] Haptic profiles + non-blocking VCP serial
- [ ] **Current FOC** only if hardware is added (ADC + current sense); SimpleFOCMiniv1 / DRV8313 is typically voltage-only
- [ ] Complementary PWM / dead-time for a 6-PWM stage (not required for SimpleFOCMiniv1)
