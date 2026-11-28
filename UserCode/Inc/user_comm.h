#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** Init USART2 @ 115200 8N1 on PA2/PA3 (ST-Link VCP). Pins already AF'd in MX_GPIO_Init. */
void user_comm_init(void);

/**
 * Non-blocking RX parse + ~60 Hz telemetry TX.
 * Call from user_loop only — never from the TIM1 ISR.
 */
void user_comm_poll(void);

#ifdef __cplusplus
}
#endif
