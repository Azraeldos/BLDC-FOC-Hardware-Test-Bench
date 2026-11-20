# BLDC_FOC

STM32 firmware for BLDC motor bring-up and (eventually) FOC on a **NUCLEO-F446RE** with a **SimpleFOC Mini v1.0** (DRV8313) power stage and an **AS5600** magnetic encoder.

Right now the project spins the motor **open-loop** and reads the encoder for debugging. Sensor-aligned FOC is not implemented yet.

## Hardware

| Piece | Role |
|-------|------|
| NUCLEO-F446RE | MCU |
| SimpleFOC Mini v1.0 (DRV8313) | 3-PWM + EN gate driver |
| AS5600 | Magnetic angle sensor (I2C) |
| BLDC motor | 3-phase load |

### Pin map

| Function | MCU pin | Mini / sensor |
|----------|---------|---------------|
| Phase U PWM | PA8 (TIM1_CH1) | IN1 |
| Phase V PWM | PA9 (TIM1_CH2) | IN2 |
| Phase W PWM | PA10 (TIM1_CH3) | IN3 |
| Driver enable | PC7 | EN |
| I2C1 SCL | PB6 | AS5600 SCL |
| I2C1 SDA | PB7 | AS5600 SDA |

Motor phases → Mini **M1 / M2 / M3**. Motor supply goes to the Mini (not USB alone). Share GND with the Nucleo.

## What’s implemented

- CubeMX / CMake project for STM32F446RE
- TIM1 center-aligned 3-phase PWM (~25 kHz)
- Driver EN GPIO
- AS5600 I2C bus scan + raw 12-bit angle read
- Open-loop rotating cosine voltage on U/V/W
- Live Watch variables for enable, amplitude, and electrical speed

Application logic lives in `UserCode/Src/user_main.cpp` (`user_setup` / `user_loop` called from `main.c`).

## Build & flash

1. Open / configure the CMake project (STM32Cube / VS Code / CLI as you prefer).
2. Build for the NUCLEO-F446RE target.
3. Flash over ST-Link.
4. Power the Mini motor supply and connect the motor before enabling drive.

## Bring-up (open-loop)

Watch these globals in the debugger Live Expressions / Watch window:

| Variable | Meaning |
|----------|---------|
| `g_enc_found` | `1` if AS5600 (`0x36`) answered the bus scan |
| `g_enc_raw` | Raw angle `0..4095` |
| `g_enc_status` | Last I2C read status |
| `g_motor_en` | `0` = off (EN low); `1` = open-loop spin |
| `g_amp` | Voltage amplitude (start ~`0.05`–`0.20`) |
| `g_omega` | Electrical speed in rad/s |

Typical flow:

1. Flash with motor/supply ready (or keep `g_motor_en = 0` until ready).
2. Confirm `g_enc_found == 1` and `g_enc_raw` moves when you turn the shaft.
3. Set `g_motor_en = 1` to spin.
4. Raise `g_omega` in steps; bump `g_amp` slightly if it stalls.

Mechanical RPM ≈ `(g_omega / 2π) × 60 / pole_pairs`.

## Project layout

```
Core/          CubeMX-generated HAL init (GPIO, I2C1, TIM1)
UserCode/      Application code (encoder + open-loop drive)
Drivers/       CMSIS + STM32F4 HAL
BLDC_FOC.ioc   CubeMX configuration
```

## Future implementations

- [ ] Encoder → electrical angle (pole pairs + zero-offset / alignment calibration)
- [ ] Sensor-aligned **voltage FOC** (dq voltage → inverse Park → PWM; encoder closes the angle loop)
- [ ] Outer **speed / position** loop (PI on RPM or shaft angle)
- [ ] Move the control update off `HAL_Delay(2)` into a TIM ISR / fixed-rate interrupt
- [ ] Safer defaults and fault handling (e.g. start with `g_motor_en = 0`, optional Mini `nFT` / fault pin)
- [ ] **Current FOC** only if hardware is added (ADC + current sense); Mini v1 / DRV8313 is typically voltage-only and ADC is not enabled in Cube yet
- [ ] Complementary PWM / dead-time if a different 6-PWM power stage is used later (not required for Mini)
