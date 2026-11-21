#pragma once

#include <cstdint>

/** Scan I2C1; fills g_i2c_addrs / g_i2c_nfound and sets g_enc_found. */
void as5600_init(void);

/** Read raw angle 0..4095 (keeps last value on I2C error). */
uint16_t as5600_read_raw(void);

float as5600_raw_to_mech(uint16_t raw);

/** Electrical angle from mechanical, pole pairs, and zero offset. */
float as5600_mech_to_elec(float mech, float zero_offset, uint8_t pole_pairs);
