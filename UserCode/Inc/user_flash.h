#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

struct UserCalib {
  uint32_t magic;
  uint32_t version;
  uint32_t pole_pairs;
  float    zero_offset;
  uint32_t crc;
};

/** Load calibration from flash. Returns 1 on success. */
uint8_t user_flash_load(UserCalib *out);

/** Erase calib sector and write. Returns 1 on success. */
uint8_t user_flash_save(const UserCalib *in);

#ifdef __cplusplus
}
#endif
