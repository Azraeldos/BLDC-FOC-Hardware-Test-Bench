#include "user_flash.h"
#include "stm32f4xx_hal.h"

// STM32F446RE sector 7: 128 KB @ 0x08060000 (reserved in linker script)
static constexpr uint32_t CALIB_ADDR   = 0x08060000u;
static constexpr uint32_t CALIB_SECTOR = FLASH_SECTOR_7;
static constexpr uint32_t CALIB_MAGIC  = 0x464F4342u;  // 'FOCB'
static constexpr uint32_t CALIB_VER    = 1u;

static uint32_t crc32_words(const uint32_t *data, uint32_t nwords)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (uint32_t i = 0; i < nwords; ++i) {
    uint32_t w = data[i];
    for (int b = 0; b < 32; ++b) {
      const uint32_t bit = (crc ^ w) & 1u;
      crc >>= 1;
      w >>= 1;
      if (bit) {
        crc ^= 0xEDB88320u;
      }
    }
  }
  return ~crc;
}

static uint32_t calib_crc(const UserCalib *c)
{
  // CRC over all fields except crc itself
  return crc32_words(reinterpret_cast<const uint32_t *>(c),
                     (sizeof(UserCalib) / sizeof(uint32_t)) - 1u);
}

uint8_t user_flash_load(UserCalib *out)
{
  const auto *src = reinterpret_cast<const UserCalib *>(CALIB_ADDR);
  if (src->magic != CALIB_MAGIC || src->version != CALIB_VER) {
    return 0;
  }
  if (src->crc != calib_crc(src)) {
    return 0;
  }
  if (src->pole_pairs == 0u || src->pole_pairs > 64u) {
    return 0;
  }
  *out = *src;
  return 1;
}

uint8_t user_flash_save(const UserCalib *in)
{
  UserCalib tmp = *in;
  tmp.magic   = CALIB_MAGIC;
  tmp.version = CALIB_VER;
  tmp.crc     = calib_crc(&tmp);

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector       = CALIB_SECTOR;
  erase.NbSectors    = 1;

  uint32_t sector_error = 0;
  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  const uint32_t *words = reinterpret_cast<const uint32_t *>(&tmp);
  const uint32_t nwords = sizeof(UserCalib) / sizeof(uint32_t);
  for (uint32_t i = 0; i < nwords; ++i) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          CALIB_ADDR + i * 4u,
                          words[i]) != HAL_OK) {
      HAL_FLASH_Lock();
      return 0;
    }
  }

  HAL_FLASH_Lock();

  // Verify
  UserCalib check = {};
  if (!user_flash_load(&check)) {
    return 0;
  }
  return (check.pole_pairs == tmp.pole_pairs &&
          check.zero_offset == tmp.zero_offset) ? 1u : 0u;
}
