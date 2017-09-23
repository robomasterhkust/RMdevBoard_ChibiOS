/*
 * flash.h
 *
 *  Created on: 2016��6��2��
 *      Author: lenovo
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#if !defined(FLASH_SECTOR_COUNT) || defined(__DOXYGEN__)
#define FLASH_SECTOR_COUNT 12
#endif

/* size_t no longer exist */
#define size_t uint32_t

/* Vimble configured
 * Sector 3 :   0x0800 C000 - 0x0800 FFFF
 * Sector 6 :   0x0804 0000 - 0x0805 FFFF
 */

/* Error codes */

#define CH_FAILED 0
#define CH_SUCCESS 1
/** @brief Flash operation successful */
#define FLASH_RETURN_SUCCESS CH_SUCCESS

/** @brief Flash operation error because of denied access, corrupted memory.*/
#define FLASH_RETURN_NO_PERMISSION -1

/** @brief Flash operation error because of bad flash, corrupted memory */
#define FLASH_RETURN_BAD_FLASH -11


#ifdef __cplusplus
extern "C" {
#endif


uint32_t STMFLASH_ReadWord(uint32_t faddr);

/**
 * @brief Maximum program/erase parallelism
 *
 * FLASH_CR_PSIZE_MASK is the mask to configure the parallelism value.
 * FLASH_CR_PSIZE_VALUE is the parallelism value suitable for the voltage range.
 *
 * PSIZE(1:0) is defined as:
 * 00 to program 8 bits per step
 * 01 to program 16 bits per step
 * 10 to program 32 bits per step
 * 11 to program 64 bits per step
 */
// Warning, flashdata_t must be unsigned!!!
#if defined(STM32F4XX) || defined(__DOXYGEN__)
#define FLASH_CR_PSIZE_MASK         FLASH_CR_PSIZE_0 | FLASH_CR_PSIZE_1
#if ((STM32_VDD >= 270) && (STM32_VDD <= 360)) || defined(__DOXYGEN__)
#define FLASH_CR_PSIZE_VALUE        FLASH_CR_PSIZE_1
typedef uint32_t flashdata_t;
#elif (STM32_VDD >= 240) && (STM32_VDD < 270)
#define FLASH_CR_PSIZE_VALUE        FLASH_CR_PSIZE_0
typedef uint16_t flashdata_t;
#elif (STM32_VDD >= 210) && (STM32_VDD < 240)
#define FLASH_CR_PSIZE_VALUE        FLASH_CR_PSIZE_0
typedef uint16_t flashdata_t;
#elif (STM32_VDD >= 180) && (STM32_VDD < 210)
#define FLASH_CR_PSIZE_VALUE        ((uint32_t)0x00000000)
typedef uint8_t flashdata_t;
#else
#error "invalid VDD voltage specified"
#endif
#endif /* defined(STM32F4XX) */

/** @brief Address in the flash memory */
typedef uintptr_t flashaddr_t;

/** @brief Index of a sector */
typedef uint8_t flashsector_t;

/**
 * @brief Get the size of @p sector.
 * @return @p sector size in bytes.
 */
size_t flashSectorSize(flashsector_t sector);

/**
 * @brief Get the beginning address of @p sector.
 * @param sector Sector to retrieve the beginning address of.
 * @return First address (inclusive) of @p sector.
 */
flashaddr_t flashSectorBegin(flashsector_t sector);

/**
 * @brief Get the end address of @p sector.
 * @param sector Sector to retrieve the end address of.
 * @return End address (exclusive) of @p sector (i.e. beginning address of the next sector).
 */
flashaddr_t flashSectorEnd(flashsector_t sector);

/**
 * @brief Get the sector containing @p address.
 * @warning @p address must be in the flash addresses range.
 * @param address Address to be searched for.
 * @return Sector containing @p address.
 */
flashsector_t flashSectorAt(flashaddr_t address);

//uint8_t flashUnlock(void);
#define flashLock() { FLASH->CR |= FLASH_CR_LOCK; }

flashdata_t flashReadData(flashaddr_t address);
void flashWriteData(flashaddr_t address, const flashdata_t data);
void flashSectorErase(flashsector_t sector);
void flashWrite(flashaddr_t address, const char* buffer, size_t size);
void flashRead(flashaddr_t address, char* buffer, size_t size);

#endif /* INC_FLASH_H_ */
