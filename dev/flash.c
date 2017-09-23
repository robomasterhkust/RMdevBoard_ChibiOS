/**
 * Edward ZHANG, 20170320
 * @file    flash.c
 * @brief   STM32F4xx flash driver
 */
#include "hal.h"
#include "flash.h"

size_t flashSectorSize(flashsector_t sector)
{
    if (sector <= 3)
        return 16 * 1024;
    else if (sector == 4)
        return 64 * 1024;
    else if (sector >= 5 && sector <= 11)
        return 128 * 1024;
    return 0;
}

flashaddr_t flashSectorBegin(flashsector_t sector)
{
    flashaddr_t address = FLASH_BASE;
    while (sector > 0)
    {
        --sector;
        address += flashSectorSize(sector);
    }
    return address;
}

flashaddr_t flashSectorEnd(flashsector_t sector)
{
    return flashSectorBegin(sector + 1);
}

flashsector_t flashSectorAt(flashaddr_t address)
{
    flashsector_t sector = 0;
    while (address >= flashSectorEnd(sector))
        ++sector;
    return sector;
}

/**
 * @brief Wait for the flash operation to finish.
 */
#define flashWaitWhileBusy() { while (FLASH->SR & FLASH_SR_BSY) {} }

/**
  * @brief  Enables or disables the Data Cache feature.
  * @param  NewState: new state of the Data Cache.
  *          This parameter  can be: ENABLE or DISABLE.
  * @retval None
  */
static void FLASH_DataCacheCmd(FunctionalState NewState)
{
  if(NewState != DISABLE)
  {
    FLASH->ACR |= FLASH_ACR_DCEN;
  }
  else
  {
    FLASH->ACR &= (~FLASH_ACR_DCEN);
  }
}

/**
 * @brief Unlock the flash memory for write access.
 * @return CH_SUCCESS  Unlock was successful.
 * @return CH_FAILED    Unlock failed.
 */
static uint8_t flashUnlock(void)
{
    /* Check if unlock is really needed */
    if (!(FLASH->CR & FLASH_CR_LOCK))
        return CH_SUCCESS;

    /* Write magic unlock sequence */
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;

    /* Check if unlock was successful */
    if (FLASH->CR & FLASH_CR_LOCK)
        return CH_FAILED;
    return CH_SUCCESS;
}

flashdata_t flashReadData(flashaddr_t address)
{
  return *(flashdata_t*)address;
}

void flashWriteData(flashaddr_t address, const flashdata_t data)
{
  //Red LED on indicates flash error
  if(flashUnlock()==CH_FAILED)
    return;

  FLASH_DataCacheCmd(ENABLE);
  FLASH->CR &= ~FLASH_CR_PSIZE_MASK;
  FLASH->CR |= FLASH_CR_PSIZE_VALUE;
  flashWaitWhileBusy();

  /* Enter flash programming mode */
  FLASH->CR |= FLASH_CR_PG;

  /* Write the data */
  *(flashdata_t*)address = data;

  /* Wait for completion */
  flashWaitWhileBusy();

  /* Exit flash programming mode */
  FLASH->CR &= ~FLASH_CR_PG;

  flashLock();
}

static void flashWriteDataI(flashaddr_t address, const flashdata_t data)
{
    /* Enter flash programming mode */
    FLASH->CR |= FLASH_CR_PG;

    /* Write the data */
    *(flashdata_t*)address = data;

    /* Wait for completion */
    flashWaitWhileBusy();

    /* Exit flash programming mode */
    FLASH->CR &= ~FLASH_CR_PG;
}

void flashSectorErase(flashsector_t sector)
{
    /* Unlock flash for write access */
    if(flashUnlock() == CH_FAILED)
        return;

    /* Wait for any busy flags. */
    flashWaitWhileBusy();

    FLASH_DataCacheCmd(DISABLE);//disable cache

    /* Setup parallelism before any program/erase */
    FLASH->CR &= ~FLASH_CR_PSIZE_MASK;
    FLASH->CR |= FLASH_CR_PSIZE_VALUE;

    flashWaitWhileBusy();
    /* Start deletion of sector.
     * SNB(3:1) is defined as:
     * 0000 sector 0
     * 0001 sector 1
     * ...
     * 1011 sector 11
     * others not allowed */
    FLASH->CR &= ~(FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_2 | FLASH_CR_SNB_3);//sector mask
    if (sector & 0x1) FLASH->CR |= FLASH_CR_SNB_0;
    if (sector & 0x2) FLASH->CR |= FLASH_CR_SNB_1;
    if (sector & 0x4) FLASH->CR |= FLASH_CR_SNB_2;
    if (sector & 0x8) FLASH->CR |= FLASH_CR_SNB_3;
    FLASH->CR |= FLASH_CR_SER;
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait until it's finished. */
    flashWaitWhileBusy();

    /* Sector erase flag does not clear automatically. */
    FLASH->CR &= ~FLASH_CR_SER;
    FLASH->CR &= ~(FLASH_CR_SNB_0 | FLASH_CR_SNB_1 | FLASH_CR_SNB_2 | FLASH_CR_SNB_3);

    FLASH_DataCacheCmd(ENABLE);

    /* Lock flash again */
    flashLock();
}

void flashWrite(flashaddr_t address, const char* buffer, size_t size)
{
    if(flashUnlock() == CH_FAILED)
        return;

    flashWaitWhileBusy();

    /* Setup parallelism before any program/erase */
    FLASH->CR &= ~FLASH_CR_PSIZE_MASK;
    FLASH->CR |= FLASH_CR_PSIZE_VALUE;

    /* Now, address is correctly aligned. One can copy data directly from
     * buffer's data to flash memory until the size of the data remaining to be
     * copied requires special treatment. */
    while (size >= sizeof(flashdata_t))
    {
        flashWriteDataI(address, *(const flashdata_t*)buffer);
        address += sizeof(flashdata_t);
        buffer += sizeof(flashdata_t);
        size -= sizeof(flashdata_t);
    }

    /* Now, address is correctly aligned, but the remaining data are to
     * small to fill a entier flashdata_t. Thus, one must read data already
     * in flash and update them with buffer's data before writing an entire
     * flashdata_t to flash memory. */
    if (size > 0)
    {
        flashdata_t tmp = *(volatile flashdata_t*)address;
        memcpy(&tmp, buffer, size);
        flashWriteDataI(address, tmp);
    }
    /* Lock flash again */
    flashLock();
}

void flashRead(flashaddr_t address, char* buffer, size_t size)
{
    if(flashUnlock() == CH_FAILED)
        return;

    flashWaitWhileBusy();

    flashdata_t* flashbuffer = (flashdata_t*)buffer;

    while (size >= sizeof(flashdata_t))
    {
        *flashbuffer = flashReadData(address);
        address += sizeof(flashdata_t);
        flashbuffer++;
        size -= sizeof(flashdata_t);
    }

    flashLock();
}
