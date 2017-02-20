#ifndef FLASH_MANAGE_H
#define FLASH_MANAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"

static __INLINE uint16_t flash_page_size()
{
  return (uint16_t)NRF_FICR->CODEPAGESIZE;
}

#define FLASH_PAGE_SIZE  flash_page_size()

#define START_DATA_ADDR   0x23000//0x22800 //0x22C00 for some reason page 22C00 does not keep record, defected?
#define END_DATA_ADDR   0x3B800  //first record in last block
#define VAR_ADDR   0x3BC00
#define START_PAGE_NUM   (START_DATA_ADDR / FLASH_PAGE_SIZE)  //page number (dec 140)
#define END_PAGE_NUM   (END_DATA_ADDR / FLASH_PAGE_SIZE)  //page number (dec 238)
#define VAR_PAGE_NUM   (VAR_ADDR / FLASH_PAGE_SIZE)   

void flash_word_write(uint32_t * address, uint32_t value);
uint32_t flash_word_read(uint32_t * address);
void flash_write_array(uint32_t address,uint8_t * array,uint8_t length);
void flash_read_array(uint32_t address,uint8_t * array,uint8_t length);
uint32_t flash_page_erase(uint32_t pagenum);
#endif /* FLASH_MANAGE_H */
