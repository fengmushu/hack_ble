#include "flash_manage.h"


void flash_word_write(uint32_t * address, uint32_t value)
{
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    *address = value;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
}

uint32_t flash_word_read(uint32_t * address)
{
	uint32_t value;
    // Turn on flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    value=*address;

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }

    // Turn off flash write enable and wait until the NVMC is ready:
    NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

    while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
    {
        // Do nothing.
    }
		return value;
}


void nrf_nvmc_page_erase(uint32_t address) //function from nrf_nvmc.c
{ 
   // Enable erase.
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
  // Erase the page
  NRF_NVMC->ERASEPAGE = address;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
  {
  }
}

void flash_write_array(uint32_t address,uint8_t * array,uint8_t length)
{
if ((address>=START_DATA_ADDR)&&(address<=(VAR_ADDR+1023)))  //end with variable block (last page)
{	
uint32_t * adr_pnt=(uint32_t*) address;
uint32_t * array_pnt=(uint32_t*)array;	
while ((uint32_t)adr_pnt<=(address+(length-1))) flash_word_write(adr_pnt++,*(array_pnt++));
}
}

void flash_read_array(uint32_t address,uint8_t * array,uint8_t length)
{
if ((address>=START_DATA_ADDR)&&(address<=(VAR_ADDR+1023)))  //end with variable block (last page)
{		
uint32_t * adr_pnt=(uint32_t*) address;
uint32_t * array_pnt=(uint32_t*)array;	
while ((uint32_t)adr_pnt<=(address+(length-1))) *(array_pnt++)=*(adr_pnt++);
}
}

uint32_t flash_page_erase(uint32_t pagenum)
{
uint32_t addr=pagenum*1024;	
if ((addr>=START_DATA_ADDR)&&(addr<=VAR_ADDR))	
{
nrf_nvmc_page_erase(addr);	
return addr;	
}	
return 0;
}
