//LIS2DH accelerometer registers and functions


#include "stdint.h"
#include "LIS2DH.h"
#include "nrf_delay.h"

SPIConfig_t my_spi = {.Config.Fields.BitOrder = SPI_BITORDER_MSB_LSB,
                        .Config.Fields.Mode     = SPI_MODE0,
                        .Frequency              = SPI_FREQ_8MBPS,
                        .Pin_SCK                = SPIM0_SCK_PIN,
                        .Pin_MOSI               = SPIM0_MOSI_PIN,
                        .Pin_MISO               = SPIM0_MISO_PIN,
                        .Pin_CSN                = SPIM0_SS_PIN};

void LIS2DH_init(void) {
	spi_master_init2(SPI0, &my_spi);
    
	  nrf_delay_ms(1);
	  //default values:
    //LIS2DH_writeRegister(LIS2DH_CTRL_REG1,0x07);

	  LIS2DH_writeRegister(LIS2DH_CTRL_REG1,0x00);
  	LIS2DH_writeRegister(LIS2DH_CTRL_REG2,0x00);
	  LIS2DH_writeRegister(LIS2DH_CTRL_REG3,0x00);
	  LIS2DH_writeRegister(LIS2DH_CTRL_REG4,0x00);	
	  LIS2DH_writeRegister(LIS2DH_CTRL_REG5,0x00);	
	  LIS2DH_writeRegister(LIS2DH_CTRL_REG6,0x00);	
	  LIS2DH_writeRegister(LIS2DH_FIFO_CTRL_REG,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT1_CFG,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT1_SOURCE,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT1_THS,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT1_DURATION,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT2_CFG,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT2_SOURCE,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT2_THS,0x00);
	  LIS2DH_writeRegister(LIS2DH_INT2_DURATION,0x00);
    LIS2DH_writeRegister(LIS2DH_CLICK_CFG,0x00);
    LIS2DH_writeRegister(LIS2DH_CLICK_SRC,0x00);
	  LIS2DH_writeRegister(LIS2DH_CLICK_THS,0x00);
    LIS2DH_writeRegister(LIS2DH_TIME_LIMIT,0x00);
	  LIS2DH_writeRegister(LIS2DH_TIME_LATENCY,0x00);
	  nrf_delay_ms(1);
		
}

void LIS2DH_writeRegister(const uint8_t register_addr, const uint8_t value) {
	
uint8_t tx_data[2];
tx_data[0]=register_addr&0x7F;  //write requires 1 the highest
tx_data[1]=value;
spi_master_tx2(SPI0, 2, tx_data);	
	
}

uint8_t LIS2DH_readRegister(const uint8_t register_addr) {
uint8_t tx_data[2];
uint8_t rx_data[2];
tx_data[0]=0x80|(register_addr&0x3F); //1(read)0(dont incr)11 1111/
tx_data[1]=0;
rx_data[0]=0;
rx_data[1]=0;	

spi_master_tx_rx2(SPI0, 2, tx_data, rx_data);
return   rx_data[1];	
}


uint16_t LIS2DH_readRegisters(const uint8_t msb_register, const uint8_t lsb_register) {
    uint8_t msb = LIS2DH_readRegister(msb_register);
    uint8_t lsb = LIS2DH_readRegister(lsb_register);
    return (((int16_t)msb) << 8) | lsb;
}



int16_t LIS2DH_getAxisX(void) {
	return LIS2DH_readRegisters(LIS2DH_OUT_X_H, LIS2DH_OUT_X_L);
}



int16_t LIS2DH_getAxisY(void) {
	return LIS2DH_readRegisters(LIS2DH_OUT_Y_H, LIS2DH_OUT_Y_L);
}


int16_t LIS2DH_getAxisZ(void) {
	return LIS2DH_readRegisters(LIS2DH_OUT_Z_H, LIS2DH_OUT_Z_L);
}


