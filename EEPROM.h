#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  EEPROM_Status_Sucess,    
  EEPROM_Status_NotInitialized,
  EEPROM_Status_Busy,
  EEPROM_Status_Timeout,
  EEPROM_Status_InvalidCRC,  
  EEPROM_Status_Error
} EEPROM_Status;

typedef struct {
  I2C_HandleTypeDef* hI2C;
  CRC_HandleTypeDef* hCRC;
  uint16_t deviceAddress;
  uint16_t pageSize;
} EEPROM_Config;

EEPROM_Config EEPROM_makeDefaultConfig(I2C_HandleTypeDef* hI2C, CRC_HandleTypeDef* hCRC);
EEPROM_Status EEPROM_Init(EEPROM_Config config);
EEPROM_Status EEPROM_Read(uint16_t page, uint8_t* bytes, uint16_t size);
EEPROM_Status EEPROM_Write(uint16_t page, uint8_t* bytes, uint16_t size);
uint16_t EEPROM_getBuffersPagesCount(uint16_t bufferSize);

#ifdef __cplusplus
}
#endif

