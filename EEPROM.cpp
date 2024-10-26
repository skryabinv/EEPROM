#include "EEPROM.h"

static EEPROM_Status decodeStatusHAL(HAL_StatusTypeDef status) {
    switch (status) {
        case HAL_OK:
            return EEPROM_Status_Sucess;
        case HAL_BUSY:
            return EEPROM_Status_Busy;
        case HAL_TIMEOUT:
            return EEPROM_Status_Timeout;
        default:        
        break;    
    }       
    return EEPROM_Status_Error; 
}

#define RETURN_IF_ERROR(halStatus) \
do { \
    auto status = decodeStatusHAL(halStatus); \
    if(status != EEPROM_Status_Sucess) { \
            return status;\
    } \
} while (0)


class EEPROM {            
    static constexpr auto sTimeout = 50;
    static constexpr auto sWriteDelay = 5;    
public:
    auto init(const EEPROM_Config& config) {   
        if(config.hI2C == nullptr || config.hCRC == nullptr || config.pageSize == 0) {
            return EEPROM_Status_NotInitialized;
        }     
        mConfig = config;
        return EEPROM_Status_Sucess;
    }   

    auto isInitialized() const {
        return mConfig.hI2C != nullptr && mConfig.hCRC != nullptr;
    }

    auto write(uint16_t page, uint8_t* buffer, uint16_t size, bool useCRC) const {               
        if(!isInitialized()) {
            return EEPROM_Status_NotInitialized;
        }    
        RETURN_IF_ERROR(writeBuffer(page, buffer, size));
        if(!useCRC) {
            return EEPROM_Status_Sucess;
        }
        RETURN_IF_ERROR(writeCRC(page + getCountOfPagesFor(size), buffer, size));                
        return EEPROM_Status_Sucess;
    }

    auto read(int16_t page, uint8_t* buffer, uint16_t size, bool useCRC) const {
        if(!isInitialized()) {
            return EEPROM_Status_NotInitialized;
        }
        RETURN_IF_ERROR(readBuffer(page, buffer, size));        
        if(!useCRC) {
            return EEPROM_Status_Sucess;
        }
        uint32_t expectedCRC{};        
        RETURN_IF_ERROR(readCRC(page + getCountOfPagesFor(size), expectedCRC));                
        if(auto actualCRC = calcCRC(buffer, size); expectedCRC != actualCRC) {
            return EEPROM_Status_InvalidCRC;
        }
        return EEPROM_Status_Sucess;
    }

    auto getCountOfPagesFor(uint16_t bufferSize) const -> uint16_t {
        return bufferSize / mConfig.pageSize + 1;
    }
    
private:    

    auto getPageMemoryAddress(uint16_t page) const -> uint16_t {
        return page * mConfig.pageSize;
    }

    template<typename IO>
    auto iterateOverPages(int16_t page, uint8_t* buffer, size_t size, IO inputOutputFunction, uint16_t delay) const {
        auto memoryAddress = getPageMemoryAddress(page);
        auto ptr = buffer;
        for(uint16_t bytesRemain = size; bytesRemain > 0;) {
            auto countOfBytesToProcess = bytesRemain > mConfig.pageSize ? mConfig.pageSize : bytesRemain;
            auto status = inputOutputFunction(mConfig.hI2C, 
                          mConfig.deviceAddress, 
                          memoryAddress, I2C_MEMADD_SIZE_16BIT, 
                          ptr, countOfBytesToProcess, 
                          sTimeout); 
            if(status != HAL_OK) {
                return status;
            }
            HAL_Delay(delay);            
            bytesRemain -= countOfBytesToProcess;
            memoryAddress += mConfig.pageSize;
            ptr += mConfig.pageSize;
        }
        return HAL_OK;
    }

    auto writeBuffer(uint16_t page, uint8_t* buffer, size_t size) const -> HAL_StatusTypeDef {
        return iterateOverPages(page, buffer, size, HAL_I2C_Mem_Write, sWriteDelay);
    }

    auto writeCRC(uint16_t page, uint8_t* buffer, size_t bufferSize) const -> HAL_StatusTypeDef {
        auto crc = calcCRC(buffer, bufferSize);                
        return HAL_I2C_Mem_Write(mConfig.hI2C, mConfig.deviceAddress, 
            getPageMemoryAddress(page),            
            I2C_MEMADD_SIZE_16BIT, 
            reinterpret_cast<uint8_t*>(&crc), sizeof(crc), 
            sTimeout);
    }

    auto readBuffer(uint16_t page, uint8_t* buffer, size_t size) const -> HAL_StatusTypeDef {
        return iterateOverPages(page, buffer, size, HAL_I2C_Mem_Read, 0);    
    }

    auto readCRC(uint16_t page, uint32_t& crc) const -> HAL_StatusTypeDef {                
        return HAL_I2C_Mem_Read(mConfig.hI2C, mConfig.deviceAddress, 
            getPageMemoryAddress(page), 
            I2C_MEMADD_SIZE_16BIT, 
            reinterpret_cast<uint8_t*>(&crc), sizeof(crc), 
            sTimeout);
    }       

    auto calcCRC(uint8_t* buffer, uint16_t bufferSize) const -> uint32_t {
        return HAL_CRC_Calculate(mConfig.hCRC,  reinterpret_cast<uint32_t*>(buffer), bufferSize / 4);
    }
    
    EEPROM_Config mConfig{nullptr, nullptr, 0xA0, 64};    
}; 

static auto sInstance = EEPROM{};

EEPROM_Config EEPROM_makeDefaultConfig(I2C_HandleTypeDef* hI2C, CRC_HandleTypeDef* hCRC) {
    return EEPROM_Config{ hI2C, hCRC, 0xA0, 64};
}

EEPROM_Status EEPROM_Init(EEPROM_Config config) {        
    return sInstance.init(config);    
}

EEPROM_Status EEPROM_Read(uint16_t page, uint8_t* bytes, uint16_t size) {   
    return sInstance.read(page, bytes, size, true);    
}

EEPROM_Status EEPROM_Write(uint16_t page, uint8_t* bytes, uint16_t size) {           
    return sInstance.write(page, bytes, size, true);    
}

uint16_t EEPROM_getBuffersPagesCount(uint16_t bufferSize) {
    // + 1 - page for CRC
    return sInstance.getCountOfPagesFor(bufferSize) + 1;
}
