#include "EEPROM.h"

#define RETURN_IF_ERROR(halStatus) \
do { \
    auto error = decodeStatus(halStatus); \
    if(error != EEPROM_Status_Sucess) { \
            return error;\
    } \
} while (0)


class EEPROM {            
    static constexpr auto sTimeout = 50;
    static constexpr auto sWriteDelay = 5;    
public:
    auto init(I2C_HandleTypeDef* i2c, CRC_HandleTypeDef* crc, uint16_t deviceAddress, uint16_t pageSize) {
        mI2C = i2c;
        mCRC = crc;
        mPageSize = pageSize;
        mDeviceAddress = deviceAddress;
    }   

    auto isInitialized() const {
        return mI2C != nullptr && mCRC != nullptr;
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
        return bufferSize / mPageSize + 1;
    }
    
private:    

    auto getPageMemoryAddress(uint16_t page) const -> uint16_t {
        return page * mPageSize;
    }

    template<uint16_t delay, typename IO>
    auto iterateOverPages(int16_t page, uint8_t* buffer, size_t size, IO inputOutputFunction) const {
        auto memoryAddress = getPageMemoryAddress(page);
        auto ptr = buffer;
        for(uint16_t bytesRemain = size; bytesRemain > 0;) {
            auto countOfBytesToProcess = bytesRemain > mPageSize ? mPageSize : bytesRemain;
            auto status = inputOutputFunction(mI2C, 
                          mDeviceAddress, 
                          memoryAddress, I2C_MEMADD_SIZE_16BIT, 
                          ptr, countOfBytesToProcess, 
                          sTimeout); 
            if(status != HAL_OK) {
                return status;
            }
            if constexpr(delay != 0) {
                HAL_Delay(sWriteDelay);
            }
            bytesRemain -= countOfBytesToProcess;
            memoryAddress += mPageSize;
            ptr += mPageSize;
        }
        return HAL_OK;
    }

    auto writeBuffer(uint16_t page, uint8_t* buffer, size_t size) const -> HAL_StatusTypeDef {
        return iterateOverPages<sWriteDelay>(page, buffer, size, HAL_I2C_Mem_Write);
    }

    auto writeCRC(uint16_t page, uint8_t* buffer, size_t bufferSize) const -> HAL_StatusTypeDef {
        auto crc = calcCRC(buffer, bufferSize);                
        return HAL_I2C_Mem_Write(mI2C, mDeviceAddress, 
            getPageMemoryAddress(page),            
            I2C_MEMADD_SIZE_16BIT, 
            reinterpret_cast<uint8_t*>(&crc), sizeof(crc), 
            sTimeout);
    }

    auto readBuffer(uint16_t page, uint8_t* buffer, size_t size) const -> HAL_StatusTypeDef {
        return iterateOverPages<0>(page, buffer, size, HAL_I2C_Mem_Read);    
    }

    auto readCRC(uint16_t page, uint32_t& crc) const -> HAL_StatusTypeDef {                
        return HAL_I2C_Mem_Read(mI2C, mDeviceAddress, 
            getPageMemoryAddress(page), 
            I2C_MEMADD_SIZE_16BIT, 
            reinterpret_cast<uint8_t*>(&crc), sizeof(crc), 
            sTimeout);
    }    

    static EEPROM_Status decodeStatus(HAL_StatusTypeDef status) {
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

    auto calcCRC(uint8_t* buffer, uint16_t bufferSize) const -> uint32_t {
        return HAL_CRC_Calculate(mCRC,  reinterpret_cast<uint32_t*>(buffer), bufferSize / 4);
    }

    I2C_HandleTypeDef* mI2C{nullptr};
    CRC_HandleTypeDef* mCRC{nullptr};
    uint16_t mPageSize{64};
    uint16_t mDeviceAddress{0b10100000};
}; 

static auto sInstance = EEPROM{};

EEPROM_Status EEPROM_Init(I2C_HandleTypeDef * i2c, CRC_HandleTypeDef* crc, uint16_t deviceAddress, uint16_t pageSize) {    
    sInstance.init(i2c, crc, deviceAddress, pageSize);
    return EEPROM_Status_Sucess;
}

EEPROM_Status EEPROM_Read(uint16_t page, uint8_t* bytes, uint16_t size) {   
    return sInstance.read(page, bytes, size, true);    
}

EEPROM_Status EEPROM_Write(uint16_t page, uint8_t* bytes, uint16_t size) {           
    return sInstance.write(page, bytes, size, true);    
}

uint16_t EEPROM_getBuffersPageCount(uint16_t bufferSize) {
    return sInstance.getCountOfPagesFor(bufferSize);
}
