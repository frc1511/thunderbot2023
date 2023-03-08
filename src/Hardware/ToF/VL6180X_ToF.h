#pragma once

#include <cstdint>

#include <units/length.h>
#include <frc/I2C.h>

class VL6180X_ToF {
public:
   VL6180X_ToF(frc::I2C::Port port = frc::I2C::Port::kMXP, int deviceAddress = 0x29);
   ~VL6180X_ToF();

   frc::I2C::Port getI2CPort();
   int getDeviceAddress();

   units::meter_t getRange();

private:
   void writeRegister(uint16_t reg, uint8_t data);
   uint8_t readRegister(uint16_t reg);

   frc::I2C i2c;
   bool isConnected = false;
};