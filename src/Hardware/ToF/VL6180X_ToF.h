#pragma once

#include <cstdint>

#include <units/length.h>
#include <frc/I2C.h>

class VL6180X_ToF {
public:
   VL6180X_ToF(frc::I2C::Port port, int deviceAddress = 0x29);
   ~VL6180X_ToF();

   frc::I2C::Port getI2CPort();
   int getDeviceAddress();

   /**
    * Returns the range in meters.
    */
   units::meter_t getRange();

private:
   /**
    * Writes an 8-bit value to a 16-bit register location over I2C.
    */
   void writeRegister(uint16_t reg, uint8_t data);

   /**
    * Reads an 8-bit value from a 16-bit register location over I2C.
    */
   uint8_t readRegister(uint16_t reg);

   frc::I2C i2c;
   bool isConnected = false;

   uint8_t range = 0xFF;

   bool isMeasuring = false;
};