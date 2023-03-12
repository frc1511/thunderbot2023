#include <Hardware/ToF/VL6180X_ToF.h>
#include <fmt/core.h>
#include <frc/Errors.h>

#define REG_IDENTIFICATION_MODEL_ID 0x00
#define REG_SYSTEM_FRESH_OUT_OF_RESET 0x0016
#define REG_RESULT_RANGE_STATUS 0x004d
#define REG_RESULT_INTERRUPT_STATUS_GPIO 0x004f
#define REG_SYSTEM_INTERRUPT_CLEAR 0x0015
#define REG_SYSRANGE_START 0x0018
#define REG_RESULT_RANGE_VAL 0x0062

VL6180X_ToF::VL6180X_ToF(frc::I2C::Port port, int deviceAddress)
: i2c(port, deviceAddress) {
    initThings();
    measurementTimer.Reset();
}

VL6180X_ToF::~VL6180X_ToF() { }

frc::I2C::Port VL6180X_ToF::getI2CPort() {
    return i2c.GetPort();
}

int VL6180X_ToF::getDeviceAddress() {
    return i2c.GetDeviceAddress();
}

units::meter_t VL6180X_ToF::getRange() {
    return 255_mm;
    if (!isConnected) return 42_m;

    if (readRegister(REG_SYSTEM_FRESH_OUT_OF_RESET) & 0x01) {
        FRC_ReportError(42, "VL6180X_ToF SYSTEM__FRESH_OUT_OF_RESET");
        initThings();
        writeRegister(REG_SYSRANGE_START, 0x00);
    }

    if (!isMeasuring) {
        uint8_t status = readRegister(REG_RESULT_RANGE_STATUS);
        // Read range status register.
        if ((status & 0x01) == 0x01) {
            // Start a range measurement.
            writeRegister(REG_SYSRANGE_START, 0x01);
            isMeasuring = true;
            measurementTimer.Reset();
            measurementTimer.Start();
        }
        else {
            FRC_ReportError(42, "VL6180X_ToF STATUS ERROR {}", status);

        }
    }

    if (isMeasuring) {
        // Read interrupt status register.
        if ((readRegister(REG_RESULT_INTERRUPT_STATUS_GPIO) & 0x04) == 0x04) {
            // Read range register (millimeters).
            range = readRegister(REG_RESULT_RANGE_VAL);

            // Clear the interrupt.
            writeRegister(REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

            isMeasuring = false;
        }
        else if (measurementTimer.Get() > 0.5_s) {
            // AAAARRRGGGHHHHH!!!!
            initThings();
            writeRegister(REG_SYSRANGE_START, 0x00);
            measurementTimer.Reset();
            measurementTimer.Stop();
            // frc::ReportError()
            FRC_ReportError(42, "VL6180X_ToF Timed out reading measurement, resetting");
            return 42_m;
        }
    }

    // Hi Byers!!!

    // Return the most recent range measurement.
    return units::millimeter_t(range);
}

void VL6180X_ToF::writeRegister(uint16_t reg, uint8_t data) {
    uint8_t buffer[3];

    buffer[0] = uint8_t(reg >> 8);
    buffer[1] = uint8_t(reg & 0xFF);
    buffer[2] = data;

    i2c.WriteBulk(buffer, 3);
}

uint8_t VL6180X_ToF::readRegister(uint16_t reg) {
    uint8_t buffer[3];

    // Prompt the sensor to read the register.
    buffer[0] = uint8_t(reg >> 8);
    buffer[1] = uint8_t(reg & 0xFF);
    i2c.WriteBulk(buffer, 2);

    // Read the register.
    bool res = i2c.ReadOnly(1, &buffer[2]);
    if (res) {
        FRC_ReportError(42, "Read timed out!");
    }

    return buffer[2];
}

void VL6180X_ToF::initThings() {
    // Read SYSTEM_FRESH_OUT_OF_RESET register.
    if (readRegister(REG_SYSTEM_FRESH_OUT_OF_RESET) & 0x01) {
        // Clear the reset bit to start using the sensor.
        writeRegister(REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);
    }

    isMeasuring = false;
    isConnected = true;

    // Apply the tuning settings.
    writeRegister(0x0207, 0x01);
    writeRegister(0x0208, 0x01);
    writeRegister(0x0096, 0x00);
    writeRegister(0x0097, 0xfd);
    writeRegister(0x00e3, 0x00);
    writeRegister(0x00e4, 0x04);
    writeRegister(0x00e5, 0x02);
    writeRegister(0x00e6, 0x01);
    writeRegister(0x00e7, 0x03);
    writeRegister(0x00f5, 0x02);
    writeRegister(0x00d9, 0x05);
    writeRegister(0x00db, 0xce);
    writeRegister(0x00dc, 0x03);
    writeRegister(0x00dd, 0xf8);
    writeRegister(0x009f, 0x00);
    writeRegister(0x00a3, 0x3c);
    writeRegister(0x00b7, 0x00);
    writeRegister(0x00bb, 0x3c);
    writeRegister(0x00b2, 0x09);
    writeRegister(0x00ca, 0x09);
    writeRegister(0x0198, 0x01);
    writeRegister(0x01b0, 0x17);
    writeRegister(0x01ad, 0x00);
    writeRegister(0x00ff, 0x05);
    writeRegister(0x0100, 0x05);
    writeRegister(0x0199, 0x05);
    writeRegister(0x01a6, 0x1b);
    writeRegister(0x01ac, 0x3e);
    writeRegister(0x01a7, 0x1f);
    writeRegister(0x0030, 0x00);

    writeRegister(0x0011, 0x10);
    writeRegister(0x010a, 0x30);
    writeRegister(0x003f, 0x46);
    writeRegister(0x0031, 0xFF);
    writeRegister(0x0041, 0x63);
    writeRegister(0x002e, 0x01);
    writeRegister(0x001b, 0x09);
    writeRegister(0x003e, 0x31);
    writeRegister(0x0014, 0x24);

    // Hi Jeff!!!!
}
