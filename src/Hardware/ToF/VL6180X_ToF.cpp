#include <Hardware/ToF/VL6180X_ToF.h>

#define REG_IDENTIFICATION_MODEL_ID 0x00
#define REG_SYSTEM_FRESH_OUT_OF_RESET 0x0016
#define REG_RESULT_RANGE_STATUS 0x004d
#define REG_RESULT_INTERRUPT_STATUS_GPIO 0x004f
#define REG_SYSTEM_INTERRUPT_CLEAR 0x0015
#define REG_SYSRANGE_START 0x0018
#define REG_RESULT_RANGE_VAL 0x0062

VL6180X_ToF::VL6180X_ToF(frc::I2C::Port port, int deviceAddress)
: i2c(port, deviceAddress) {

    uint8_t buf[1];
    *buf = 0xFF;

    // Check model id.
    i2c.Read(REG_IDENTIFICATION_MODEL_ID, 1, buf);
    if (*buf != 0xB4) {
        return;
    }

    // Read SYSTEM__FRESH_OUT_OF_RESET register.
    i2c.Read(REG_SYSTEM_FRESH_OUT_OF_RESET, 1, buf);
    // Verify that the sensor is in idle mode and working as expected.
    if (!(*buf & 0x01)) {
        // Ideally we'd reset the device by applying logic '0' to GPIO0, but we can't without that wired up.
        return;
    }

    // Clear the reset bit to start using the sensor.
    i2c.Write(REG_SYSTEM_FRESH_OUT_OF_RESET, 0x00);

    isConnected = true;

    // Apply the tuning settings.
    i2c.Write(0x0207, 0x01);
    i2c.Write(0x0208, 0x01);
    i2c.Write(0x0096, 0x00);
    i2c.Write(0x0097, 0xfd);
    i2c.Write(0x00e3, 0x00);
    i2c.Write(0x00e4, 0x04);
    i2c.Write(0x00e5, 0x02);
    i2c.Write(0x00e6, 0x01);
    i2c.Write(0x00e7, 0x03);
    i2c.Write(0x00f5, 0x02);
    i2c.Write(0x00d9, 0x05);
    i2c.Write(0x00db, 0xce);
    i2c.Write(0x00dc, 0x03);
    i2c.Write(0x00dd, 0xf8);
    i2c.Write(0x009f, 0x00);
    i2c.Write(0x00a3, 0x3c);
    i2c.Write(0x00b7, 0x00);
    i2c.Write(0x00bb, 0x3c);
    i2c.Write(0x00b2, 0x09);
    i2c.Write(0x00ca, 0x09);
    i2c.Write(0x0198, 0x01);
    i2c.Write(0x01b0, 0x17);
    i2c.Write(0x01ad, 0x00);
    i2c.Write(0x00ff, 0x05);
    i2c.Write(0x0100, 0x05);
    i2c.Write(0x0199, 0x05);
    i2c.Write(0x01a6, 0x1b);
    i2c.Write(0x01ac, 0x3e);
    i2c.Write(0x01a7, 0x1f);
    i2c.Write(0x0030, 0x00);

    i2c.Write(0x0011, 0x10);
    i2c.Write(0x010a, 0x30);
    i2c.Write(0x003f, 0x46);
    i2c.Write(0x0031, 0xFF);
    i2c.Write(0x0041, 0x63);
    i2c.Write(0x002e, 0x01);
    i2c.Write(0x001b, 0x09);
    i2c.Write(0x003e, 0x31);
    i2c.Write(0x0014, 0x24);
}

VL6180X_ToF::~VL6180X_ToF() {

}

frc::I2C::Port VL6180X_ToF::getI2CPort() {
    return i2c.GetPort();
}

int VL6180X_ToF::getDeviceAddress() {
    return i2c.GetDeviceAddress();
}

units::meter_t VL6180X_ToF::getRange() {
    if (!isConnected) return 42_m;

    uint8_t buf[1];

    *buf = 0x00;
    do {
        // Read range status register.
        i2c.Read(REG_RESULT_RANGE_STATUS, 1, buf);
    } while (!(*buf & 0x01));

    // Start a range measurement.
    i2c.Write(REG_SYSRANGE_START, 0x01);

    *buf = 0x00;
    do {
        // Read interrupt status register.
        i2c.Read(REG_RESULT_INTERRUPT_STATUS_GPIO, 1, buf);
    } while (!(*buf & 0x04));

    *buf = 0xFF;
    i2c.Read(REG_RESULT_RANGE_VAL, 1, buf);

    units::millimeter_t range(*buf);

    // Clear the interrupt.
    i2c.Write(REG_SYSTEM_INTERRUPT_CLEAR, 0x07);

    return range;
}
