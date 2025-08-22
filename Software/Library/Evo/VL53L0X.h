#ifndef VL53L0X_H
#define VL53L0X_H

#include <Evo.h>
#include <Adafruit_VL53L0X.h>

class VL53L0X
{
private:
    Adafruit_VL53L0X lox;
    uint8_t _i2cPort;
    VL53L0X_RangingMeasurementData_t measure;
    uint32_t _lastms = 0;
    int _reading = 0;

public:
    // Constructor with port and positive direction
    VL53L0X(uint8_t i2cPort);
    void begin();
    int getDistance();
};
#endif