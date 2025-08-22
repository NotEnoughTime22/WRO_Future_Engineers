#include <VL53L0X.h>

VL53L0X::VL53L0X(uint8_t i2cPort)
{
    _i2cPort = i2cPort;
}

void VL53L0X::begin()
{
    Serial.println("begining");
    EVO::getInstance().selectI2CChannel(this->_i2cPort);
    if (!this->lox.begin())
    {
        Serial.println(F("Failed to boot VL53L0X"));
        EVO::getInstance().writeToDisplay(2, 2, "tof died");
        while (true) {
            delay(10);
        }
    }
}

int VL53L0X::getDistance()
{
    EVO::getInstance().selectI2CChannel(this->_i2cPort);
    //delay(10);
    
    if (millis() - this->_lastms > 20){
        this->lox.rangingTest(&this->measure, false); // pass in 'true' to get debug data printout!
        this->_reading = this->measure.RangeMilliMeter;
        if (this->_reading < 10) this->_reading = 8191;
        this->_lastms = millis();
    }
    return this->_reading;
    
}