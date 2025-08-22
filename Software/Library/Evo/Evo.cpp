#include <Evo.h>

void EVO::begin(int sda, int scl)
{
    Wire.begin(sda, scl);
    if (!charger.begin())
    {
        Serial.println("Charger is not working");
    }
}

void EVO::selectI2CChannel(uint8_t channel)
{
    if (_channel != channel)
    {
        Wire.beginTransmission(TCA9548A_ADDR); // TCA9548A address
        Wire.write(1 << channel);              // send byte to select bus
        Wire.endTransmission();
        _channel = channel;
    }
}
float EVO::getBattery()
{
    charger.wdReset();
    charger.setADC_EN(true);
    charger.pollAllRegs();
    return charger.getADC_VBAT();
}
void EVO::playTone(uint frequency, int duration)
{
    tone(BUZZER_PIN, frequency, duration);
}

void EVO::beginDisplay()
{
    this->selectI2CChannel(SSD1306_CHANNEL);

    display.init();
    display.clear();
    display.display();
}
void EVO::clearDisplay()
{
    this->selectI2CChannel(SSD1306_CHANNEL);
    display.clear();
    display.display();
}
void EVO::writeToDisplay(int v, int row, bool clear)
{
    this->selectI2CChannel(SSD1306_CHANNEL);
    char value[32];
    if (clear)
    {
        display.clear();
    }
    display.drawString(0, row * 8, itoa(v, value, 10));
    display.display();
}
void EVO::writeToDisplay(float f, int row, bool clear)
{
    this->selectI2CChannel(SSD1306_CHANNEL);
    char value[32];
    if (clear)
    {
        display.clear();
    }
    dtostrf(f, 4, 2, value);
    display.drawString(0, row * 8, value);
    display.display();
}
void EVO::writeToDisplay(const char *c, int row, bool clear)
{
    this->selectI2CChannel(SSD1306_CHANNEL);
    if (clear)
    {
        display.clear();
    }
    display.drawString(0, row * 8, c);
    display.display();
}
