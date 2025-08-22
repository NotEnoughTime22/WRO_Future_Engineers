#ifndef EVO_H
#define EVO_H

#include <X1pins.h>
#include <BQ25887.h>
#include <Wire.h>
#include <Tones.h>
#include <SSD1306Wire.h>

class EVO
{
private:
  // Private constructor to prevent instantiation

  BQ25887 charger = BQ25887();
  SSD1306Wire display = SSD1306Wire(SSD1306_ADDR, SDA_PIN, SCL_PIN);
  uint8_t _channel = 0;

  // Delete the copy constructor and assignment operator
  EVO(const EVO &) = delete;
  EVO &operator=(const EVO &) = delete;

public:
  // Public method for demonstration
  EVO()
  {
  }

  static EVO &getInstance()
  {
    static EVO instance;
    return instance;
  }
  void begin(int sda = SDA_PIN, int scl = SCL_PIN);
  void selectI2CChannel(uint8_t channel);

  float getBattery();
  void playTone(uint frequency, int duration);

  // display
  void beginDisplay();
  void clearDisplay();
  void writeToDisplay(int v, int row, bool clear = false);
  void writeToDisplay(float f, int row, bool clear = false);
  void writeToDisplay(const char *c, int row, bool clear = false);
};

// EVO Evo;
#endif