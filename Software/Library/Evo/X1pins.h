#ifndef X1PINS_H
#define X1PINS_H

// Lego Sensor Port
#define S11 42
#define S12 41
#define S21 39
#define S22 40
#define S31 38
#define S32 48
#define S41 47
#define S42 21

#define TACH41 6
#define TACH42 4
#define TACH31 15
#define TACH32 5
#define TACH21 16
#define TACH22 7
#define TACH11 9
#define TACH12 8
// Through SX1509
#define MOTOR11 1
#define MOTOR12 0
#define MOTOR21 3
#define MOTOR22 2
#define MOTOR31 13
#define MOTOR32 12
#define MOTOR41 15
#define MOTOR42 14

#define BUZZER_PIN 14

#define SDA_PIN 1
#define SCL_PIN 2
#define BATTERY_CHARGER_ADDRESS (0x6A)
#define TCA9548A_ADDR (0x70)
#define SX1509_ADDR (0x3E)
#define SX1509_CHANNEL 5
#define SSD1306_ADDR (0x3C)
#define SSD1306_CHANNEL 7

#define MPU9250_ADDR (0x68)
#define MPU9250_CHANNEL 6

enum MotorPort
{
    M1,
    M2,
    M3,
    M4
};

enum SensorPort
{
    S1,
    S2,
    S3,
    S4
};

enum I2CPort
{
    I2C1 = 0,
    I2C2 = 1,
    I2C3 = 2,
    I2C4 = 3,
    I2C5 = 4
};

enum EIOPin
{
    EIO1 = 4,
    EIO2,
    EIO3,
    EIO4,
    EIO5,
    EIO6,
    EIO7,
    EIO8
};

#endif