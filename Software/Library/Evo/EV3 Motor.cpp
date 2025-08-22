#include <EV3Motor.h>

SX1509 EV3Motor::io;
bool EV3Motor::SX1509Initialized = false;

int clampp(int value, int min, int max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

EV3Motor::EV3Motor(MotorPort motorPort, bool motorFlip)
{
    _motorPort = motorPort;
    _motorFlip = motorFlip;
    switch (_motorPort)
    {
    case M1:
        if (!_motorFlip)
            motorPins = {MOTOR11, MOTOR12, TACH11, TACH12};
        else
            motorPins = {MOTOR12, MOTOR11, TACH12, TACH11};
        break;
    case M2:
        if (!_motorFlip)
            motorPins = {MOTOR21, MOTOR22, TACH21, TACH22};
        else
            motorPins = {MOTOR22, MOTOR21, TACH22, TACH21};
        break;
    case M3:
        if (!_motorFlip)
            motorPins = {MOTOR31, MOTOR32, TACH31, TACH32};
        else
            motorPins = {MOTOR32, MOTOR31, TACH32, TACH31};
        break;
    case M4:
        if (!_motorFlip)
            motorPins = {MOTOR41, MOTOR42, TACH41, TACH42};
        else
            motorPins = {MOTOR42, MOTOR41, TACH42, TACH41};
        break;
    }
}
void EV3Motor::begin()
{
    EVO::getInstance().selectI2CChannel(SX1509_CHANNEL);
    if (!SX1509Initialized)
    {
        if (io.begin(SX1509_ADDR) == false)
        {
            Serial.println("Failed to communicate. Check wiring and address of SX1509.");
        }
        else
        {
            SX1509Initialized = true;
        }
    }
    io.pinMode(motorPins.power1, ANALOG_OUTPUT);
    io.pinMode(motorPins.power2, ANALOG_OUTPUT);
    encoder.attachFullQuad(motorPins.tach1, motorPins.tach2);
    encoder.clearCount();
    xTaskCreate(motorControlTask, "Motor Control Task", 2048, this, 1, NULL);
}

void EV3Motor::move(int speed)
{
    speed = clampp(speed, -255, 255);
    EVO::getInstance().selectI2CChannel(SX1509_CHANNEL);
    if (speed > 0)
    {
        io.analogWrite(motorPins.power1, speed);
        io.analogWrite(motorPins.power2, 0);
    }
    else if (speed < 0)
    {
        io.analogWrite(motorPins.power1, 0);
        io.analogWrite(motorPins.power2, -speed);
    }
    else
    {
        io.analogWrite(motorPins.power1, 0);
        io.analogWrite(motorPins.power2, 0);
    }
}

void EV3Motor::run(int speed)
{
    this->motorState = RUN;
    this->move(speed);
}
// Method to get current angle
int EV3Motor::getAngle()
{
    return encoder.getCount();
}

// Method to reset angle
void EV3Motor::resetAngle()
{
    encoder.setCount(0);
}

void EV3Motor::setAngle(int angle)
{
    encoder.setCount(angle);
}

// Method to stop the motor
void EV3Motor::coast()
{
    this->motorState = COAST;
    this->move(0);
}

// Method to brake the motor
void EV3Motor::brake()
{
    this->resetAngle();
    this->targetAngle = 0;
    this->motorState = BRAKE;
}

// Method to run the motor for a specified number of degrees
void EV3Motor::runDegrees(int speed, int degrees)
{
    this->motorState = RUN;
    this->resetAngle();
    while (abs(this->getAngle()) < degrees)
    {
        this->move(speed);
    }
    this->brake();
}

void EV3Motor::runTarget(int speed, int degrees, bool blocking)
{
    this->targetSpeed = speed;
    this->targetAngle = degrees;
    this->completed = false;
    this->motorState = TARGET;
    if (blocking)
    {
        while ((abs(this->getAngle()) - degrees) < 2)
            ;
    }
}

void EV3Motor::motorControlTask(void *parameter)
{
    EV3Motor *motor = (EV3Motor *)parameter;
    int motorSpeed;
    int error;
    for (;;)
    {
        // if (xSemaphoreTake(motor->xSemaphore, portMAX_DELAY))
        // {
        int encoder = motor->getAngle();
        switch (motor->motorState)
        {
        case BRAKE:
            motor->move((encoder - motor->targetAngle) * -10);
            break;
        case TARGET:
            error = encoder - motor->targetAngle;
            motorSpeed = clampp(error * -10, motor->targetSpeed * -1, motor->targetSpeed);
            motor->move(motorSpeed);
            break;
        case COAST:
        case RUN:
            break;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
        // xSemaphoreGive(motor->xSemaphore);
        // }
    }
}