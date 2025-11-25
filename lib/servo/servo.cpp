#include "Servo.h"

bool Servo::_chnUsed[16] = {false};

Servo::Servo() : _pin(-1), _chn(-1),
                 _minUs(SERVO_MIN()), _maxUs(SERVO_MAX()),
                 _lastAngle(90.0f), _targetAngle(90.0f), // 改为浮点数
                 _lastUs(map(90, 0, 180, SERVO_MIN(), SERVO_MAX())),
                 _lastSmoothTime(0)
{
}

int Servo::_findFreeChn()
{
    for (int i = 0; i < 16; ++i)
        if (!_chnUsed[i])
            return i;
    return -1;
}

uint8_t Servo::attach(int pin)
{
    return attach(pin, SERVO_MIN(), SERVO_MAX());
}

uint8_t Servo::attach(int pin, int min, int max)
{
    if (_pin != -1)
        detach();
    int chn = _findFreeChn();
    if (chn == -1)
        return 0;

    _pin = pin;
    _chn = chn;
    _minUs = constrain(min, 500, 1000);
    _maxUs = constrain(max, 2000, 2550);
    _chnUsed[chn] = true;

    ledcSetup(chn, 50, 16);
    ledcAttachPin(pin, chn);

    _targetAngle = 82.0f; // 改为浮点数
    _lastAngle = 82.0f;   // 改为浮点数
    int initUs = map(82, 0, 180, _minUs, _maxUs);
    writeMicroseconds(initUs);
    return 1;
}

void Servo::detach()
{
    if (_pin == -1)
        return;
    ledcWrite(_chn, 0);
    ledcDetachPin(_pin);
    _chnUsed[_chn] = false;
    _pin = -1;
    _chn = -1;
}

// 整数版本（保持兼容性）
void Servo::write(int angle)
{
    write(static_cast<float>(angle)); // 调用浮点数版本
}

// 新增浮点数版本
void Servo::write(float angle)
{
    if (_pin == -1)
        return;
    angle = constrain(angle, 0.0f, 180.0f);
    if (fabs(angle - _targetAngle) <= SERVO_DEAD_ZONE)
        return;
    _targetAngle = angle;
}

void Servo::update()
{
    if (_pin == -1)
        return;
    if (fabs(_lastAngle - _targetAngle) <= SERVO_DEAD_ZONE)
    {
        _lastAngle = _targetAngle;
        int us = map(static_cast<int>(_lastAngle), 0, 180, _minUs, _maxUs);
        writeMicroseconds(us);
        return;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - _lastSmoothTime >= SERVO_SMOOTH_DELAY)
    {
        _lastSmoothTime = currentMillis;
        if (_lastAngle < _targetAngle)
            _lastAngle += SERVO_SMOOTH_STEP;
        else
            _lastAngle -= SERVO_SMOOTH_STEP;

        // 确保不会过冲
        if ((SERVO_SMOOTH_STEP > 0) &&
            fabs(_lastAngle - _targetAngle) < SERVO_SMOOTH_STEP)
        {
            _lastAngle = _targetAngle;
        }

        int us = map(static_cast<int>(_lastAngle), 0, 180, _minUs, _maxUs);
        writeMicroseconds(us);
    }
}

void Servo::writeMicroseconds(int us)
{
    if (_pin == -1)
        return;
    us = constrain(us, _minUs, _maxUs);
    _lastUs = us;
    uint32_t duty = (uint32_t)us * 65535 / 20000;
    ledcWrite(_chn, duty);
}

float Servo::read() { return _lastAngle; } // 返回浮点数
int Servo::readMicroseconds() { return _lastUs; }