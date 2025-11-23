#ifndef SERVO_H
#define SERVO_H
#include <Arduino.h>

#define SERVO_MIN() 544
#define SERVO_MAX() 2400
#define SERVO_SMOOTH_STEP 1
#define SERVO_SMOOTH_DELAY 10
#define SERVO_DEAD_ZONE 1

class Servo {
public:
    Servo();
    uint8_t attach(int pin);
    uint8_t attach(int pin, int min, int max);
    void detach();
    void write(int angle);
    void writeMicroseconds(int us);
    int read();
    int readMicroseconds();
    bool attached() const { return _pin != -1; }
    void update(); // 新增无阻塞更新函数

private:
    int _pin;
    int _chn;
    int _minUs;
    int _maxUs;
    int _lastAngle;
    int _targetAngle;
    int _lastUs;
    unsigned long _lastSmoothTime; // 新增计时变量
    static bool _chnUsed[16];
    static int _findFreeChn();
};

#endif