#ifndef SERVO_H
#define SERVO_H
#include <Arduino.h>

#define SERVO_MIN() 544
#define SERVO_MAX() 2400
#define SERVO_SMOOTH_STEP 0.5f  // 改为浮点数
#define SERVO_SMOOTH_DELAY 10
#define SERVO_DEAD_ZONE 0.1f    // 改为浮点数

class Servo {
public:
    Servo();
    uint8_t attach(int pin);
    uint8_t attach(int pin, int min, int max);
    void detach();
    void write(int angle);           // 保持整数版本
    void write(float angle);         // 新增浮点数版本
    void writeMicroseconds(int us);
    float read();                    // 改为浮点数
    int readMicroseconds();
    bool attached() const { return _pin != -1; }
    void update();

private:
    int _pin;
    int _chn;
    int _minUs;
    int _maxUs;
    float _lastAngle;               // 改为浮点数
    float _targetAngle;             // 改为浮点数
    int _lastUs;
    unsigned long _lastSmoothTime;
    static bool _chnUsed[16];
    static int _findFreeChn();
};

#endif