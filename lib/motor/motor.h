#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor
{
public:
    Motor(int pwmPin, int IN1PIN, int IN2PIN);
    void setSpeed(int speed);
    void start(int StartSpeed, int EndSpeed, unsigned long moveTime);
    void stop();  // 新增停止函数
    void MoveForward();
    void MoveBackward();
    int getCurrentSpeed();
    bool isMoving();
    void update();

private:
    int _pwmPin;
    int _IN1PIN;
    int _IN2PIN;
    unsigned long startTime;
    unsigned long duration;
    int currentSpeed;
    int startSpeed;
    int endSpeed;
    bool isActive;
    float smoothStep(float t) { return 1.0f - (1.0f - t) * (1.0f - t); }
};

#endif