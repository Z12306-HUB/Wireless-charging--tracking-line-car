#include "motor.h"

Motor::Motor(int pwmPin, int IN1PIN, int IN2PIN) {
    _pwmPin = pwmPin;
    _IN1PIN = IN1PIN;
    _IN2PIN = IN2PIN;
    
    pinMode(_pwmPin, OUTPUT);
    pinMode(_IN1PIN, OUTPUT);
    pinMode(_IN2PIN, OUTPUT);
    
    currentSpeed = 0;
    startSpeed = 0;
    endSpeed = 0;
    duration = 0;
    isActive = false;
    
    // 初始停止
    digitalWrite(_IN1PIN, LOW);
    digitalWrite(_IN2PIN, LOW);
    analogWrite(_pwmPin, 0);
}

void Motor::setSpeed(int speed) {
    speed = constrain(speed, 0, 255);
    currentSpeed = speed;
    analogWrite(_pwmPin, currentSpeed);
}

void Motor::stop() {
    digitalWrite(_IN1PIN, LOW);
    digitalWrite(_IN2PIN, LOW);
    setSpeed(0);
    isActive = false;
}

void Motor::MoveForward() {
    digitalWrite(_IN1PIN, HIGH);
    digitalWrite(_IN2PIN, LOW);
}

void Motor::MoveBackward() {
    digitalWrite(_IN1PIN, LOW);
    digitalWrite(_IN2PIN, HIGH);
}

void Motor::start(int StartSpeed, int EndSpeed, unsigned long moveTime) {
    // 如果正在运行，先停止
    if(isActive) {
        startSpeed = currentSpeed;
    }else{
        startSpeed = constrain(StartSpeed, 0, 255);
    }
    endSpeed = constrain(EndSpeed, 0, 255);
    startTime = millis();
    duration = moveTime;
    currentSpeed = startSpeed;
    isActive = true;
    
    // 设置方向并启动
    MoveForward();  // 在运动开始时设置方向
    
    Serial.print("Motor start: "); Serial.print(startSpeed);
    Serial.print(" -> "); Serial.print(endSpeed);
    Serial.print(" in "); Serial.print(moveTime);
    Serial.println(" ms");
}

void Motor::update() {
    if (!isActive) return;

    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - startTime;

    if (elapsedTime >= duration) {
        currentSpeed = endSpeed;
        analogWrite(_pwmPin, currentSpeed);
        isActive = false;
        Serial.println("Motor completed");
        return;
    }

    float t = static_cast<float>(elapsedTime) / duration;
    float smoothT = smoothStep(t);
    int newSpeed = startSpeed + static_cast<int>((endSpeed - startSpeed) * smoothT);
    currentSpeed = constrain(newSpeed, 0, 255);
    analogWrite(_pwmPin, currentSpeed);
}

int Motor::getCurrentSpeed() {
    return currentSpeed;
}

bool Motor::isMoving() {
    return isActive;
}