#include <Arduino.h>
#include <BluetoothSerial.h>
#include "Servo.h"
#include "motor.h"

// ==================== 引脚定义 ====================
#define IR_OutLeftPin 32  // 左侧外部传感器
#define IR_InLeftPin 33   // 左侧内部传感器
#define IR_InRightPin 34  // 右侧内部传感器
#define IR_OutRightPin 35 // 右侧外部传感器
#define SERVO_PIN 27      // 舵机引脚

// ==================== 优化参数配置 ====================
#define TRACK_INTERVAL 50    // 循迹修正间隔（毫秒）大幅缩短
#define BASE_SPEED 180       // 循迹基础速度（0-255）
#define CURVE_SPEED 150      // 弯道速度
#define SHARP_TURN_SPEED 120 // 急弯速度

// PID控制参数（简单比例控制）
#define KP 2.5 // 比例系数
#define KD 0.8 // 微分系数

// 舵机参数
#define SERVO_CENTER 80     // 舵机中位角度
#define SERVO_MIN 60        // 舵机最小安全角度
#define SERVO_MAX 100       // 舵机最大安全角度
#define MAX_ANGLE_CHANGE 15 // 单次最大角度变化量

// ==================== 全局对象 ====================
Motor motor(5, 18, 19);   // 电机对象（PWM, IN1, IN2）
Servo myservo;            // 舵机对象
BluetoothSerial SerialBT; // 蓝牙对象

// ==================== 全局状态变量 ====================
bool isTracking = false;              // 循迹状态标志
int currentServoAngle = SERVO_CENTER; // 当前舵机角度
int lastError = 0;                    // 上次误差（用于微分项）

// ==================== 函数声明 ====================
void moveStraight(int speed);
void stopMotor();
void proccessBluetoothCommands(String command);
bool isAngleCommand(String cmd);
void setServoAngle(int angle);
void showHelp();
String sendIRSensorData();
void sendIRtoBluetooth();
void smoothLineTracking(int ir1, int ir2, int ir3, int ir4);

// ==================== 初始化 ====================
void setup()
{
  Serial.begin(115200);

  // 蓝牙初始化
  SerialBT.begin("MyCar");
  Serial.printf("蓝牙设备名：%s，等待连接...\n", "MyCar");

  // 红外传感器引脚模式
  pinMode(IR_OutLeftPin, INPUT);
  pinMode(IR_InLeftPin, INPUT);
  pinMode(IR_InRightPin, INPUT);
  pinMode(IR_OutRightPin, INPUT);

  // 舵机初始化
  myservo.attach(SERVO_PIN);
  delay(200);
  setServoAngle(SERVO_CENTER);

  // 电机低负载启动测试
  moveStraight(180);
  delay(200);
  stopMotor();

  Serial.println("系统就绪！发送 START 开始循迹");
}

// ==================== 主循环 ====================
void loop()
{
  // 舵机无阻塞更新
  myservo.update();

  // 处理蓝牙命令
  if (SerialBT.available())
  {
    String command = SerialBT.readStringUntil('\n');
    command.trim();
    Serial.printf("收到蓝牙命令：%s\n", command.c_str());
    proccessBluetoothCommands(command);
  }

  // 执行优化的循迹控制
  if (isTracking)
  {
    int ir1 = digitalRead(IR_OutLeftPin);
    int ir2 = digitalRead(IR_InLeftPin);
    int ir3 = digitalRead(IR_InRightPin);
    int ir4 = digitalRead(IR_OutRightPin);

    smoothLineTracking(ir1, ir2, ir3, ir4);
  }

  // 定时发送红外数据（调试用）
  static unsigned long lastIRSendTime = 0;
  if (millis() - lastIRSendTime >= 1000)
  {
    lastIRSendTime = millis();
    sendIRtoBluetooth();
  }
}

// ==================== 电机控制函数 ====================
void moveStraight(int speed)
{
  speed = constrain(speed, 0, 255);
  motor.MoveForward();
  motor.setSpeed(speed);
}

void stopMotor()
{
  motor.setSpeed(0);
  Serial.println("电机停止");
}

// ==================== 蓝牙命令处理 ====================
void proccessBluetoothCommands(String command)
{
  command.trim();
  command.toUpperCase();

  // 角度命令
  if (isAngleCommand(command))
  {
    int angle = command.toInt();
    setServoAngle(angle);
    SerialBT.println("Set: " + String(angle) + "°");
  }
  // 开始循迹
  else if (command == "START")
  {
    isTracking = true;
    currentServoAngle = SERVO_CENTER;
    lastError = 0;
    moveStraight(BASE_SPEED);
    setServoAngle(SERVO_CENTER);
    SerialBT.println("开始优化循迹（50ms修正，PID控制）");
    Serial.println("蓝牙命令：开始优化循迹");
  }
  // 停止循迹
  else if (command == "STOP_TRACK" || command == "PAUSE")
  {
    isTracking = false;
    stopMotor();
    setServoAngle(SERVO_CENTER);
    SerialBT.println("停止循迹");
    Serial.println("蓝牙命令：停止循迹");
  }
  // 手动控制命令
  else if (command == "STRAIGHT" || command == "FORWARD")
  {
    SerialBT.println("前进");
    moveStraight(150);
  }
  else if (command == "STOP")
  {
    SerialBT.println("停止");
    stopMotor();
  }
  // 帮助命令
  else if (command == "HELP" || command == "?")
  {
    showHelp();
  }
  else
  {
    SerialBT.println("未知命令");
    Serial.println("蓝牙命令：未知");
  }
}

// ==================== 辅助函数 ====================
bool isAngleCommand(String cmd)
{
  for (int i = 0; i < cmd.length(); i++)
  {
    if (!isdigit(cmd[i]))
      return false;
  }
  int value = cmd.toInt();
  return (value >= SERVO_MIN && value <= SERVO_MAX);
}

void setServoAngle(int angle)
{
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  myservo.write(angle);
  currentServoAngle = angle;
  Serial.printf("设置舵机角度：%d°\n", angle);
}

void showHelp()
{
  String helpMsg = "======== 可用命令 ========\n";
  helpMsg += "START    - 开始优化循迹\n";
  helpMsg += "STOP     - 停止电机\n";
  helpMsg += "STOP_TRACK- 暂停循迹\n";
  helpMsg += "STRAIGHT - 手动直行\n";
  helpMsg += "60-100   - 设置舵机角度\n";
  helpMsg += "HELP/?   - 显示帮助\n";
  helpMsg += "========================\n";
  helpMsg += "优化特性：50ms响应，PID控制，自适应速度";

  SerialBT.println(helpMsg);
}

String sendIRSensorData()
{
  int ir1 = digitalRead(IR_OutLeftPin);
  int ir2 = digitalRead(IR_InLeftPin);
  int ir3 = digitalRead(IR_InRightPin);
  int ir4 = digitalRead(IR_OutRightPin);

  return String(ir1) + "," + String(ir2) + "," + String(ir3) + "," + String(ir4);
}

void sendIRtoBluetooth()
{
  String irData = sendIRSensorData();
  SerialBT.println("IR_DATA:" + irData);
  Serial.println("发送红外数据：" + irData);
}

// ==================== 优化的核心循迹函数 ====================
/**
 * @brief 平滑循迹控制（PID原理 + 自适应速度）
 * 传感器权重分配：[-3, -1, 1, 3] 对应 [左外, 左内, 右内, 右外]
 * 误差计算：加权求和，正值表示偏右，负值表示偏左
 */
void smoothLineTracking(int ir1, int ir2, int ir3, int ir4)
{
  if (!isTracking)
    return;

  static unsigned long lastAdjustTime = 0;
  unsigned long currentTime = millis();

  // 50ms执行一次修正（大幅提高响应频率）
  if (currentTime - lastAdjustTime < TRACK_INTERVAL)
  {
    return;
  }
  lastAdjustTime = currentTime;

  // ===== 计算当前误差 =====
  int error = 0;

  // 传感器权重分配（可根据实际效果调整）
  if (ir1 == 1)
    error -= 3; // 左外检测到：强烈左偏信号
  if (ir2 == 1)
    error -= 1; // 左内检测到：轻微左偏信号
  if (ir3 == 1)
    error += 1; // 右内检测到：轻微右偏信号
  if (ir4 == 1)
    error += 3; // 右外检测到：强烈右偏信号

  // ===== PID控制计算 =====
  int angleChange = 0;

  // 比例项
  angleChange = error * KP;

  // 微分项（抑制振荡）
  angleChange += (error - lastError) * KD;
  lastError = error;

  // 限制单次最大角度变化
  angleChange = constrain(angleChange, -MAX_ANGLE_CHANGE, MAX_ANGLE_CHANGE);

  // ===== 自适应速度控制 =====
  int currentSpeed = BASE_SPEED;

  // 根据偏差大小调整速度
  if (abs(error) >= 3)
  {
    // 急弯情况：大幅偏差，降低速度
    currentSpeed = SHARP_TURN_SPEED;
  }
  else if (abs(error) >= 1)
  {
    // 普通弯道：中等偏差，适中速度
    currentSpeed = CURVE_SPEED;
  }
  // 直线情况：保持基础速度

  // ===== 应用控制 =====
  currentServoAngle += angleChange;
  currentServoAngle = constrain(currentServoAngle, SERVO_MIN, SERVO_MAX);

  setServoAngle(currentServoAngle);
  motor.setSpeed(currentSpeed);

  // ===== 调试输出 =====
  Serial.printf("传感器:%d%d%d%d 误差:%d 角度变化:%d 舵机:%d° 速度:%d\n",
                ir1, ir2, ir3, ir4, error, angleChange, currentServoAngle, currentSpeed);

  // 蓝牙发送关键数据（可选）
  if (abs(error) >= 2)
  { // 只在较大偏差时发送，避免蓝牙拥堵
    String statusMsg = "TRACK:Err=" + String(error) + ",Ang=" + String(currentServoAngle) + ",Spd=" + String(currentSpeed);
    SerialBT.println(statusMsg);
  }
}