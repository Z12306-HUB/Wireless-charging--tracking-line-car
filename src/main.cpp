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
#define TRACK_INTERVAL 30

// PID控制参数
#define KP 0.3f
#define KD 0.8f

// 传感器滤波参数
#define SENSOR_FILTER_SAMPLES 3 // 传感器滤波采样次数
static int sensorHistory[4][SENSOR_FILTER_SAMPLES] = {0};
static int historyIndex = 0;

// 舵机参数
#define SERVO_CENTER 84.5f
#define SERVO_MIN 70.0f
#define SERVO_MAX 100.0f
#define MAX_ANGLE_CHANGE 5.0f

// 速度参数 - 修改为启动加速模式
#define STARTUP_DURATION 1000 // 启动加速期2秒
#define BOOST_SPEED 210       // 新增：启动加速速度（较大）
#define CRUISE_SPEED 195      // 新增：巡航稳定速度（稍低）
#define CURVE_SPEED 190
#define SHARP_TURN_SPEED 190

// ==================== 全局对象 ====================
Motor motor(5, 18, 19);   // 电机对象（PWM, IN1, IN2）
Servo myservo;            // 舵机对象
BluetoothSerial SerialBT; // 蓝牙对象

// ==================== 全局状态变量 ====================
bool isTracking = false;
float currentServoAngle = SERVO_CENTER;
float lastError = 0;
unsigned long trackingStartTime = 0;

// ==================== 函数声明 ====================
void moveStraight(int speed);
void stopMotor();
void proccessBluetoothCommands(String command);
bool isAngleCommand(String cmd);
bool isFloatAngleCommand(String cmd); // 新增浮点数命令检测
void setServoAngle(float angle);      // 改为浮点数参数
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
  String originalCommand = command; // 保存原始命令
  command.trim();
  command.toUpperCase();

  // 浮点数角度命令（支持小数点）
  if (isFloatAngleCommand(originalCommand))
  {
    float angle = originalCommand.toFloat();
    setServoAngle(angle);
    SerialBT.println("Set: " + String(angle, 1) + "°"); // 显示1位小数
  }
  // 整数角度命令（向后兼容）
  else if (isAngleCommand(command))
  {
    float angle = command.toFloat();
    setServoAngle(angle);
    SerialBT.println("Set: " + String(angle, 1) + "°");
  }
  // 开始循迹
  else if (command == "START")
  {
    isTracking = true;
    currentServoAngle = SERVO_CENTER;
    lastError = 0;
    trackingStartTime = millis();

    // 启动时使用较大速度快速提速
    moveStraight(BOOST_SPEED); // 修改：使用BOOST_SPEED
    setServoAngle(SERVO_CENTER);
    SerialBT.println("开始优化循迹（启动加速模式）");
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
// 整数角度命令检测（向后兼容）
bool isAngleCommand(String cmd)
{
  for (int i = 0; i < cmd.length(); i++)
  {
    if (!isdigit(cmd[i]))
      return false;
  }
  float value = cmd.toFloat();
  return (value >= SERVO_MIN && value <= SERVO_MAX);
}

// 浮点数角度命令检测（新增）
bool isFloatAngleCommand(String cmd)
{
  bool hasDecimalPoint = false;
  for (int i = 0; i < cmd.length(); i++)
  {
    if (cmd[i] == '.')
    {
      if (hasDecimalPoint)
        return false; // 多个小数点
      hasDecimalPoint = true;
    }
    else if (!isdigit(cmd[i]))
    {
      return false;
    }
  }
  float value = cmd.toFloat();
  return (value >= SERVO_MIN && value <= SERVO_MAX);
}

// 设置舵机角度（改为浮点数）
void setServoAngle(float angle)
{
  angle = constrain(angle, SERVO_MIN, SERVO_MAX);
  myservo.write(angle); // 使用浮点数版本的write
  currentServoAngle = angle;
  Serial.printf("设置舵机角度：%.1f°\n", angle); // 浮点数格式输出
}

void showHelp()
{
  String helpMsg = "======== 可用命令 ========\n";
  helpMsg += "START    - 开始优化循迹\n";
  helpMsg += "STOP     - 停止电机\n";
  helpMsg += "STOP_TRACK- 暂停循迹\n";
  helpMsg += "STRAIGHT - 手动直行\n";
  helpMsg += "60-110   - 设置舵机角度（整数）\n";
  helpMsg += "85.5     - 设置舵机角度（浮点数）\n";
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
 * @brief 平滑循迹控制（PID原理 + 自适应速度 + 启动加速）
 * 传感器权重分配：[-3, -1, 1, 3] 对应 [左外, 左内, 右内, 右外]
 * 速度控制：启动加速 → 稳定巡航
 */
void smoothLineTracking(int ir1, int ir2, int ir3, int ir4)
{
  if (!isTracking)
    return;

  static unsigned long lastAdjustTime = 0;
  unsigned long currentTime = millis();

  // 50ms执行一次修正
  if (currentTime - lastAdjustTime < TRACK_INTERVAL)
  {
    return;
  }
  lastAdjustTime = currentTime;

  // ===== 传感器数据滤波 =====

  // 更新传感器历史数据
  sensorHistory[0][historyIndex] = ir1;
  sensorHistory[1][historyIndex] = ir2;
  sensorHistory[2][historyIndex] = ir3;
  sensorHistory[3][historyIndex] = ir4;

  historyIndex = (historyIndex + 1) % SENSOR_FILTER_SAMPLES;

  // 计算滤波后的传感器值（多数表决）
  int filteredIR[4];
  for (int i = 0; i < 4; i++)
  {
    int sum = 0;
    for (int j = 0; j < SENSOR_FILTER_SAMPLES; j++)
    {
      sum += sensorHistory[i][j];
    }
    filteredIR[i] = (sum > SENSOR_FILTER_SAMPLES / 2) ? 1 : 0;
  }

  // 使用滤波后的数据
  ir1 = filteredIR[0];
  ir2 = filteredIR[1];
  ir3 = filteredIR[2];
  ir4 = filteredIR[3];

  // ===== 计算当前误差 =====
  float error = 0.0f;

  // 传感器权重分配
  if (ir1 == 1)
    error += 2.0f; // 左外检测到：强烈左偏信号
  if (ir2 == 1)
    error += 1.0f; // 左内检测到：轻微左偏信号
  if (ir3 == 1)
    error -= 1.0f; // 右内检测到：轻微右偏信号
  if (ir4 == 1)
    error -= 2.0f; // 右外检测到：强烈右偏信号

  // ===== 丢失路线处理 =====
  static unsigned long lostLineTime = 0;
  if (ir1 == 0 && ir2 == 0 && ir3 == 0 && ir4 == 0)
  {
    // 所有传感器都看不到线
    if (lostLineTime == 0)
    {
      lostLineTime = currentTime;
    }

    // 如果持续丢失路线超过2秒，停车
    if (currentTime - lostLineTime > 2000)
    {
      SerialBT.println("严重：持续丢失路线，停车！");
      stopMotor();
      isTracking = false;
      lostLineTime = 0; // 重置计时器
      return;
    }
    else
    {
      // 暂时丢失路线，保持当前方向
      error = 0; // 不调整方向，保持直线
      SerialBT.println("警告：暂时丢失路线，保持方向");
    }
  }
  else
  {
    // 重置丢失路线计时器
    lostLineTime = 0;
  }

  // ===== PID控制计算 =====
  float angleChange = 0.0f;

  // 比例项（主要控制）
  angleChange = error * KP;

  // 微分项（抑制振荡的关键）
  static float lastError2 = 0; // 上上次误差
  float derivative = (error - lastError) + (lastError - lastError2);
  angleChange += derivative * KD * 0.5f;

  // 更新误差历史
  lastError2 = lastError;
  lastError = error;

  // 死区控制 - 微小误差不响应
  if (fabs(error) < 0.3f)
  {
    angleChange = 0;
  }

  // 限制单次最大角度变化
  angleChange = constrain(angleChange, -MAX_ANGLE_CHANGE, MAX_ANGLE_CHANGE);

  // ===== 启动加速后稳定降速控制 =====
  int currentSpeed;
  unsigned long trackingDuration = currentTime - trackingStartTime;

  if (trackingDuration < STARTUP_DURATION)
  {
    // 启动加速期内使用较大速度快速提速
    currentSpeed = BOOST_SPEED;

    // 在加速期最后阶段开始平滑降速
    if (trackingDuration > STARTUP_DURATION * 0.8f)
    {
      // 最后20%时间开始平滑降速到巡航速度
      float progress = (float)(trackingDuration - STARTUP_DURATION * 0.8f) / (STARTUP_DURATION * 0.2f);
      currentSpeed = BOOST_SPEED + (CRUISE_SPEED - BOOST_SPEED) * progress;

      // 限制在合理范围内
      currentSpeed = constrain(currentSpeed, CRUISE_SPEED, BOOST_SPEED);
    }
  }
  else
  {
    // 加速期结束后使用较低的巡航速度
    currentSpeed = CRUISE_SPEED;
  }

  // 根据偏差进一步调整速度
  if (fabs(error) >= 1.5f)
  {
    currentSpeed = min(currentSpeed, SHARP_TURN_SPEED); // 急弯降速
  }
  else if (fabs(error) >= 0.8f)
  {
    currentSpeed = min(currentSpeed, CURVE_SPEED); // 弯道适中速度
  }

  // 确保速度在安全范围内
  currentSpeed = constrain(currentSpeed, 80, 255);

  // ===== 应用控制 =====
  currentServoAngle += angleChange;
  currentServoAngle = constrain(currentServoAngle, SERVO_MIN, SERVO_MAX);

  setServoAngle(currentServoAngle);
  motor.setSpeed(currentSpeed);

  // ===== 状态提示 =====
  static bool speedReduced = false;
  if (!speedReduced && trackingDuration >= STARTUP_DURATION)
  {
    speedReduced = true;
    SerialBT.println("启动加速完成，进入巡航模式");
  }

  // ===== 调试输出 =====
  static unsigned long lastPrintTime = 0;
  if (currentTime - lastPrintTime >= 500)
  { // 每500ms输出一次，避免太频繁
    lastPrintTime = currentTime;

    const char *mode = (trackingDuration < STARTUP_DURATION) ? "加速" : "巡航";

    // 显示原始和滤波后的传感器数据对比
    Serial.printf("模式:%s 时间:%ds 速度:%d ", mode, trackingDuration / 1000, currentSpeed);
    Serial.printf("传感器(原始/滤波):%d%d%d%d/%d%d%d%d ",
                  digitalRead(IR_OutLeftPin), digitalRead(IR_InLeftPin),
                  digitalRead(IR_InRightPin), digitalRead(IR_OutRightPin),
                  ir1, ir2, ir3, ir4);
    Serial.printf("误差:%.1f 舵机:%.1f°\n", error, currentServoAngle);

    // 蓝牙发送关键数据（避免太频繁）
    if (fabs(error) >= 1.5f || (trackingDuration < STARTUP_DURATION && trackingDuration % 1000 < 50))
    {
      String statusMsg = "TRACK:" + String(mode) +
                         " Time:" + String(trackingDuration / 1000) + "s" +
                         " Spd:" + String(currentSpeed) +
                         " Err:" + String(error, 1) +
                         " Ang:" + String(currentServoAngle, 1);
      SerialBT.println(statusMsg);
    }
  }
}