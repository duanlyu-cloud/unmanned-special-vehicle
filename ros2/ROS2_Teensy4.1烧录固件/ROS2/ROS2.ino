#include <AccelStepper.h>
#include <Encoder.h>
#include <avr/pgmspace.h> // 用于访问程序空间的库
#include <math.h> // 提供数学函数的库

// 固件版本
const char* VERSION = "0.0.1";

///////////////////////////////////////////////////////////////////////////////
// 物理参数
///////////////////////////////////////////////////////////////////////////////

// 步进电机的步进引脚
const int STEP_PINS[] = {0, 2, 4, 6, 8, 10};
// 步进电机的方向引脚
const int DIR_PINS[] = {1, 3, 5, 7, 9, 11};
// 限位开关引脚
const int LIMIT_PINS[] = {26, 27, 28, 29, 30, 31};

// 每度的步数（针对每个电机）
const float MOTOR_STEPS_PER_DEG[] = {44.44444444, 55.55555556, 55.55555556,
                                     43.55555556, 21.86024888, 21.11111111};
// 每旋转的步数（针对每个电机）
const int MOTOR_STEPS_PER_REV[] = {400, 400, 400, 400, 800, 400};

// 设置编码器引脚
Encoder encPos[6] = {Encoder(14, 15), Encoder(17, 16), Encoder(19, 18),
                     Encoder(20, 21), Encoder(23, 22), Encoder(24, 25)};


// 如果编码器方向与电机方向匹配，+1；否则，-1
int ENC_DIR[] = {-1, 1, 1, 1, 1, 1};
// 如果编码器的最大值在最小关节角度处为+1，否则为0
int ENC_MAX_AT_ANGLE_MIN[] = {1, 0, 1, 0, 0, 1};
// 电机步数 * ENC_MULT = 编码器步数
const float ENC_MULT[] = {10, 10, 10, 10, 10, 10};

// 关节的极限位置（度数）
int JOINT_LIMIT_MIN[] = {-170, -42, -89, -165, -105, -155};
int JOINT_LIMIT_MAX[] = {170, 90, 52, 165, 105, 155};

///////////////////////////////////////////////////////////////////////////////
// ROS驱动参数
///////////////////////////////////////////////////////////////////////////////

// 大约等于0，-6，0，0，0，0度
const int REST_ENC_POSITIONS[] = {75507, 20000, 49234, 71867, 23346, 32595};
// 状态机状态：轨迹状态或错误状态
enum SM { STATE_TRAJ, STATE_ERR };
SM STATE = STATE_TRAJ;

const int NUM_JOINTS = 6;
AccelStepper stepperJoints[NUM_JOINTS];

// 校准设置
const int LIMIT_SWITCH_HIGH[] = {
    1, 1, 1, 1, 1, 1};  // 考虑到NC和NO限位开关
const int CAL_DIR[] = {-1, -1, 1,
                       -1, -1, 1};  // 关节到限位开关的旋转方向
const int CAL_SPEED = 500;          // 电机每秒步数
const int CAL_SPEED_MULT[] = {
    1, 1, 1, 2, 1, 1};  // 用于调整电机每旋转步数的乘数
// 设置关节的最大速度和加速度
// JOINT_MAX_SPEED: 关节的最大速度数组，单位为度/秒
// JOINT_MAX_ACCEL: 关节的最大加速度数组，单位为度/秒^2
float JOINT_MAX_SPEED[] = {30.0, 30.0, 30.0, 30.0, 30.0, 30.0};  // deg/s
float JOINT_MAX_ACCEL[] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};  // deg/s^2

// 关节名称数组
char JOINT_NAMES[] = {'A', 'B', 'C', 'D', 'E', 'F'};

// 编码器步数：定义了关节运动范围内的编码器步数
// ENC_RANGE_STEPS: 每个关节的编码器步数数组，用于定义关节数字编码器的运动范围
int ENC_RANGE_STEPS[NUM_JOINTS];

void setup() {
  Serial.begin(115200); // 初始化串行通信

  for (int i = 0; i < NUM_JOINTS; ++i) {
    pinMode(STEP_PINS[i], OUTPUT); // 设置步进引脚为输出
    pinMode(DIR_PINS[i], OUTPUT); // 设置方向引脚为输出
    pinMode(LIMIT_PINS[i], INPUT_PULLUP); // 设置限位开关引脚为输入

    int joint_range = JOINT_LIMIT_MAX[i] - JOINT_LIMIT_MIN[i];
    ENC_RANGE_STEPS[i] =
        static_cast<int>(MOTOR_STEPS_PER_DEG[i] * joint_range * ENC_MULT[i]);
  }
}

bool initStateTraj(String inData) {
  // 解析初始化消息
  int idxVersion = inData.indexOf('A');
  String softwareVersion =
      inData.substring(idxVersion + 1, inData.length() - 1);
  int versionMatches = (softwareVersion == VERSION);

  // 返回确认消息包含结果
  String msg = String("ST") + "A" + versionMatches + "B" + VERSION;
  Serial.println(msg);

  return versionMatches ? true : false;
}

void readMotorSteps(int* motorSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    // motorSteps[i] = encPos[i].read() / ENC_MULT[i];
    motorSteps[i] = stepperJoints[i].currentPosition()*10.24 * ENC_DIR[i] / ENC_MULT[i];
  }
}

void encStepsToJointPos(int* encSteps, double* jointPos) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    jointPos[i] = encSteps[i] / MOTOR_STEPS_PER_DEG[i] * ENC_DIR[i];
  }
}

void jointPosToEncSteps(double* jointPos, int* encSteps) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    encSteps[i] = jointPos[i] * MOTOR_STEPS_PER_DEG[i] * ENC_DIR[i];
  }
}

String JointPosToString(double* jointPos) {
  String out;
  for (int i = 0; i < NUM_JOINTS; ++i) {
    out += JOINT_NAMES[i];  // 追加关节名称
    out += String(jointPos[i], 6);
  }
  return out;
}

void updateStepperSpeed(String inData) {
  // 更新关节的最大速度和加速度
  int idxSpeedJ1 = inData.indexOf('A');
  int idxAccelJ1 = inData.indexOf('B');
  int idxSpeedJ2 = inData.indexOf('C');
  int idxAccelJ2 = inData.indexOf('D');
  int idxSpeedJ3 = inData.indexOf('E');
  int idxAccelJ3 = inData.indexOf('F');
  int idxSpeedJ4 = inData.indexOf('G');
  int idxAccelJ4 = inData.indexOf('H');
  int idxSpeedJ5 = inData.indexOf('I');
  int idxAccelJ5 = inData.indexOf('J');
  int idxSpeedJ6 = inData.indexOf('K');
  int idxAccelJ6 = inData.indexOf('L');

  JOINT_MAX_SPEED[0] = inData.substring(idxSpeedJ1 + 1, idxAccelJ1).toFloat();
  JOINT_MAX_ACCEL[0] = inData.substring(idxAccelJ1 + 1, idxSpeedJ2).toFloat();
  JOINT_MAX_SPEED[1] = inData.substring(idxSpeedJ2 + 1, idxAccelJ2).toFloat();
  JOINT_MAX_ACCEL[1] = inData.substring(idxAccelJ2 + 1, idxSpeedJ3).toFloat();
  JOINT_MAX_SPEED[2] = inData.substring(idxSpeedJ3 + 1, idxAccelJ3).toFloat();
  JOINT_MAX_ACCEL[2] = inData.substring(idxAccelJ3 + 1, idxSpeedJ4).toFloat();
  JOINT_MAX_SPEED[3] = inData.substring(idxSpeedJ4 + 1, idxAccelJ4).toFloat();
  JOINT_MAX_ACCEL[3] = inData.substring(idxAccelJ4 + 1, idxSpeedJ5).toFloat();
  JOINT_MAX_SPEED[4] = inData.substring(idxSpeedJ5 + 1, idxAccelJ5).toFloat();
  JOINT_MAX_ACCEL[4] = inData.substring(idxAccelJ5 + 1, idxSpeedJ6).toFloat();
  JOINT_MAX_SPEED[5] = inData.substring(idxSpeedJ6 + 1, idxAccelJ6).toFloat();
  JOINT_MAX_ACCEL[5] = inData.substring(idxAccelJ6 + 1).toFloat();
}

void calibrateJoints(int* calJoints) {
  // 检查要校准的关节
  bool calAllDone = false;
  bool calJointsDone[NUM_JOINTS];
  for (int i = 0; i < NUM_JOINTS; ++i) {
    calJointsDone[i] = !calJoints[i];
  }

  // 第一次校准通过，快速速度
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i]);
  }
  while (!calAllDone) {
    calAllDone = true;
    for (int i = 0; i < NUM_JOINTS; ++i) {
      // 如果关节尚未校准
      if (!calJointsDone[i]) {
        // 检查限位开关
        if (!reachedLimitSwitch(i)) {
          // 未达到限位开关，继续移动
          stepperJoints[i].runSpeed();
          calAllDone = false;
        } else {
          // 达到限位开关
          stepperJoints[i].setSpeed(0);  // 冗余
          calJointsDone[i] = true;
        }
      }
    }
  }
  delay(2000);

  return;
}

void moveAwayFromLimitSwitch() {
  // 使关节远离限位开关
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(CAL_SPEED * CAL_SPEED_MULT[i] * CAL_DIR[i] * -1);
  }
  for (int j = 0; j < 10000000; j++) {
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].runSpeed();
    }
  }
  // 冗余
  for (int i = 0; i < NUM_JOINTS; i++) {
    stepperJoints[i].setSpeed(0);
  }
  delay(2000);
  return;
}

bool reachedLimitSwitch(int joint) {
  // 检查是否达到限位开关
  int pin = LIMIT_PINS[joint];
  // 多次检查以处理噪声
  for (int i = 0; i < 5; ++i) {
    if (digitalRead(pin) != LIMIT_SWITCH_HIGH[joint]) {
      return false;
    }
  }
  return true;
}

/**
 * 状态机函数：用于处理关节运动的轨迹规划
 * 无参数
 * 无返回值
 */
void stateTRAJ() {
  // 清空接收的消息
  String inData = "";

  // 初始化当前关节位置和电机步数
  double curJointPos[NUM_JOINTS];
  int curMotorSteps[NUM_JOINTS];
  readMotorSteps(curMotorSteps);

  double cmdJointPos[NUM_JOINTS];
  int cmdEncSteps[NUM_JOINTS];
  // 使用当前电机步数作为初始命令编码器步数
  for (int i = 0; i < NUM_JOINTS; ++i) {
    cmdEncSteps[i] = curMotorSteps[i];
  }

  // 初始化AccelStepper实例
  for (int i = 0; i < NUM_JOINTS; ++i) {
    stepperJoints[i] = AccelStepper(1, STEP_PINS[i], DIR_PINS[i]);
    stepperJoints[i].setPinsInverted(true, false, false);  // DM542T正转
    stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                     MOTOR_STEPS_PER_DEG[i]);
    stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] * MOTOR_STEPS_PER_DEG[i]);
    stepperJoints[i].setMinPulseWidth(10);
  }
  // J4使用DM320T反向
  stepperJoints[3].setPinsInverted(false, false, false);  

  // 主循环：等待和处理来自主机的消息
  while (STATE == STATE_TRAJ) {
    char received = '\0';
    // 检查是否有来自主机的消息
    if (Serial.available()) {
      received = Serial.read();
      inData += received;
    }

    // 当接收到换行符时处理消息
    if (received == '\n') {
      String function = inData.substring(0, 2);
      // 根据接收到的指令处理不同功能
      if (function == "MT") {
        // 更新当前电机步数
        readMotorSteps(curMotorSteps);

        // 将当前电机步数转换为关节位置并回复给主机
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);

        // 接收并处理新的位置指令
        int msgIdxJ1 = inData.indexOf('A');
        int msgIdxJ2 = inData.indexOf('B');
        int msgIdxJ3 = inData.indexOf('C');
        int msgIdxJ4 = inData.indexOf('D');
        int msgIdxJ5 = inData.indexOf('E');
        int msgIdxJ6 = inData.indexOf('F');
        cmdJointPos[0] = inData.substring(msgIdxJ1 + 1, msgIdxJ2).toFloat();
        cmdJointPos[1] = inData.substring(msgIdxJ2 + 1, msgIdxJ3).toFloat();
        cmdJointPos[2] = inData.substring(msgIdxJ3 + 1, msgIdxJ4).toFloat();
        cmdJointPos[3] = inData.substring(msgIdxJ4 + 1, msgIdxJ5).toFloat();
        cmdJointPos[4] = inData.substring(msgIdxJ5 + 1, msgIdxJ6).toFloat();
        cmdJointPos[5] = inData.substring(msgIdxJ6 + 1).toFloat();
        jointPosToEncSteps(cmdJointPos, cmdEncSteps);

        // 更新目标关节位置
        readMotorSteps(curMotorSteps);
        for (int i = 0; i < NUM_JOINTS; ++i) {
          int diffEncSteps = cmdEncSteps[i] - curMotorSteps[i];
          if (abs(diffEncSteps) > 2) {
            int diffMotSteps = diffEncSteps * ENC_DIR[i];
            // 如果步数差异过大，进行运动控制
            stepperJoints[i].move(diffMotSteps);
            stepperJoints[i].run();
          }
        }
      } else if (function == "JC") {
        // 执行关节校准
        int calJoints[] = {1, 1, 1, 1, 1, 1};
        calibrateJoints(calJoints);

        // 记录编码器校准位置
        int calSteps[6];
        for (int i = 0; i < NUM_JOINTS; ++i) {
          // calSteps[i] = encPos[i].read();
          calSteps[i] = stepperJoints[i].currentPosition()*10.24 * ENC_DIR[i];
        }

        // 设置编码器初始位置
        for (int i = 0; i < NUM_JOINTS; ++i) {
          // encPos[i].write(ENC_RANGE_STEPS[i] * ENC_MAX_AT_ANGLE_MIN[i]);
          stepperJoints[i].setCurrentPosition(ENC_RANGE_STEPS[i] * ENC_MAX_AT_ANGLE_MIN[i]/10.24 * ENC_DIR[i]);
        }

        // 远离限位开关
        moveAwayFromLimitSwitch();

        // 返回初始位置
        for (int i = 0; i < NUM_JOINTS; ++i) {
          stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                           MOTOR_STEPS_PER_DEG[i]);
          stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                       MOTOR_STEPS_PER_DEG[i]);
        }

        // 等待关节回到休息位置
        bool restPosReached = false;
        while (!restPosReached) {
          restPosReached = true;
          readMotorSteps(curMotorSteps);

          for (int i = 0; i < NUM_JOINTS; ++i) {
            if (abs(REST_ENC_POSITIONS[i] / ENC_MULT[i] - curMotorSteps[i]) >
                10) {
              restPosReached = false;
              float target_pos =
                  (REST_ENC_POSITIONS[i] / ENC_MULT[i] - curMotorSteps[i]) *
                  ENC_DIR[i];
              stepperJoints[i].move(target_pos);
              stepperJoints[i].run();
            }
          }
        }

        // 发送校准结果给主机
        String msg = String("JC") + "A" + calSteps[0] + "B" + calSteps[1] +
                     "C" + calSteps[2] + "D" + calSteps[3] + "E" + calSteps[4] +
                     "F" + calSteps[5];
        Serial.println(msg);
      } else if (function == "JP") {
        // 更新并发送当前关节位置给主机
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);
      } else if (function == "SS") {
        // 更新步进器速度
        updateStepperSpeed(inData);

        // 设置电机速度和加速度
        for (int i = 0; i < NUM_JOINTS; ++i) {
          stepperJoints[i].setAcceleration(JOINT_MAX_ACCEL[i] *
                                           MOTOR_STEPS_PER_DEG[i]);
          stepperJoints[i].setMaxSpeed(JOINT_MAX_SPEED[i] *
                                       MOTOR_STEPS_PER_DEG[i]);
        }

        // 更新主机上的关节位置
        readMotorSteps(curMotorSteps);
        encStepsToJointPos(curMotorSteps, curJointPos);
        String msg = String("JP") + JointPosToString(curJointPos);
        Serial.println(msg);
      } else if (function == "ST") {
        // 初始化轨迹规划状态
        if (!initStateTraj(inData)) {
          STATE = STATE_ERR;
          return;
        }
      }
      // 清空接收到的消息
      inData = "";
    }
    // 更新所有关节的位置
    for (int i = 0; i < NUM_JOINTS; ++i) {
      stepperJoints[i].run();
    }
  }
}

/**
 * 进入错误状态并保持，直到系统重置。
 * 该状态下，所有关节步进电机被禁止动作，并且会周期性地打印错误信息。
 * 该函数不接受参数，也不返回任何值。
 */
void stateERR() {
  // 进入保持状态，将所有步进电机的步进信号置为低电平
  for (int i = 0; i < NUM_JOINTS; ++i) {
    digitalWrite(STEP_PINS[i], LOW);
  }

  // 在错误状态下循环，不断打印错误信息并等待重置
  while (STATE == STATE_ERR) {
    Serial.println("DB: Unrecoverable error state entered. Please reset.");
    delay(1000); // 每秒打印一次错误信息
  }
}

/**
 * 主循环函数
 * 该函数不断执行，根据当前状态选择相应的处理逻辑。
 * 
 * 参数: 无
 * 返回值: 无
 */
void loop() {
  // 设置当前状态为轨迹执行状态
  STATE = STATE_TRAJ;

  // 根据当前状态选择处理函数
  switch (STATE) {
    case STATE_ERR: // 如果状态为错误状态，执行错误处理函数
      stateERR();
      break;
    default: // 默认情况下，执行轨迹执行函数
      stateTRAJ();
      break;
  }
}
