#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "commander.h"
#include "relative_localization.h"
#include "ranging_struct.h"
#include "num.h"
#include "param.h"
#include "debug.h"
#include <stdlib.h> // random
#include "configblock.h"
#include "uart2.h"
#include "log.h"
#include "math.h"
#include "adhocdeck.h"
#define RUNNING_STAGE 1 // 0代码debug阶段，1代码运行阶段

static uint16_t MY_UWB_ADDRESS;
static bool isInit;
static bool onGround = true;               // 无人机当前是否在地面上?
static bool isCompleteTaskAndLand = false; // 无人机是否已经执行了飞行任务并落地?
bool keepFlying = false;
static setpoint_t setpoint;
static float_t relaVarInCtrl[RANGING_TABLE_SIZE + 1][STATE_DIM_rl];
static float_t neighbor_height[RANGING_TABLE_SIZE + 1];
static currentNeighborAddressInfo_t currentNeighborAddressInfo;
static float_t set_height = 0.5;
static float_t set_height0 = 0.5;
static paramVarId_t idMultiranger;
static logVarId_t idUp;
static logVarId_t idLeft;
static logVarId_t idRight;
static logVarId_t idFront;
static logVarId_t idBack;
static const float velMax = 3.0f;
static const uint16_t radius = 300;
#define MAX(a, b) ((a > b) ? a : b)
#define MIN(a, b) ((a < b) ? a : b)

float relaCtrl_p = 2.0f;
// static float relaCtrl_i = 0.0001f;
float relaCtrl_i = 0.01f;
float relaCtrl_d = 0.01f;
static const float up_down_delta = 0.002f;
// static float NDI_k = 2.0f;

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  float velFront = 0;
  float velSide = 0;
  float factor = velMax / radius;
  // uint8_t multirangerInit = paramGetUint(idMultiranger);
  // if (multirangerInit)
  // {
  //   uint16_t up = logGetUint(idUp);
  //   uint16_t left = logGetUint(idLeft);
  //   uint16_t right = logGetUint(idRight);
  //   uint16_t front = logGetUint(idFront);
  //   uint16_t back = logGetUint(idBack);
  //   uint16_t left_o = radius - MIN(left, radius);
  //   uint16_t right_o = radius - MIN(right, radius);
  //   float l_comp = (-1) * left_o * factor;
  //   float r_comp = right_o * factor;
  //   velSide = r_comp + l_comp;

  //   uint16_t front_o = radius - MIN(front, radius);
  //   uint16_t back_o = radius - MIN(back, radius);
  //   float f_comp = (-1) * front_o * factor;
  //   float b_comp = back_o * factor;
  //   velFront = b_comp + f_comp;

  //   if (up < radius)
  //   {
  //     set_height -= 0.002f;
  //   }
  // }
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx + velFront;
  setpoint->velocity.y = vy + velSide;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
  if (set_height <= 0.1)
  {
    // isCompleteTaskAndLand = true;
    land(set_height);
  }
}

static void setHoverSetpoint_takeoff(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;
  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity_body = true;
  commanderSetSetpoint(setpoint, 3);
}

static void flyRandomIn1meter(float_t randomVel, float height)
{
  float_t randomYaw = (rand() / (float)RAND_MAX) * 6.28f; // 0-2pi rad
  // float_t randomVel = (rand() / (float)RAND_MAX) * 1;     // 0-1 m/s
  float_t vxBody = randomVel * cosf(randomYaw); // 速度分解
  float_t vyBody = randomVel * sinf(randomYaw);
  for (int i = 1; i < 100; i++)
  {
    setHoverSetpoint(&setpoint, vxBody, vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
  setHoverSetpoint(&setpoint, 0, 0, height, 0);
  vTaskDelay(M2T(10));
  for (int i = 1; i < 100; i++)
  {
    setHoverSetpoint(&setpoint, -vxBody, -vyBody, height, 0);
    vTaskDelay(M2T(10));
  }
}

#define SIGN(a) ((a >= 0) ? 1 : -1)
static float_t targetX;
static float_t targetY;
static float PreErr_x = 0;
static float PreErr_y = 0;
static float IntErr_x = 0;
static float IntErr_y = 0;
static uint32_t PreTime;
static void formation0asCenter(float_t tarX, float_t tarY, float_t height)
{
  float dt = (float)(xTaskGetTickCount() - PreTime) / configTICK_RATE_HZ;
  PreTime = xTaskGetTickCount();
  if (dt > 1) // skip the first run of the EKF
    return;
  // pid control for formation flight 当前是1号无人机
  float err_x = -(tarX - relaVarInCtrl[0][STATE_rlX]);
  float err_y = -(tarY - relaVarInCtrl[0][STATE_rlY]);
  float pid_vx = relaCtrl_p * err_x;  // 2.0*err_x 基于距离差进行一个速度控制
  float pid_vy = relaCtrl_p * err_y;  // 2.0*err_y
  float dx = (err_x - PreErr_x) / dt; // 先前的速度
  float dy = (err_y - PreErr_y) / dt;
  PreErr_x = err_x;
  PreErr_y = err_y;
  pid_vx += relaCtrl_d * dx; // 0.01*dx 先前速度*比例系数
  pid_vy += relaCtrl_d * dy; // 0.01*dy
  IntErr_x += err_x * dt;
  IntErr_y += err_y * dt;
  pid_vx += relaCtrl_i * constrain(IntErr_x, -0.5, 0.5); // += (+-)0.00005
  pid_vy += relaCtrl_i * constrain(IntErr_y, -0.5, 0.5);
  pid_vx = constrain(pid_vx, -0.5f, 0.5f);
  pid_vy = constrain(pid_vy, -0.5f, 0.5f);

  // float rep_x = 0.0f;
  // float rep_y = 0.0f;
  // for(uint8_t i=0; i<NumUWB; i++){
  //   if(i!=selfID){
  //     float dist = relaVarInCtrl[i][STATE_rlX]*relaVarInCtrl[i][STATE_rlX] + relaVarInCtrl[i][STATE_rlY]*relaVarInCtrl[i][STATE_rlY];
  //     dist = sqrtf(dist);
  //     rep_x += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlX]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlX]);
  //     rep_y += -0.5f * (SIGN(0.5f - dist) + 1) / (abs(relaVarInCtrl[i][STATE_rlY]) + 0.001f) * SIGN(relaVarInCtrl[i][STATE_rlY]);
  //   }
  // }
  // rep_x = constrain(rep_x, -1.5f, 1.5f);
  // rep_y = constrain(rep_y, -1.5f, 1.5f);

  // pid_vx = constrain(pid_vx + rep_x, -1.5f, 1.5f);
  // pid_vy = constrain(pid_vy + rep_y, -1.5f, 1.5f);

  setHoverSetpoint(&setpoint, pid_vx, pid_vy, height, 0);
}

void take_off(float_t height)
{
  for (int i = 0; i < 20; i++)
  {
    setHoverSetpoint_takeoff(&setpoint, 0, 0, (height * i) / 20, 0);
    vTaskDelay(M2T(25));
  }
  for (int i = 0; i < 20; i++)
  {
    setHoverSetpoint_takeoff(&setpoint, 0, 0, height, 0);
    vTaskDelay(M2T(10));
  }
  // for (int i = 0; i < 10 * MY_UWB_ADDRESS; i++)
  // {
  //   setHoverSetpoint(&setpoint, 0, 0, height, 0);
  //   vTaskDelay(M2T(100));
  // }
  onGround = false;
}

void land(float_t height)
{
  // landing procedure
  if (!onGround)
  {
    int i = 0;
    float land_height_per_100ms = 0.02;                            // 每秒下降的高度为该变量的值*10
    while (height > 0.1 && height - i * land_height_per_100ms > 0) // 1s下降0.2s
    {
      i++;
      setHoverSetpoint(&setpoint, 0, 0, height - (float)i * land_height_per_100ms, 0);
      vTaskDelay(M2T(100));
    }
    isCompleteTaskAndLand = true;
  }
  onGround = true;
}

float get_min(float *var_history, int len_history)
{
  float res = var_history[0];
  for (size_t i = 1; i < len_history; i++)
  {
    res = var_history[i] < res ? var_history[i] : res;
  }
  return res;
}

float get_max(float *var_history, int len_history)
{
  float res = var_history[0];
  for (size_t i = 1; i < len_history; i++)
  {
    res = var_history[i] > res ? var_history[i] : res;
  }
  return res;
}

void reset_estimators()
{

  int len_history = 10;
  float var_x_history[len_history];
  float var_y_history[len_history];
  float var_z_history[len_history];
  for (size_t i = 0; i < len_history; i++)
  {
    var_x_history[i] = 1000.0;
    var_y_history[i] = 1000.0;
    var_z_history[i] = 1000.0;
  }
  float threshold = 0.001;
  int i = 0;
  while (true)
  {
    /* PX,PY,PZ log variable id */
    float idVelocityX = logGetVarId("kalman", "varPX");
    float idVelocityY = logGetVarId("kalman", "varPY");
    float idVelocityZ = logGetVarId("kalman", "varPZ");
    float velocityX = logGetFloat(idVelocityX);
    float velocityY = logGetFloat(idVelocityY);
    float velocityZ = logGetFloat(idVelocityZ);
    var_x_history[i] = velocityX;
    var_y_history[i] = velocityY;
    var_z_history[i] = velocityZ;

    float min_x = get_min(var_x_history, len_history);
    float max_x = get_max(var_x_history, len_history);
    float min_y = get_min(var_y_history, len_history);
    float max_y = get_max(var_y_history, len_history);
    float min_z = get_min(var_z_history, len_history);
    float max_z = get_max(var_z_history, len_history);
    if (((max_x - min_x) < threshold) &&
        ((max_y - min_y) < threshold) &&
        ((max_z - min_z) < threshold))
    {
      break;
    }
    i = (i + 1) % len_history;
  }
}

void control_fly_square(int fly_case){
    switch (fly_case)
    {
    case 0:
        for(int i=0;i< 200;i++){
            setHoverSetpoint(&setpoint, 1, 0, set_height ,0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_one_square\n");
        break;
    
    case 1:
        for(int i=0;i< 200;i++){
            setHoverSetpoint(&setpoint, 0, 1, set_height ,0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_two_square\n");
        break;
    
    case 2:
        for(int i=0;i< 200;i++){
            setHoverSetpoint(&setpoint, -1, 0, set_height ,0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_three_square\n");
        break;
    
    case 3:
        for(int i=0;i< 200;i++){
            setHoverSetpoint(&setpoint, 0, -1, set_height ,0);
            vTaskDelay(M2T(10));
        }
        DEBUG_PRINT("stage_four_square\n");
        break;
    }
}

void relativeControlTask(void *arg)
{
  int8_t targetShift = 0;

  uint8_t currentPosition_3Stage = MY_UWB_ADDRESS; // 当前位于的位置
  uint8_t currentPosition_4Stage = MY_UWB_ADDRESS; // 当前位于的位置
  const float_t initDist = 1;
  const float_t doubInitDist = 2;
  static const float_t targetList[25][STATE_DIM_rl] = {
    {0.0f, 0.0f, 0.0f},                   // 0
    {0.0f, -initDist, 0.0f},              // 1
    {-initDist, -initDist, 0.0f},         // 2
    {-initDist, 0.0f, 0.0f},              // 3
    {-initDist, initDist, 0.0f},          // 4
    {0.0f, initDist, 0.0f},               // 5
    {initDist, initDist, 0.0f},           // 6
    {initDist, 0.0f, 0.0f},               // 7
    {initDist, -initDist, 0.0f},          // 8
    {doubInitDist, initDist, 0.0f},       // 9
    {doubInitDist, 0.0f, 0.0f},           // 10
    {doubInitDist, -initDist, 0.0f},      // 11
    {doubInitDist, -doubInitDist, 0.0f},  // 12 
    {initDist, -doubInitDist, 0.0f},      // 13
    {0.0f, -doubInitDist, 0.0f},          // 14
    {-initDist, -doubInitDist, 0.0f},     // 15
    {-doubInitDist, -doubInitDist, 0.0f}, // 16
    {-doubInitDist, -initDist, 0.0f},     // 17
    {-doubInitDist, 0.0f, 0.0f},          // 18
    {-doubInitDist, initDist, 0.0f},      // 19
    {-doubInitDist, doubInitDist, 0.0f},  // 20
    {-initDist, doubInitDist, 0.0f},     // 21
    {0.0f, doubInitDist, 0.0f},          // 22
    {initDist, doubInitDist, 0.0f},      // 23
    {doubInitDist, doubInitDist, 0.0f}};  // 24
  uint8_t SQURE3_3_NUM = 9;             // 3阶段转圈的无人机数量+1（0号无人机）
  uint8_t SQURE3_4_NUM = 11;
  static const uint8_t targetSquere3_3[15] = {
      0, 1, 2, 3, 4, 5, 6, 7, 8 // 8个位置,为了使得索引和值一一对应，所以有0
  };
  static const uint8_t indexToPosi3_4[15] = {
      0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9 // 11个位置
  };
  static const int8_t posiToIndex3_4[15] = {// -1代码无效
                                            -1, 1, 2, 3, 4, 5, 6, 7, -1, 10, 9, 8};
  /*
  posi  0, 1, 2, 3, 4, 5, 6, 7, 11, 10, 9
  index 0  1  2  3  4  5  6  7  8   9   10
  */
  /*下面是五边形，0号在中间*/
  // static const float_t target_five[15][STATE_DIM_rl] = {
  //     {0.0f, 0.0f, 0.0f},   // 0
  //     {0.43, -1.13, 0.0f},  // 1
  //     {-1.13, -0.82, 0.0f}, // 2
  //     {-1.13, 0.82, 0.0f},  // 3
  //     {0.43, 1.33, 0.0f},   // 4
  //     {1.4, 0.0, 0.0f},     // 5
  // };
  static const float_t target_five[15][STATE_DIM_rl] = {
      {0.0f, 0.0f, 0.0f},  // 0
      {0.22, -0.6, 0.0f},  // 1
      {-0.6, -0.41, 0.0f}, // 2
      {-0.6, 0.41, 0.0f},  // 3
      {0.22, 0.7, 0.0f},   // 4
      {0.7, 0.0, 0.0f},    // 5
  };
  /*下面是三角形编队*/
  // static const float_t target_trangle[15][STATE_DIM_rl] = {
  //     {0.0f, 0.0f, 0.0f}, // 0
  //     {-1.0, -0.6, 0.0f}, // 1
  //     {-2.0, -1.2, 0.0f}, // 2
  //     {-2.0, -0.0, 0.0f}, // 3
  //     {-2.0, 1.2, 0.0f},  // 4
  //     {-1.0, 0.6, 0.0f},  // 5
  // };
  static const float_t target_trangle[15][STATE_DIM_rl] = {
      {0.0f, 0.0f, 0.0f}, // 0
      {-0.8, -0.5, 0.0f}, // 1
      {-2.0, -1.2, 0.0f}, // 2
      {-2.0, -0.0, 0.0f}, // 3
      {-2.0, 1.2, 0.0f},  // 4
      {-0.8, 0.5, 0.0f},  // 5
  };

  systemWaitStart();
  reset_estimators(); // 判断无人机数值是否收敛
  // idUp = logGetVarId("range", "up");
  // idLeft = logGetVarId("range", "left");
  // idRight = logGetVarId("range", "right");
  // idFront = logGetVarId("range", "front");
  // idBack = logGetVarId("range", "back");
  // idMultiranger = paramGetVarId("deck", "bcMultiranger");
  // uint8_t multirangerInit = paramGetUint(idMultiranger);
  uint8_t multirangerInit = false;
  DEBUG_PRINT("multirangerInit is %d\n", multirangerInit);
  static uint8_t stage = 0;
  uint32_t tickInterval = 0; 
  uint32_t takeofftickInterval = 0; 
  while (1)
  { vTaskDelay(10);
    keepFlying = getOrSetKeepflying(MY_UWB_ADDRESS, keepFlying);
    if(keepFlying){
      if(onGround){
      take_off(set_height);
      takeofftickInterval = xTaskGetTickCount();
    }
    if (MY_UWB_ADDRESS == 0){
      setHoverSetpoint(&setpoint, 0, 0, set_height, 0);
    }else
    {
    DEBUG_PRINT("stage is %d\n", stage);
    control_fly_square(stage);
    stage = (stage+1)%4;
    DEBUG_PRINT("stage is %d\n", stage);
    }
    tickInterval  = xTaskGetTickCount() - takeofftickInterval; // 获取当前时间
    if(tickInterval > 60000){
      keepFlying = false;
      land(set_height);
      
    }
    }else{
      land(set_height);
    }
    
  }
}

void relativeControlInit(void)
{
  if (isInit)
    return;
  MY_UWB_ADDRESS = getUWBAddress();
  srand(MY_UWB_ADDRESS);
  xTaskCreate(relativeControlTask, "relative_Control", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
  isInit = true;
}

PARAM_GROUP_START(relative_ctrl)
PARAM_ADD(PARAM_UINT8, keepFlying, &keepFlying)
PARAM_ADD(PARAM_UINT8, mode, &CONTROL_MODE)
PARAM_ADD(PARAM_FLOAT, relaCtrl_p, &relaCtrl_p)
PARAM_ADD(PARAM_FLOAT, relaCtrl_i, &relaCtrl_i)
PARAM_ADD(PARAM_FLOAT, relaCtrl_d, &relaCtrl_d)
PARAM_GROUP_STOP(relative_ctrl)