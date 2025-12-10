#include <Adafruit_BNO055.h>
#include <PicomniRover.h>

#define ROBOT_RADIUS 0.083f
#define WHEEL_RADIUS 0.024f

#define FEETECH_TO_RPS(x) ((x) / 4096.0f)
#define RPS_TO_FEETECH(x) ((x) * 4096.0f)

FeetechBus feetechBus;
FeetechServo motor1(1, &feetechBus);
FeetechServo motor2(2, &feetechBus);
FeetechServo motor3(3, &feetechBus);
Adafruit_BNO055 bno;

unsigned long prev_us = 0;

int16_t motor1_vel = 0;
int16_t motor2_vel = 0;
int16_t motor3_vel = 0;

float x = 0.0f;
float y = 0.0f;

void setup() {
  WirelessControl::begin();

  Serial.begin(115200);

  feetechBus.begin();
  motor1.controlMode(1);
  motor2.controlMode(1);
  motor3.controlMode(1);

  Wire.setSDA(16);
  Wire.setSCL(17);
  bno.begin(OPERATION_MODE_IMUPLUS);
}

void loop() {
  unsigned long now_us = micros();
  unsigned long delta_us = now_us - prev_us;
  float delta_s = delta_us / 1000000.0f;
  prev_us = now_us;

  // モーター速度受信
  motor1.getVelocity(&motor1_vel);
  motor2.getVelocity(&motor2_vel);
  motor3.getVelocity(&motor3_vel);

  // IMU角度受信
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = -radians(euler.x()) + 2 * PI;

  // BLE受信
  Command cmd = WirelessControl::getCommand();
  float vx = cmd.vx;
  float vy = cmd.vy;
  float w = cmd.w;

  float v1 = (vy * cos(0.0f) + vx * sin(0.0f) + w * ROBOT_RADIUS) / (2.0f * M_PI * WHEEL_RADIUS);
  float v2 =
      (vy * cos(PI * 2.0f / 3.0f) + vx * sin(PI * 2.0f / 3.0f) + w * ROBOT_RADIUS) / (2.0f * M_PI * WHEEL_RADIUS);
  float v3 =
      (vy * cos(PI * 4.0f / 3.0f) + vx * sin(PI * 4.0f / 3.0f) + w * ROBOT_RADIUS) / (2.0f * M_PI * WHEEL_RADIUS);

  // モーター速度送信
  motor1.setVelocity(RPS_TO_FEETECH(v1));
  motor2.setVelocity(RPS_TO_FEETECH(v2));
  motor3.setVelocity(RPS_TO_FEETECH(v3));

  float d1 = 2.0f * PI * WHEEL_RADIUS * FEETECH_TO_RPS(motor1_vel) * delta_s;
  float d2 = 2.0f * PI * WHEEL_RADIUS * FEETECH_TO_RPS(motor2_vel) * delta_s;
  float d3 = 2.0f * PI * WHEEL_RADIUS * FEETECH_TO_RPS(motor3_vel) * delta_s;

  float local_dx = d1 * sin(0.0f) + d2 * sin(PI * 2.0f / 3.0f) + d3 * sin(PI * 4.0f / 3.0f);
  float local_dy = d1 * cos(0.0f) + d2 * cos(PI * 2.0f / 3.0f) + d3 * cos(PI * 4.0f / 3.0f);

  float global_dx = local_dy * sin(yaw) + local_dx * cos(yaw);
  float global_dy = local_dy * cos(yaw) - local_dx * sin(yaw);

  x += global_dx;
  y += global_dy;

  // BLE送信
  Odometry odom;
  odom.x = x;
  odom.y = y;
  odom.yaw = yaw;
  WirelessControl::setOdometry(odom);

  Serial.println(delta_us);
}
