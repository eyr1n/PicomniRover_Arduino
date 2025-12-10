#include <PicomniRover.h>

// 新しいFeetechモーターID
#define MOTOR_ID_NEW 2

FeetechBus feetechBus;

void setup() {
  Serial.begin(115200);
  feetechBus.begin();

  while (Serial.read() != 'a') {
    Serial.println("press a to continue...");
    delay(1000);
  }

  {
    int motor_id_old = -1;

    // search motor id
    for (int i = 0; i < 254; ++i) {
      FeetechServo motor(i, &feetechBus);
      if (motor.ping()) {
        motor_id_old = i;
        break;
      }
      delay(20);
    }

    if (motor_id_old == -1) {
      Serial.println("motor not found");
      return;
    }

    Serial.printf("press y to change motor id from %d to %d:\r\n", motor_id_old, MOTOR_ID_NEW);
    while (Serial.read() != 'y') {
    }

    FeetechServo motor(motor_id_old, &feetechBus);
    uint8_t buf;

    // unlock
    buf = 0;
    if (!motor.writeData(0x37, &buf, sizeof(buf))) {
      Serial.println("unlock error");
      return;
    }
    delay(100);

    // change id
    buf = MOTOR_ID_NEW;
    motor.writeData(0x05, &buf, sizeof(buf));
    delay(100);
  }

  {
    FeetechServo motor(MOTOR_ID_NEW, &feetechBus);
    uint8_t buf;

    // lock
    buf = 1;
    if (!motor.writeData(0x37, &buf, sizeof(buf))) {
      Serial.println("lock error");
      return;
    }
    delay(100);

    // ping test
    if (!motor.ping()) {
      Serial.println("ping failed");
    }
  }

  Serial.println("success");
}

void loop() {}
