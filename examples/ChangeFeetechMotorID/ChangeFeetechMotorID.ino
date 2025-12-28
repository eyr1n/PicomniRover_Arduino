#include <FeetechServo.h>

// 新しいFeetechモーターID
#define MOTOR_ID_NEW 2

FeetechBus feetech_bus;

void setup() {
  Serial.begin(115200);
  feetech_bus.begin();

  {
    int motor_id_old;

    // search motor id
    int i = 0;
    while (true) {
      FeetechServo motor(i, &feetech_bus);
      if (motor.ping()) {
        motor_id_old = i;
        break;
      }
      i++;
      i %= 254;
      delay(20);
    }

    while (true) {
      Serial.printf("press y to change motor id from %d to %d:\r\n", motor_id_old, MOTOR_ID_NEW);
      if (Serial.available() > 0 && Serial.read() == 'y') {
        break;
      }
      delay(1000);
    }

    FeetechServo motor(motor_id_old, &feetech_bus);
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
    FeetechServo motor(MOTOR_ID_NEW, &feetech_bus);
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
