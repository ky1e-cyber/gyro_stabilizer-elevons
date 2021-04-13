#define DEBUG

#include <TroykaIMU.h>
#include <Multiservo.h>

#define INPUT_X 7
#define INPUT_Y 6
#define INPUT_Z 5

#define OUTPUT_VSTAB 0
#define OUTPUT_ELEV_R 1
#define OUTPUT_ELEV_L 2

#define SIGNAL_X_RIGHT 57
#define SIGNAL_X_ZERO 44
#define SIGNAL_X_LEFT 32

#define SIGNAL_Y_UP 56
#define SIGNAL_Y_ZERO 44
#define SIGNAL_Y_DOWN 32

#define SIGNAL_Z_LEFT 31
#define SIGNAL_Z_ZERO 44
#define SIGNAL_Z_RIGHT 56

#define SPEED_X 9000
#define SPEED_Y 4500
#define SPEED_Z 9000

#define ELEV_ANGLE 30
#define ELEV_ANGLE_MAX 45
#define VSTAB_ANGLE 45

#define PID_X_KP 0.4
#define PID_X_KI 0.01

#define PID_Y_KP 0.4 
#define PID_Y_KI 0.01

volatile uint8_t width_x;
volatile uint8_t width_y;
volatile uint8_t width_z;

volatile bool readFlag = false;

int32_t t_speed_x;
int32_t t_speed_y;
int32_t t_speed_z;

int32_t gyro_zeros_x;
int32_t gyro_zeros_y;

int32_t prev_speed_x;
int32_t prev_speed_y;

Gyroscope gyro;

Multiservo vstab;
Multiservo elev_r;
Multiservo elev_l;

void readSignal() {
  width_x, width_y, width_z = 0;
  while (digitalRead(INPUT_X) | digitalRead(INPUT_Y) | digitalRead(INPUT_Z)) {
    width_x += digitalRead(INPUT_X);
    width_y += digitalRead(INPUT_Y);
    width_z += digitalRead(INPUT_Z);
  }
  readFlag = true;
}

void servoLoop(int32_t speed_x, int32_t speed_y, int32_t speed_z) {
  int16_t angle_elev_r = map(speed_x, -SPEED_X, SPEED_X, -ELEV_ANGLE, ELEV_ANGLE);;
  int16_t angle_elev_l = angle_elev_r * -1;
  int16_t delta_angle = map(speed_y, -SPEED_Y, SPEED_Y, -ELEV_ANGLE, ELEV_ANGLE) * -1;
  int16_t angle_vstab = map(speed_z, -SPEED_Z, SPEED_Z, -VSTAB_ANGLE, VSTAB_ANGLE);
  angle_elev_r = constrain(angle_elev_r + delta_angle, -ELEV_ANGLE_MAX, ELEV_ANGLE_MAX);
  angle_elev_l = constrain(angle_elev_l + delta_angle, -ELEV_ANGLE_MAX, ELEV_ANGLE_MAX);
  elev_r.write(90 + angle_elev_r);
  elev_l.write(90 + angle_elev_l);
  vstab.write(90 + angle_vstab);
}

int32_t computePID(int32_t gyro_speed, int32_t set_speed, int32_t &prev_speed, float kp, float ki, int32_t minOut, int32_t maxOut) {
  int32_t err = set_speed - gyro_speed;
  int32_t pid_speed = constrain(set_speed + (int(kp * err) + int(ki * (err - prev_speed))), minOut, maxOut);
  prev_speed = gyro_speed;
  return pid_speed;
}


void setup() {
  pinMode(INPUT_X, INPUT);
  pinMode(INPUT_Y, INPUT);
  pinMode(INPUT_Z, INPUT);
  gyro.begin();
  vstab.attach(OUTPUT_VSTAB);
  elev_r.attach(OUTPUT_ELEV_R);
  elev_l.attach(OUTPUT_ELEV_L);
  #ifdef DEBUG
  Serial.begin(9600);
  Serial.println("Do not move, calibrating gyro...");
  #endif
  gyro.begin();
  gyro_zeros_x = int(gyro.readRotationDegX() * 100);
  gyro_zeros_y = int(gyro.readRotationDegY() * 100);
  while (digitalRead(INPUT_X)) { // Wait for signal to end
    1;
  }
  while (!digitalRead(INPUT_X)) { // Wait for signal to start
    1;
  }
  attachInterrupt(digitalPinToInterrupt(INPUT_X), readSignal, RISING);
}

void loop() {
  if (readFlag) {
    readFlag = false;
    if (width_x > SIGNAL_X_ZERO) {
      t_speed_x = map(width_x, SIGNAL_X_ZERO, SIGNAL_X_RIGHT, 0, SPEED_X);
    }
    else {
      t_speed_x = map(width_x, SIGNAL_X_ZERO, SIGNAL_X_LEFT, 0, SPEED_X) * -1;
    }
    
    if (width_y > SIGNAL_Y_ZERO) {
      t_speed_y = map(width_y, SIGNAL_Y_ZERO, SIGNAL_Y_UP, 0, SPEED_Y) * -1;
    }
    else {
      t_speed_y = map(width_y, SIGNAL_Y_ZERO, SIGNAL_Y_DOWN, 0, SPEED_Y);
    }

    if (width_z > SIGNAL_Z_ZERO) {
      t_speed_z = map(width_z, SIGNAL_Z_ZERO, SIGNAL_Z_RIGHT, 0, SPEED_Z);
    }
    else {
      t_speed_z = map(width_z, SIGNAL_Z_ZERO, SIGNAL_Z_LEFT, 0, SPEED_Z) * -1;
    }

    servoLoop(
      computePID(int(gyro.readRotationDegY() * 100) - gyro_zeros_x, t_speed_x, prev_speed_x, PID_X_KP, PID_X_KI, -SPEED_X, SPEED_X),
      computePID(int(gyro.readRotationDegX() * 100) - gyro_zeros_y, t_speed_y, prev_speed_y, PID_Y_KP, PID_Y_KI, -SPEED_Y, SPEED_Y),
      t_speed_z
      );
  }
}
