#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <cmath>

#define RAD_TO_DEG 57.295779513082320876798154814105
#define DEG_TO_RAD 0.017453292519943295769236907684886

Adafruit_MPU6050 mpu;

unsigned long previousTime = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;

const float gravity = 9.81; /* m/s^2 */
const float ACCEL_UNCERTAINTY = 0.05;  /* m/s^2 */
const float GYRO_UNCERTAINTY  = 0.05;  /* rad/s*/
const float FAIL_BENCH = 0.15;  /* m/s */ 
const float weightLbs = 180; /* lbs */
const float kg_to_lbs = 2.2046;
const float weightKgs = weightLbs / kg_to_lbs ; /* kg */

float pitch = 0;
float roll = 0;
float vel = 0;
float pitch_RAD = 0;
float roll_RAD = 0;
float global_a = 0;
float vI = 0;
float tot_a= 0;
float force = 0;
float added_weight = 0;


void setup(void) {
  Serial.begin(115200);
  
  while (!Serial)
    delay(10); 

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("");
  delay(100);
  
  startTime = micros();
  previousTime = startTime;
}

while (True){
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Set time */
  unsigned long currentTime = micros();
  float t = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  float pitch_angle = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG; /* Finding forward / backwards tilt angle + Conversion to DEG*/
  float roll_angle = atan2(-a.acceleration.x, sqrt(a.acceleration.z * a.acceleration.z + a.acceleration.y * a.acceleration.y )) * RAD_TO_DEG; /* Finding side to side tilt angle + Conversion to DEG*/

  /* Clearing noise in vibrations and other discrepancies */
  pitch = 0.98 * (pitch + (g.gyro.x * RAD_TO_DEG) * t) + 0.02 * pitch_angle; 
  roll = 0.98 * (roll + (g.gyro.y * RAD_TO_DEG) * t) + 0.02 * roll_angle;
  
  /* Return to RAD*/
  pitch_RAD = pitch * DEG_TO_RAD;
  roll_RAD = roll * DEG_TO_RAD;

  /* Finds glabal acceleration produced during the movement using the result of the dot product of the roll and pitch rotation matrixes*/
  global_a = (-sin(pitch_RAD) * a.acceleration.x + sin(roll_RAD)*cos(pitch_RAD) * a.acceleration.y + cos(roll_RAD)*cos(pitch_RAD) * a.acceleration.z); 

  /* Takes out gravity gives total output acceleration from the user*/
  tot_a = global_a - gravity;

  Serial.print("X: ");
  Serial.print(a.acceleration.x);

  Serial.print(" Y: ");
  Serial.print(a.acceleration.y);

  Serial.print(" Z: ");
  Serial.print(a.acceleration.z);

  Serial.print(" Global: ");
  Serial.print(global_a);

  Serial.print(" Total: ");
  Serial.println(tot_a);

//   vel = vI + tot_a * t; /* Uses Kinematics equation to find the velocity */
// 
//   float tot_gyro = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z); /* Total gyroscope data magnitude */
//   
//   /*If the bar isn't moving set the velocity to 0 */
//   if (fabs(tot_a) < ACCEL_UNCERTAINTY && tot_gyro < GYRO_UNCERTAINTY) {
//     vel = 0;
//     
//   }
//   
//   /* If the velocity is bigger than 0 and equal to or less than 0.15 max bench press has been reached */
//   if (vel > 0 && vel >= FAIL_BENCH - 0.02 && vel <= FAIL_BENCH + 0.02) {
//     Serial.println("Max bench has been reached ");
//     Serial.print("Speed in m/s: ");  
//     Serial.println(vel);
//     continue; /* Max has been achieved, not neccesary to do the max weight calculations */
//   }
// 
//   vI = vel;
//   
//   /* Newtons 2nd law - Calculates the the surplus of extra force used during the non maxed out movements and finds the extra weight that can be pushed to find the max weight */
//   force = weightKgs * tot_a;
//   added_weight = (force / gravity) * kg_to_lbs;
//   
//   Serial.print("Speed in m/s: ");
//   Serial.println(vel);
//   Serial.print("Max Bench weight is: ");
//   Serial.print(weightLbs + added_weight);
//   Serial.println(" lbs");
//   Serial.println("")
}
void loop() {
  
  
}

