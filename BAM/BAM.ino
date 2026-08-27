#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TM2LRP8Exw1k"
#define BLYNK_TEMPLATE_NAME "BAM"
#define BLYNK_AUTH_TOKEN "tlB5JPdD1yG7W0LN7ryF1riytCsvHwkn"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <cmath>

char ssid[] = "Micle's iphone";
char pass[] = "brawlstars";

Adafruit_MPU6050 mpu;

unsigned long previousTime = 0;
unsigned long startTime = 0;
unsigned long elapsedTime = 0;
unsigned long timeRacked = 0;

const float gravity = 9.81; 
const float ACCEL_UNCERTAINTY = 0.3;  
const float GYRO_UNCERTAINTY  = 0.05;
const float VEL_UNCERTAINTY = 0.012;  
const float FAIL_BENCH = 0.15;  
const float kg_to_lbs = 2.2046;
const float restRequired = 10;

float pitch = 0;
float roll = 0;
float vel = 0;
float pitch_RAD = 0;
float roll_RAD = 0;
float vI = 0;
float tot_a= 0;
float force = 0;
float added_weight = 0;
float estimated_max = 0;
float peak_vel = 0;
float peak_max = 0;
float weightLbs = 0; 
float weightKgs = 0;
float restCount = 0;

bool isRacked = false;
bool isTracking = false;
bool wasDescending = false;

// Sets weight value from Blynk app 
BLYNK_WRITE(V2) {
  weightLbs = param.asFloat();
  weightKgs = weightLbs / kg_to_lbs;
}

void setup(void) {
  Serial.begin(115200);
  
  unsigned long serialTimer = millis();
  while (!Serial && millis() - serialTimer < 2500){
    delay(10);
  } 

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

  // Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass); // Connect esp32 to internet
  
  startTime = micros();
  previousTime = startTime;
}

// "return" does the same as "continue" for a normal while or for loop in the loop() function
void loop() {
  // Blynk.run(); 

  // Defining and initalizing the variables of a and g 
  sensors_event_t a, g;
  mpu.getEvent(&a, &g, nullptr);

  unsigned long currentTime = micros();
  float t = (currentTime - previousTime) / 1000000.0;
  previousTime = currentTime;

  // Finds angles from away from Z axis using acceleration
  float pitch_angle = atan2(a.acceleration.y, sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;
  float roll_angle = atan2(-a.acceleration.x, sqrt(a.acceleration.z * a.acceleration.z + a.acceleration.y * a.acceleration.y )) * RAD_TO_DEG; 

  // Takes 98% accuracy of the gyroscope and 2% of the acceleration angles to dampen noise
  pitch = 0.98 * (pitch + (g.gyro.x * RAD_TO_DEG) * t) + 0.02 * pitch_angle; 
  roll = 0.98 * (roll + (g.gyro.y * RAD_TO_DEG) * t) + 0.02 * roll_angle;
  
  pitch_RAD = pitch * DEG_TO_RAD;
  roll_RAD = roll * DEG_TO_RAD;

  // Uses rotation matrix of Rx dot Ry and uses the 3rd row for Z axis acceleration calcualtions and find the total acceleration subtracting gravity
  tot_a = (-sin(pitch_RAD) * a.acceleration.x + sin(roll_RAD)*cos(pitch_RAD) * a.acceleration.y + cos(roll_RAD)*cos(pitch_RAD) * a.acceleration.z) - gravity; 
  float tot_gyro = sqrt(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z); 
  bool atRest = (fabs(tot_a )< ACCEL_UNCERTAINTY && tot_gyro < GYRO_UNCERTAINTY);

  if (!isTracking && wasDescending){
    if (atRest){
      restCount++;
      if (restCount >= restRequired){
        wasDescending = false;
        restCount = 0;
      }
    }
    else{
      restCount = 0;
    }
    return;

  }
  
  if (fabs(tot_a) >= ACCEL_UNCERTAINTY){
    if (!isTracking){
      Serial.println("Tracking");
      isTracking = true;
    }
  }
  else{
    tot_a = 0;
  }

  if (!isTracking){
    return;
  }

  Serial.print("X: ");
  Serial.print(a.acceleration.x);

  Serial.print(" Y: ");
  Serial.print(a.acceleration.y);

  Serial.print(" Z: ");
  Serial.print(a.acceleration.z);

  Serial.print(" Total: ");
  Serial.println(tot_a);
  
  
  vel = vI + tot_a * t; // Kinematics equation
  
  // Checks if the bar is racked 
  if (tot_a == 0 && tot_gyro < GYRO_UNCERTAINTY) {
    if (!isRacked){
      timeRacked = millis();
      isRacked = true;
    }
  } 
  else{
    isRacked = false;
  }
  
  // If bar is actually racked stop tracking, send the values to Blynk and reset variables to default values
  if (isRacked && millis() - timeRacked >= 3000){
    isTracking = false;
    
    Blynk.virtualWrite(V0, peak_vel);
    Blynk.virtualWrite(V1, peak_max);
    Blynk.syncVirtual(V2);

    peak_vel = 0;
    peak_max = 0;
    tot_a = 0;
    vel = 0;
    vI = vel;
    pitch = 0;              
    roll = 0;
    pitch_RAD = 0;
    roll_RAD = 0;
    timeRacked = 0;
    previousTime = micros();
    return;
  }

  if (vel > 0){
    // Finds estimated max
    force = weightKgs * tot_a;
    added_weight = (force / gravity) * kg_to_lbs;
    estimated_max = weightLbs + added_weight;

    if (vel > peak_vel){
    
      peak_vel = vel;
    }

    // checks if max is more than last
    if (estimated_max > peak_max){
      peak_max = estimated_max;
    }

    vI = vel;
    // Finds if the current weight is max 
    if (vel >= FAIL_BENCH - VEL_UNCERTAINTY && vel <= FAIL_BENCH + VEL_UNCERTAINTY) {
      // checks if velocity is bigger than the last 
      if (vel > peak_vel){
        
        peak_vel = vel;
      }
      // checks if max is bigger than last
      if (estimated_max > peak_max){
        peak_max = estimated_max;
      }
      return; 
    } 
  } 
  else{
    vel = 0;
    isTracking = false;
    wasDescending = true;
    vI = vel;
    return;
  }
}
