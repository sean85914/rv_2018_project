// Left  motor C1 -> Pin2
// Right motor C1 -> Pin3
// Left  motor C2 -> Pin10
// Right motor C2 -> Pin!1
// Don't forget to power encoders on!
#include "math.h"
// Structure Parameters (unit: meter)
#define WIDTH 0.291
#define RADIUS 0.125
// For debug
#define DEBUG 0
// Encoder Parameters
#define PPR 990
// Pin Parameters
int encoderL = 10;
int encoderR = 11;
// Distance motor traveled per tick (unit: meter)
float dis_per_tick = 2* PI * RADIUS / PPR;
float disL;
float disR;
// Encoder Parameters
volatile int encoderLNow = 0;
volatile int encoderRNow = 0;
int encoderLPrior = 0;
int encoderRPrior = 0;
// Vehicle Speed Parameters (unit: m/s)
double v_x;
double v_y;
double omega;
// Motor Speed Parameters (unit: m/s)
double v_L;
double v_R;
// Location Parameters (unit: m and rad)
double x = 0;
double y = 0;
double theta = 0;
double dtheta;
// Time Parameters
unsigned long dt = 100; // Unit: ms, 10Hz
unsigned long now;
unsigned long _time;
void setup() {
  // Declare interrupt pin and function
  attachInterrupt(0, EncoderL, RISING); // Pin 2, left motor, call EncoderL when rising
  attachInterrupt(1, EncoderR, RISING); // Pin 3, right motor, call EncoderR when rising
  // Set pin mode
  pinMode(encoderL, INPUT); // Pin 10
  pinMode(encoderR, INPUT); // Pin 11
  // Start Serial for data transmit 
  Serial.begin(9600);
  // Update time 
  now = millis();
}

void loop() {
    //Serial.print("Encoder L: "); Serial.print(encoderLNow); Serial.print("\t; Encoder R: "); Serial.println(encoderRNow);
  if(millis() - now >= dt)
  {
    _time = millis() - now; // Unit: ms
    disL = dis_per_tick * (encoderLNow - encoderLPrior);
    disR = dis_per_tick * (encoderRNow - encoderRPrior);
    v_L = disL / _time * 1000;
    v_R = disR / _time * 1000;
    dtheta = (disR - disL) / WIDTH;
    omega = dtheta / _time * 1000;
    v_x = (disR + disL) / 2 * cos(theta + dtheta / 2.) / _time * 1000;
    v_y = (disR + disL) / 2 * sin(theta + dtheta / 2.) / _time * 1000;
    x += (disR + disL) / 2 * cos(theta + dtheta / 2.);
    y += (disR + disL) / 2 * sin(theta + dtheta / 2.);
    theta += dtheta;
    // Set theta in range [0, 2*pi)
    if(theta >= 2 * PI ) theta -= 2 * PI;
    if(theta < 0 )       theta += 2 * PI;
    // Update all parameters
    encoderLPrior = encoderLNow;
    encoderRPrior = encoderRNow;
    now = millis();
  }
  // Display Information
  // Format: x y theta v_L v_R v_x v_y omega
  display();
}

void EncoderL()
{
  if (digitalRead(encoderL) == LOW) // Forward
  {
    encoderLNow += 1; 
    if(DEBUG) Serial.println("Left forward");
  }
  else // Backward
  {
    encoderLNow -= 1; 
    if(DEBUG) Serial.println("Left backward");
  }
}
void EncoderR()
{
  if (digitalRead(encoderR) == HIGH) // Forward
  {
    encoderRNow += 1; 
    if(DEBUG) Serial.println("right forward");
  }
  else // Backward
  {
    encoderRNow -= 1; 
    if(DEBUG) Serial.println("right backward");
  }
}
void display()
{
  // Format: x y theta v_L v_R v_x v_y omega
  Serial.print(x);Serial.print(" ");Serial.print(y);Serial.print(" ");Serial.print(theta);Serial.print("  ");Serial.print(v_L);
  Serial.print(" ");Serial.print(v_R);Serial.print(" ");Serial.print(v_x);Serial.print(" ");Serial.print(v_y);Serial.print(" ");Serial.println(omega);
}

