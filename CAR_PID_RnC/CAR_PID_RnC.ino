#include <PID_v1.h>

#define POS_SENSOR_INPUT A0
#define VEL_SENSOR_INPUT A1
#define ACCELERATOR_OUTPUT 3

double Setpoint_position, Input_position, Output_velocity, Input_velocity, Output_acceleration;
double* Setpoint_velocity = &Output_velocity; //Instead of doing Setpoint_velocity = Output_velocity; everywhere this is better

//initial tuning parameters
double Kp_1=2, Kp_2=4, Ki_1=5, Ki_2=3, Kd_1=1, Kd_2=1;  //Random values

PID velocityPID(&Input_velocity, &Output_acceleration, Setpoint_velocity, Kp_1, Ki_1, Kd_1, DIRECT);
PID positionPID(&Input_position, &Output_velocity, &Setpoint_position, Kp_2, Ki_2, Kd_2, DIRECT);

void setup()
{
  //initialize the variables we're linked to
  Input_position = analogRead(POS_SENSOR_INPUT);
  Input_velocity = analogRead(VEL_SENSOR_INPUT);
  Setpoint_position = 10;   // 10 meters
  //Setpoint_velocity = Output_velocity;
  
  //make inner loop 10X faster than Outer loop so 
  //that inner constants have little effect on the whole
  velocityPID.SetSampleTime(10);
  positionPID.SetSampleTime(100);
  
  //turn the PID on
  velocityPID.SetMode(AUTOMATIC);
  positionPID.SetMode(AUTOMATIC);
}

void loop()
{
  Input_velocity = analogRead(VEL_SENSOR_INPUT);
  Input_position = analogRead(POS_SENSOR_INPUT);  //Assuming our sensors give analog outputs
  velocityPID.Compute();
  positionPID.Compute();
  analogWrite(ACCELERATOR_OUTPUT, Output_acceleration);//Assuming output device takes a PWM signal
}
