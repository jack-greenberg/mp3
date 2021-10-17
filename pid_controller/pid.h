// A struct to hold the values of our PID controller
typedef struct {
  float kp; // proportional
  float ki; // integral
  float kd; // derivative
  
  float tau; // derivative low-pass filter time constant
  
  // Output Limits
  int lim_min;
  int lim_max;
  
  float T; // Time step
  
  float integrator; 
  float prev_error; 
  float differentiator; 
  float prev_measurement;
  
  float out;

} PID;

float pid_step(PID* pid, float setpoint, float measurement);
