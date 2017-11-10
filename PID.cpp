#include "PID.h"
#include <iostream>

using namespace std;

PID::PID() {}

PID::~PID() {}

double p_error;
double i_error;
double d_error;

//initialize Kp, Ki, Kd, and errors
void PID::Init(double Kp, double Ki, double Kd) {
  this -> Kp = Kp;
  this -> Ki = Ki;
  this -> Kd = Kd;
   
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

//update errors; sequence matters
void PID::UpdateError(double cte) {
  //update d_error first
  d_error = cte - p_error;
  //then assign p_error
  p_error = cte;
  //finally increase i_error
  i_error += cte;
}

//return steering value
double PID::GetOutput() {
   // negation due to CCW
   return -(Kp * p_error + Ki * i_error + Kd * d_error);
}
