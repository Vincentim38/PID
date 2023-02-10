#include <Arduino.h>

// PID tuning parameters
const float Kp = 1.0;
const float Ki = 0.1;
const float Kd = 0.01;

// Setpoint value
const float setpoint = 50.0;

// PID control variables
float error, P, I, D;
float last_error, integral;



void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);
  pinMode(MOTOR, OUTPUT);
}
}

void loop() {
  // Read process value from sensor
  float processValue = analogRead(A0);

  // Calculate error
  error = setpoint - processValue;

  // Calculate P term
  P = Kp * error;

  // Calculate I term
  integral += error;
  I = Ki * integral;

  // Calculate D term
  D = Kd * (error - last_error);

  // Store error for next iteration
  last_error = error;

  // Calculate control output
  float output = P + I + D;

  // Control output to avoid saturation of actuator
  if (output > 255) {
    output = 255;
  } else if (output < 0) {
    output = 0;
  }

  // Send control output to actuator
  analogWrite(DAC1, output);

  // Print data to serial monitor
  Serial.print("Process Value: ");
  Serial.print(processValue);
  Serial.print(" | Output: ");
  Serial.println(output);

  delay(100);
}

/*
This code implements a basic PID control algorithm to control the process value, which is read from the analog input pin A0. The control output is then sent to the DAC1 pin.
The code uses the Serial library to print the process value and control output to the serial monitor for monitoring purposes.Note that this code is just a starting point,
and the PID parameters (Kp, Ki, and Kd) may need to be adjusted for your specific application.
*/
