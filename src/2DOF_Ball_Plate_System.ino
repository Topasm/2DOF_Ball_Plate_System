

#include <Servo.h>
#include <inttypes.h>
#include <stdlib.h>

  // Touch Panel
  #include <usbhid.h>
  #include <hiduniversal.h>
  #include <usbhub.h>  
  #include "le3dp_rptparser.h"

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

  USB                                             Usb;
  USBHub                                          Hub(&Usb);
  HIDUniversal                                    Hid(&Usb);
  JoystickEvents                                  JoyEvents;
  JoystickReportParser                            Joy(&JoyEvents);

  static uint32_t touch_x;
  static uint32_t touch_y;
  static uint32_t touch_x_global;
  static uint32_t touch_y_global;
 
Servo servo1; //servo on y axis
Servo servo2; // x servo

// -------------------- Constants --------------------
// -- Working Constants: --
#define outMin -43
#define outMax 43

// -- Servo Pins: --
#define servopin1 5
#define servopin2 6
// -------------------- End Constants -----------------

// -------------------- Variables --------------------
// -- Position Vars: --
double xO, yO; //setpoint
double xI, yI; //actual 
double lastXI = 0, lastYI = 0;

// -- Error Vars: --
double error[2];//holds current errror
double lastError[2] = {0,0};
double lastLastError[2] = {0,0};

// ----- Output Vars: -----
double output[3]; //Hold actual output

// -- Computations Vars: --
double ITerm[2] = {0,0};
double lastDVal[2] = {0,0};
double lastLastDVal[2] = {0,0};
unsigned long now, deltaT, lastT;

// -- Tunings Vars: --


double setkp[2] = {0.005,0.0075};  //max 0.02 0.03
double setki[2] = {0.0015,0.00225};
double setkd[2] = {0.001,0.0015};

//double setkp[2] = {0.04,0.04};
//double setki[2] = {0.04,0.04};
//double setkd[2] = {0.01,0.01};

double setka[2] = {1,1}; //{1,1};
double kp[2], ki[2], kd[2], ka[2]; // working tunings
int compT; //computation time in ms

// -- Control Vars: --
bool aggTuningSet = false, compute_bool = true, setpointChange = false;
char controlByte = '1';
int servo1Rotate, servo2Rotate;
int auxTimeCount = 0, methodCount = 0;

// -------------------- End Variables --------------------

void setup()
{
  Serial.begin( 19200 );
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  Serial.println("Start");

  if (Usb.Init() == -1)
      Serial.println("OSC did not start.");

  delay( 200 );

  if (!Hid.SetReportParser(0, &Joy))
      ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1  );

        
  // ---------- Servos ----------
  // attach servo to pwn pins:
  servo1.attach(servopin1);
  servo2.attach(servopin2);
  // level the servos:
  servo1.write(47); //90 position is 92
  servo2.write(42); //90 position is 95
  // ---------- End Servos ----------
  
  // ---------- PID Init ----------
  //  These values can be initially set or set in real time
  xO =2000;
  yO = 2000;
  compT = 10;
  setPIDTunings(false);
  // ---------- End PID Init ----------
}



// #################### Loop1 - Touch ####################
void loop()
{

  // if control byte calls for PID calculations, do so:
  if(compute_bool)
  {
    if(controlPID())
    {
      writeServos();
      if(xI==0 && yI==0)
      {
        servo1.write(47); //90 position is 92
        servo2.write(42); //90 position is 95
      }
    }
  }
  
  //Touch
  Usb.Task();
  JoyEvents.GetTouchData(&touch_x, &touch_y);

  xI = touch_x;
  yI = touch_y;
  
    //Serial.print("touch_x=");
    Serial.print(xI);  
    Serial.print(" ");
    //Serial.print(", touch_y=");
    Serial.print(yI);
    Serial.print(" ");
    Serial.print(4000);  
    Serial.print(" ");
    Serial.print(2500);  
    Serial.print(" ");
    Serial.print(2000);  
    Serial.print(" ");
    Serial.print(1500);  
    Serial.print(" ");
    Serial.println(0);  
    }

// -------------------- PID Methods --------------------
// ---------- Limit Output ----------
// Limits output from Compute PID to keep
// servo output within user defined bounds
void limitOutput(int i)
{
  if(output[i] > outMax) output[i] = outMax;
  else if(output[i] < outMin) output[i] = outMin;
  if(ITerm[i] > outMax) ITerm[i] = outMax;
  else if(ITerm[i] < outMin) ITerm[i] = outMin;
}
// ---------- End Limit Output ----------

// ---------- Control PID ----------
// Main control function that calls and controls
// PID algorithm, returns true if PID was recalculated
bool controlPID()
{
  // check the elapsed time since last compute:
  now = millis(); // current time
  deltaT = now - lastT; // last compute time
  
  if(deltaT >= compT) // if compT time has elapsed
  { 
    for(int i=0; i<2; i++) // for each servo
    { 
      JoyEvents.GetTouchData(&touch_x, &touch_y);
      getError(i); // finds error values
      JoyEvents.GetTouchData(&touch_x, &touch_y);
      computePID(i); // computes output from PID algorithm
      JoyEvents.GetTouchData(&touch_x, &touch_y);
      limitOutput(i); // limit the output
    }
    lastT = now; //keep track of last compute time    
    return true; //PID was calculated
  }
  else return false; //PID wasn't calculated
}
// ---------- End Control PID ----------

// ---------- Compute PID ----------
// PID algorithm that calculates the output
// for each servo
void computePID(int i)
{

  // to mitigate touchscreen errors, make sure current
  // error is within 10 of last error. If not, ignore it.
  if(controlByte == '0' || setpointChange)
    if(abs(error[i] - lastError[i]) > 10) error[i] = lastError[i];
  
  // calculate each PID term individually:
  double PTerm = error[i] * kp[i];
  ITerm[i] += (ki[i] * error[i]);
  double DVal = (error[i] - lastError[i]);
  double DTerm = kd[i] * (((5*DVal) + (3*lastDVal[i]) + (2*lastLastDVal[i]))/10.0);
  double ATerm = ka[i] * (((DVal - lastDVal[i]) + (lastDVal[i] - lastLastDVal[i]))/2.0);
      
  // Calculate Output:
  output[i] = PTerm + ITerm[i] + DTerm + ATerm;

  // save some calculations for later:
  lastError[i] = error[i];
  lastLastDVal[i] = lastDVal[i];
  lastDVal[i] = DVal;
}
// ---------- End Compute PID ----------

// ---------- Get Error ----------
//  Computes the difference in the current
//  position and the perpendicular line of
//  action for each servo
void getError(int i)
{
  if (i == 0) // for servo1: (on y axis)
    error[0] = (yI - yO);

  if (i == 1) // for servo2 (x servo1)
    error[1] = (xI - xO);
}
// ---------- End Get Error ----------

// ---------- Set PID Parameters ----------
//  Because compute time is variable, this
//  method normalizes PID tunings against time
//  You can also select aggressive or standard tunings
//  by passing in a boolean variable (true = aggressive)
void setPIDTunings(bool aggressive)
{
  if(!aggressive)
    for(int i = 0; i < 2; i++)
    {
      kp[i] = setkp[i];
      kd[i] = setkd[i] / (compT/1000.0);
      ki[i] = setki[i] * (compT/1000.0);
      ka[i] = setka[i] * (compT/1000.0);
    }
  else
    for(int i = 0; i < 2; i++)
    {
      kp[i] = setkp[i];
      kd[i] = setkd[i] / (compT/1000.0);
      ki[i] = setki[i] * (compT/1000.0);
      ka[i] = setka[i] * (compT/1000.0);
    }
}
// ---------- End Set Parameters ----------

// ---------- Write Servos ----------
// This method sends the PID output to the Servos
void writeServos()
{
  servo1.write(47 + output[0]); //y
  servo2.write(42 - output[1]); //x
}
// ---------- End Write Servos ----------
// -------------------- End PID Methods --------------------

  
