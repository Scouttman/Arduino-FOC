/**
 * 
 * 
 * The same example can be ran with the B-G431-ESC1 devkit
 * 
 */
#define B_G431
#include "main.h"
#include <SimpleFOC.h>

// motor instance
BLDCMotor motor = BLDCMotor(7); // 7 for austars model 7 for garatt
BLDCDriver6PWM driver = BLDCDriver6PWM(PHASE_UH, PHASE_UL, PHASE_VH, PHASE_VL, PHASE_WH, PHASE_WL);
InlineCurrentSense currentSense = InlineCurrentSense(0.03,64.0/7.0,1,0,2);

// encoder instance
// Flipping A B to swap direction 
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 512, ENCODER_Z); 
// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doI(){encoder.handleIndex();}


// Which part of the buffer read from is important 
// THe ADC sasmplign intuerpt is called for both the peak and of peak meaning that 1 should have no current in it
bool toggle = false;
float volt_rest = 0;
bool current_control = true;


void setup() { 
  
  Serial.begin(2000000);
//  Serial.begin(115200);
  Serial.println("Starting Init:");
//  OG_MX_GPIO_Init();  
  

  // initialize encoder sensor hardware
  encoder.init(); 
  encoder.enableInterrupts(doB, doA, doI); // TODO check if this is reversed

  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // power supply voltage [V] 
  driver.voltage_power_supply = 18;
  driver.init(); 
  // link driver
  motor.linkDriver(&driver);


  // aligning voltage [V] 
  motor.voltage_sensor_align = 2; 
  // index search velocity [rad/s] 
  motor.velocity_index_search = 10; 
  
  // set motion control loop to be used 
  motor.torque_controller = TorqueControlType::voltage; 
  motor.controller = MotionControlType::velocity;

  // contoller configuration  
  // default parameters in defaults.h
  // velocity PI controller parameters 
  motor.PID_velocity.P = 0.1; 
  motor.PID_velocity.I = 0; //0.01;
  motor.PID_velocity.D = 0;
  
  // default voltage_power_supply 
  motor.voltage_limit = 18;


  // velocity low pass filtering time constant 
  motor.LPF_velocity.Tf = 0.01; 

  // angle P controller 
  motor.P_angle.P = 20; 
  //  maximal velocity of the position control 
  motor.velocity_limit = 50;

  // comment out if not needed
  motor.useMonitoring(Serial);
  
  // align encoder and start FOC
  Serial.print("Align:"); 
  Serial.println(motor.voltage_sensor_align);
  // initialize motor
  motor.init();
  currentSense.init(); // Current sense init has to be called after PWM start 
  motor.initFOC();
  motor.linkCurrentSense(&currentSense);
  if(current_control){
    motor.torque_controller = TorqueControlType::foc_current; 
    motor.controller = MotionControlType::torque;
    motor.controller = MotionControlType::velocity;
    motor.controller = MotionControlType::angle;
    motor.controller = MotionControlType::spring_mass_damper;
    motor.PID_velocity.P = 0.04; //0.005;
    motor.PID_velocity.I = 0.001;
    motor.PID_velocity.D = 0.0;
    motor.PID_velocity.limit = 1;
    motor.P_angle.P = 10; 
  }
  motor.PID_current_q.P = 20; 
//  motor.PID_current_q.I = 20;
  motor.PID_current_d.P = 2.0;
  motor.PID_current_d.limit = 1.0;
  motor.PID_current_q.limit = 1.0;

  Serial.println("Motor ready.");
  Serial.println("Set the target velocity using serial terminal:");
}

// angle set point variable
float target_velocity = 50;
float target_current = 0.04;
float target_angle = 0;
int counter = 0;
float max_volt = 0;
float min_volt = 100;
float adc_volt = 0;
float currentU = 0;
float currentV = 0;
float currentW = 0;
float i_d = 0;
float i_q = 0;
float f_d = 0;
float f_q = 0;
float elecAng = 0;

float U_0, W_0, V_0 = 0;
float current_s;

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC(true);

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  if(motor.controller == MotionControlType::torque){
    motor.move(target_current);
  }else if(motor.controller == MotionControlType::velocity){
    motor.move(target_velocity);
  }else if(motor.controller == MotionControlType::angle){
    motor.move(target_angle);
  }else if(motor.controller == MotionControlType::spring_mass_damper){
    motor.move(target_angle);
  }

  // function intended to be used with serial plotter to monitor motor variables
//  // significantly slowing the execution down!!!!
//    float shaft_angle = motor.shaftAngle();
//    float elec_ang = _normalizeAngle(_electricalAngle(shaft_angle, 7));
//    
//  if(counter%100==0){
//    DQCurrent_s m_current = motor.getCurrents();//currentSense.getFOCCurrents(elec_ang);
////    write_float(elec_ang);
////    write_float(m_current.q);
////    Serial.println();
//  }
//  counter++;
  // user communication
//  serialReceiveUserCommand();

}

// utility function enabling serial communication with the user to set the target values
// this function can be implemented in serialEvent function as well
void serialReceiveUserCommand() {
  
  // a string to hold incoming data
  static String received_chars;
  
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {
      
      // change the motor target
      target_velocity = received_chars.toFloat();
      Serial.print("Target velocity: ");
      Serial.println(target_velocity);
      
      // reset the command buffer 
      received_chars = "";
    }
  }
}


void write_float(float val){
   byte * b = (byte *) &val;
   Serial.write(b[3]);
   Serial.write(b[2]);
   Serial.write(b[1]);
   Serial.write(b[0]);
}
