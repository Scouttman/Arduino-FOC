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
BLDCMotor motor = BLDCMotor(7); // 7 for austars model
BLDCDriver6PWM driver = BLDCDriver6PWM(PHASE_UH, PHASE_UL, PHASE_VH, PHASE_VL, PHASE_WH, PHASE_WL);
CurrentSensor currentSense = CurrentSensor();

// encoder instance
// Flipping A B to swap direction 
Encoder encoder = Encoder(ENCODER_A, ENCODER_B, 2048, ENCODER_Z); 
// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doI(){encoder.handleIndex();}


uint16_t adcBuffer1[ADC_BUF_LEN_1]; // Buffer for store the results of the ADC conversion
uint16_t adcBuffer2[ADC_BUF_LEN_2]; // Buffer for store the results of the ADC conversion
// Which part of the buffer read from is important 
// THe ADC sasmplign intuerpt is called for both the peak and of peak meaning that 1 should have no current in it
uint32_t prev_val = adcBuffer1[0];
bool toggle = false;
float volt_rest = 0;


void setup() { 
  
  Serial.begin(1000000);
//  Serial.begin(115200);
  Serial.println("Starting Init:");
  OG_MX_GPIO_Init();  
  currentSense.init();
  
  Serial.println("done");
  

  // initialize encoder sensor hardware
  encoder.init(); 
  encoder.enableInterrupts(doB, doA, doI); // TODO check if this is reversed

  // link the motor to the sensor
  motor.linkSensor(&encoder);
  motor.linkCurrentSensor(&currentSense);

  // power supply voltage [V] 
  driver.voltage_power_supply = 18;
  driver.init(); 
  // link driver
  motor.linkDriver(&driver);


  // aligning voltage [V] 
  motor.voltage_sensor_align = 1.5; 
  // index search velocity [rad/s] 
  motor.velocity_index_search = 20; 
  
  // set motion control loop to be used 
//  motor.controller = ControlType::velocity; 
  //motor.controller = ControlType::angle;
  motor.controller = ControlType::current; 

  // contoller configuration  
  // default parameters in defaults.h
  // velocity PI controller parameters 
  motor.PID_velocity.P = 0.05; 
  motor.PID_velocity.I = 1.0;

  motor.PID_current.P = 0.02; 
  motor.PID_current.I = 0.05;

  // default voltage_power_supply 
  motor.voltage_limit = 18;

//  motor.PI_current.P = 0.1; 
//  motor.PI_current.I = 0.01;

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
  motor.initFOC();


  Serial.println("Motor ready.");
  Serial.println("Set the target velocity using serial terminal:");
  Serial.println("Dones setting up current sensor");
}

// angle set point variable
float target_velocity = 100;
float target_current = 30;
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

uint16_t adcBufferTmp1[ADC_BUF_LEN_1];
uint16_t adcBufferTmp2[ADC_BUF_LEN_2];
float U_0, W_0, V_0 = 0;
float current_s;

void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz 
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
//  motor.move(target_velocity);
  motor.move(target_current);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  if(counter%200==0){
//    Serial.printf("Speed:%f",encoder.getVelocity());
//    motor.monitor();
//    float shaft_angle = motor.shaftAngle();
//    float elec_ang = _normalizeAngle(_electricalAngle(shaft_angle, 7));
//    Serial.print(elec_ang);
//    Serial.print("\t");
//    Serial.print(currentSense.test(0));
//    Serial.print("\t");
//    Serial.print(currentSense.test(1));
//    Serial.print("\t");
//    Serial.print(currentSense.test2(0));
//    Serial.print("\t");
//    Serial.print(currentSense.test(2));
//    Serial.print("\t");
//    Serial.print(currentSense.test(3));
//    Serial.print("\t");
//    Serial.print(currentSense.test2(1));
//    Serial.print(currentSense.test(0)-currentSense.test(2));
//    Serial.print("\t");
//    Serial.print(currentSense.test2(0)-currentSense.test2(1));
//    Serial.print("\t");
//    Serial.print(currentSense.test(1)-currentSense.test(3));
    Serial.print(currentSense.get_d());
    Serial.print("\t");
    Serial.print(currentSense.get_q());
    Serial.println();
  }
  counter++;

//  currentSense.updateCurrent(elec_ang);
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
