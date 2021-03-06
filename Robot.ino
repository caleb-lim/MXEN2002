/* **********************************************************************************************
 *  Controller.ino
 *  
 *  Vehicle program
 *  
 *  Description:
 *  This prgram contains the function to allow manual control of the vehicle and alternatively 
 *  controlling the vehicle automonously via distance sensors.  
 *  
 *  Version History
 *  
 *  Date          Version    Description of changes
 *  ------------ ---------   -------------------------------------------------------------- 
 *  20200525      0.1         Initial Version. 
 *  20200605      0.2         Fixed autonomous operation
 *  20200610      1.0         Clean up code for final submition    
 *
 *  AUTHORS: Caleb Lim
 ************************************************************************************************/
#include "Controller.h"
#include <math.h>


//----------------------
//     Constants
//----------------------
#define MIN_DISTANCE_TO_SIDEWALL          10  
#define MIN_DISTANCE_TO_FRONTWALL         15
#define MOTOR_COMPARE_REGISTER_MIDPOINT   2000

#define MSG_DURATION_MS                   500
#define SENDING_RATE_OF_MSG_MS            100

#define MOTOR_COMPARE_REGISTER_TOP        400
#define SERVO_COMPARE_REGISTER_TOP        20000

#define ADJUSTMENT_TOLERANCE              3

//----------------------
//   Global Variables
//----------------------

static char serial_string[200] = {0};

//turning variables
static const uint16_t robotRevolutionTime = 10400;

//COMS variables
uint8_t recvDataByteManual=0, recvDataByteVert=0, recvDataByteHorz=0, recvDataByteServo=0; // data bytes recieved
uint8_t frontSensorRead=0, leftSensorRead=0, rightSensorRead=0;                            // data bytes sent

uint32_t last_send_ms = 0, lastMsgReceived = 0, current_ms = 0;
uint8_t serial_fsm_state = 0, serial_byte_in = 0;

bool msgRecieved = false;
bool autonomousMode = false;


//----------------------
//      Functions
//----------------------

//=============================================
// Sensor bit values conversion to cm
//============================================
static int sensorFrontDisplay(int sensorReading)
{
  int distanceValue =  4361.4*pow(sensorReading,-0.927)-5;

  if(distanceValue>80)
    distanceValue = 80;
  if(distanceValue<10)
    distanceValue = 10;
    
  return distanceValue;
}

static int sensorRightDisplay(int sensorReading)
{
  int distanceValue = 2390.7*pow(sensorReading,-1.039)+2.5;
 

  if(distanceValue>30)
    distanceValue = 30;
  if(distanceValue<3)
    distanceValue = 3;

  return distanceValue;
}

int sensorLeftDisplay(int sensorReading)
{
  int distanceValue = 2254.4*pow(sensorReading,-0.727)-15;

  if(distanceValue>30)
    distanceValue = 30;
  if(distanceValue<3)
    distanceValue = 3;
    
  return distanceValue;
}
//=======================================================
// End of Sensor bit values conversion to cm
//=======================================================

//=======================================================
// Manual Mode Command
//=======================================================
void operateCommandDrive()
{
      ///     MAUNAL CONTROLS     ///
      //motors
      float joystickPosition [2] = {mapValues ((float)recvDataByteHorz, 253.0f, -1.0f, 1.0f), mapValues ((float)recvDataByteVert, 253.0f, 1.0f, -1.0f)};
      motorWheelDirection (joystickPosition);

      //servo
      OCR4A = mapValues (recvDataByteServo, 253,1000,2000);
}
//=======================================================
// End of Manual Mode Command
//=======================================================

//=======================================================
// Autonomous Mode Command
//=======================================================
void operateAutonomousDrive()
{
        //go forward naturally
        if (frontSensorRead >= MIN_DISTANCE_TO_FRONTWALL)
        {  
            //Robot adjusts direction when driving a straight corridor 
            if (leftSensorRead - rightSensorRead > ADJUSTMENT_TOLERANCE ) //turn left
            {
                executeTurnLeft();
            } 
            else if (rightSensorRead - leftSensorRead > ADJUSTMENT_TOLERANCE ) //turn right
            {
                executeTurnRight();
            } 
            else
            {
                executeForward();
            }
        }
  
     else
     {
            if(leftSensorRead < MIN_DISTANCE_TO_SIDEWALL && rightSensorRead > MIN_DISTANCE_TO_SIDEWALL) //turn hard right
            {
                   executeTurnRight();         
            }
            else if(rightSensorRead < MIN_DISTANCE_TO_SIDEWALL && leftSensorRead > MIN_DISTANCE_TO_SIDEWALL) //turn hard left
            {
                   executeTurnLeft();
            }
            else //reverse
            {    
                  executeReverse();
            }
     }   
}
//=======================================================
// End of Autonomous Mode Command
//=======================================================

/**
 * This fuction will stop the robot on command
 */
void stopRobot ()
{
  float stopMatrix [2] = {0, 0};
  motorWheelDirection (stopMatrix);
}

/**
 * This function will set a direction for each motor wheel
 */
void motorWheelDirection (float joystickPosition[])
{
        /*
          JOY POS TO MOTOR DIR
          
          FWD = -1, 0
          BCK = 1, 0
          LFT = 0, 1
          RGT = 0, -1 
        */
        
        float motorMapping [2] =
        {
          joystickPosition[0] + joystickPosition[1], joystickPosition[1] - joystickPosition[0]
        };
        
        //normalization
        if (motorMapping[0] > 1.0f)
        motorMapping[0] = 1.0f;
        if (motorMapping[0] < -1.0f)
        motorMapping[0] = -1.0f;
        if (motorMapping[1] > 1.0f)
        motorMapping[1] = 1.0f;
        if (motorMapping[1] < -1.0f)
        motorMapping[1] = -1.0f;
      
        //set compare values for pwm generation
        OCR1A = (MOTOR_COMPARE_REGISTER_MIDPOINT / 2) - (int)(motorMapping[0] * (MOTOR_COMPARE_REGISTER_MIDPOINT / 2));
        OCR1B = (MOTOR_COMPARE_REGISTER_MIDPOINT / 2) + (int)(motorMapping[0] * (MOTOR_COMPARE_REGISTER_MIDPOINT / 2));
        OCR3A = (MOTOR_COMPARE_REGISTER_MIDPOINT / 2) + (int)(motorMapping[1] * (MOTOR_COMPARE_REGISTER_MIDPOINT / 2));
        OCR3B = (MOTOR_COMPARE_REGISTER_MIDPOINT / 2) - (int)(motorMapping[1] * (MOTOR_COMPARE_REGISTER_MIDPOINT / 2));
        
        //sprintf (serial_string, "x: %3d y:%3d || M1: %3d %3d M2: %3d %3d\r", (int)(joystickPosition [0] * 100), (int)(joystickPosition [1] * 100), OCR1A, OCR1B, OCR3A, OCR3B);
        //serial0_print_string (serial_string);
}


// ====================================================================
// Vehicle operation section
// ====================================================================
void executeForward()
{
    sprintf(serial_string,"forward: - f: %3d || l: %3d || r: %3d \n",frontSensorRead,leftSensorRead,rightSensorRead);
    serial0_print_string(serial_string);
    
    float forward [2] = {mapValues (133, 253.0f, -1.0f, 1.0f), mapValues (0, 253.0f, 1.0f, -1.0f)};
    motorWheelDirection (forward);
}

void executeReverse()
{
    sprintf(serial_string,"reverse: - f: %3d || l: %3d || r: %3d \n",frontSensorRead,leftSensorRead,rightSensorRead);
    serial0_print_string(serial_string);
    
    float reverse [2] = {mapValues (133, 253.0f, -1.0f, 1.0f), mapValues (253, 253.0f, 1.0f, -1.0f)};
    motorWheelDirection (reverse);
}

void executeTurnRight()
{
     sprintf(serial_string,"right: - f: %3d || l: %3d || r: %3d \n",frontSensorRead,leftSensorRead,rightSensorRead);
     serial0_print_string(serial_string);
     
     float turnRight [2] = {mapValues (0, 253.0f, -1.0f, 1.0f), mapValues (133, 253.0f, 1.0f, -1.0f)};
     motorWheelDirection (turnRight);
}

void executeTurnLeft()
{
    sprintf(serial_string,"left: - f: %3d || l: %3d || r: %3d \n",frontSensorRead,leftSensorRead,rightSensorRead);
    serial0_print_string(serial_string);
    
    float turnLeft [2] = {mapValues (253, 253.0f, -1.0f, 1.0f), mapValues (133, 253.0f, 1.0f, -1.0f)};
    motorWheelDirection (turnLeft);
}

// ====================================================================
// End of Vechicle operation section
// ====================================================================

/**
 * This function send message to the controller
 */
void sendMessageToController()
{
    if (frontSensorRead>253)
      frontSensorRead = 0;
    if (leftSensorRead>253)
      leftSensorRead = 0;
    if (rightSensorRead>253)
      rightSensorRead = 0;
        
    last_send_ms = current_ms;
    serial2_write_byte(0xFF); //Send start byte
    serial2_write_byte(frontSensorRead);  //send first data byte: must be scaled to the range 0-253
    serial2_write_byte(leftSensorRead);  //send second parameter: must be scaled to the range 0-253
    serial2_write_byte(rightSensorRead);  //send second parameter: must be scaled to the range 0-253
    serial2_write_byte(0xFE); //Send sMOTOR_COMPARE_REGISTER_TOP byte

    sprintf(serial_string,"left: - f: %3d || l: %3d || r: %3d \n",frontSensorRead,leftSensorRead,rightSensorRead);
    serial0_print_string(serial_string);
}

/*
 * This function initialises motors and servo timers. Also globally enable interrupts. 
 */
void initialiseVariables()
{
  //Set distance sensor input pins
  DDRF &= ~(1<<0)|~(1<<1)|~(1<<2); //Sensors

  //Sets timers up
  DDRB |= (1<<5)|(1<<6);                        //Set OC1A and OC1B as OUTPUT
  TCCR1A |= (1<<COM1A1)|(1<<COM1B1);            //Set on down, clear on up (OC1A & OC1B)
  TCCR1B |= (1<<CS10);                          //SET PRESCALER to 1
  TCCR1B |= (1<<WGM11)|(1<<WGM13);              //Select PWM Mode (Phase & frequency Correct)[MODE 10]
  ICR1 = MOTOR_COMPARE_REGISTER_TOP;            //Set compare register 400
  
  //Motor 2
  DDRE |= (1<<3)|(1<<4);                        //Set OC3A and OC3B as Output
  TCCR3A |= (1<<COM3A1)|(1<<COM3B1);            //Set on down, clear on up (OC3A & OC3B)
  TCCR3B |= (1<<CS10);                          //Set PRESCALER to 1
  TCCR3B |= (1<<WGM31)|(1<<WGM33);              //Select PWM Mode (Phase and frequency correct) [MODE 10]
  ICR3 = MOTOR_COMPARE_REGISTER_TOP;            //Set compare register 400
  
  //Servo
  DDRH |= (1<<3);                             //Set OC4A as Output
  TCCR4A |= (1<<COM4A1);                      //Set on down, clear on up (OC4A)
  TCCR4B |= (1<<CS11);                        //Set PRE to 8
  TCCR4B |= (1<<WGM41)|(1<<WGM43);            //Select PWM Mode (Phase and frequency correct) [MODE 10]
  ICR4 = SERVO_COMPARE_REGISTER_TOP;          //Set compare register 20000
 
  //Globally enable interrupts
  sei ();
  
  UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
}

/*
 * This function maps values 
 */
float mapValues (float a, float b, float c, float d)
{
  return ((a / b) * (d - c)) + c;
}

/*
 * This function will recieve data bytes from the controller and set them in variables
 */
ISR (USART2_RX_vect)
{
    serial_byte_in = UDR2;
    
    switch (serial_fsm_state)
    {
        case 0:
            //do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
            break;
        
        case 1:
            recvDataByteManual = serial_byte_in;
            serial_fsm_state ++;
            break;
        
        case 2:
            recvDataByteHorz = serial_byte_in;
            serial_fsm_state ++;
            break;
        
        case 3:
            recvDataByteVert = serial_byte_in;
            serial_fsm_state ++;
            break;
        
        case 4:
            recvDataByteServo = serial_byte_in;
            serial_fsm_state ++;
            break;
        
        case 5:
        if (serial_byte_in == 0xFE)
        {
            msgRecieved = true;
        }
        serial_fsm_state = 0;
        break;
    } 
    
    if (serial_byte_in == 0xFF)
        serial_fsm_state = 1;
}
//-------------------------------------------------------------------//
//   Main function                                                   //
//-------------------------------------------------------------------//
int main(void)
{
    //------------------------
    //    Initialisation
    //------------------------
    bool operating = false;
    
    adc_init();
    milliseconds_init();
    serial2_init();
    serial0_init();
    _delay_ms(20);
    initialiseVariables();
    
    //------------------------
    //  Program Loop
    //------------------------
    while(true)
    {
        //Read sensor values (bit)
        uint16_t sensorLeft = adc_read(1);
        uint16_t sensorFront = adc_read(0);
        uint16_t sensorRight = adc_read(2);

        //Convert bit values to distance
        frontSensorRead =  sensorFrontDisplay(sensorFront);
        leftSensorRead = sensorLeftDisplay(sensorLeft);
        rightSensorRead =  sensorRightDisplay(sensorRight);

        //DEBUG
        //sprintf(serial_string,"f: %3d || l: %3d || r: %3d \n",frontSensorRead,leftSensorRead,rightSensorRead);
        //serial0_print_string(serial_string);
  
        //Get current time
        current_ms = milliseconds;

        //Determine if robot is operation
        if (current_ms - lastMsgReceived >= MSG_DURATION_MS)
          operating = false;
        else
          operating = true;
   
     
        if (msgRecieved)
        {
            //DEBUG
            //sprintf (serial_string, "1: %3d || 2: %3d || 3: %3d || 4: %3d\n", recvDataByteManual, recvDataByteHorz, recvDataByteVert, recvDataByteServo);
            //serial0_print_string(serial_string);
          
            lastMsgReceived = current_ms;
            msgRecieved = false;

           //Operation
            if (operating)
            {
                //set autonomy mode
                autonomousMode = recvDataByteManual == 1;
                if (autonomousMode) 
                  operateAutonomousDrive();    
                   
                //set manual mode
                else   
                  operateCommandDrive();         
            }
        }
    
        //Send message to controller
        if (current_ms - last_send_ms >= SENDING_RATE_OF_MSG_MS)
        {
             sendMessageToController();  
        }

        //Inform if the robot is in operation
        if (!operating)
        {
          stopRobot();     
          //servo speed signal
          OCR4A = 1870;
          //autonomy mode disable
          autonomousMode = false;
        } 
     }
     return(1);
}
