/* **********************************************************************************************
 *  Controller.ino
 *  
 *  Controller program
 *  
 *  Description:
 *  This prgram contains the function to control the robotic vehicle using the arduino
 *  
 *  Version History
 *  
 *  Date          Version    Description of changes
 *  ------------ ---------   -------------------------------------------------------------- 
 *  20200525      0.1         Initial Version. 
 *  20200603      0.2         Completed Manual Mode
 *  20200609      0.3         Fixed LCD to display Sensor Reading
 *  20200610      1.0         Clean up code for final submition
 *
 *  AUTHORS: Caleb Lim
 ************************************************************************************************/
#include "Controller.h"

// file scope variables
static char serial_string[200] = {0};  // declare and initialise string for serial printing
static char lcd_string[33] = {0};  // declare and initialise string for LCD

//-----------------------------
//        Declerations
//----------------------------- 
uint8_t sendDataByteVert=0, sendDataByteHorz=0, sendDataByteServo=0;   // data bytes sent
uint8_t recvDataByteFront=0, recvDataByteLeft=0, recvDataByteRight=0;   // data bytes received

uint8_t serial_fsm_state=0; // used in the serial receive section

boolean msgRecieved = false;
boolean roboticMode = false;

//-----------------------------
//        Functions
//-----------------------------

/*
 * This function will start and stop manual mode with a botton
 */
ISR (INT0_vect)
{
    static const uint8_t debounceDelay = 100;
    static uint32_t lastPressedTime = 0;
    
    if (milliseconds > lastPressedTime + debounceDelay)
    {
        roboticMode = !roboticMode;
        lastPressedTime = milliseconds;
    }
}

/**
 * This function will convert a 10 bit to an 8 bit value
 */
int convertBit(float a, float b, float c, float d)
{
    return ((a/b)*(d-c))+c;
}

/**
 * This will recieve sensor values from the vehicle arduino
 */
ISR (USART2_RX_vect)
{
  uint8_t serial_byte_in = UDR2;
  
  switch (serial_fsm_state)
  {
    case 0:
    //do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
    break;
    
    case 1:
        recvDataByteFront = serial_byte_in;       //Front Sensor Value
        serial_fsm_state ++;
        break;
    
    case 2:
        recvDataByteLeft = serial_byte_in;        //Left Sensor Value
        serial_fsm_state ++;
        break;
    
    case 3:
        recvDataByteRight= serial_byte_in;       //Right Sensor Value
        serial_fsm_state ++;
        break;
    
    case 4:
        if (serial_byte_in == 0xFE)
        {
          msgRecieved= true;
        }
        serial_fsm_state = 0;
        break;
   }
  
    if (serial_byte_in == 0xFF)
    {
      serial_fsm_state = 1;
    }
}

//-------------------------------------------------------------------//
//   Main function                                                   //
//-------------------------------------------------------------------//
int main(void) 
{
  
        //-----------------------------
        //INITIALISATION
        //----------------------------- 
        serial0_init();       // initialise serial subsystem
        serial2_init ();
        adc_init();           // initialse ADC
        lcd_init();           // initialise
        milliseconds_init();  // initialise timer3 to track milliseconds
        _delay_ms(20);


        //button interrupts
        DDRD &= ~(1<<PD0);    // INT0  is also PD0 and we set the DDR to input
        PORTD |= (1<<PD0);    // enable pullupresistor on PD0
        EICRA &= ~(1<<ISC00); // INT0 to trigger on a FALLING edge
        EIMSK |= (1<<INT0);   // enable INT0 & INT1
        sei();                // globally enable interrupts

        UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)

        //joystick pins set to inputs
        DDRF &= ~(1<<0)|~(1<<1); //Joystick Left
        DDRK &= ~(1<<6)|~(1<<7); //Joystick Right       

        //LED set to outputs
        DDRA |= (1<<PA0);

        uint32_t current_ms=0, last_send_ms=0;        // used for timing the serial send
   
        while (1) 
        {
                //LED light control // ON = Automatic Mode OFF = Manual Mode 
                if(!roboticMode)
                {
                      PORTA &= ~(1<<PA0);
                }
                else
                {
                      PORTA |= (1<<PA0);
                }
                
                //Read Joystick Values
                uint16_t horzRead = adc_read(0); //read the voltage at ADC0
                uint16_t vertRead = adc_read(1); //read the voltage at ADC1 
                uint16_t servoRead = adc_read(15); //read the voltage at ADC0

                current_ms = milliseconds;

                //Sending to vehicle arduino
                if(current_ms-last_send_ms >= 100) //sending rate controlled here one message every 100ms (10Hz)
                {
                    sendDataByteVert = convertBit(vertRead, 1023 , 0, 253);
                    sendDataByteHorz = convertBit(horzRead, 1023 , 0, 253);
                    sendDataByteServo = convertBit(servoRead, 1023 , 0, 253);
                    
                    if (sendDataByteVert>253)
                      sendDataByteVert = 0;
                    if (sendDataByteHorz>253)
                      sendDataByteHorz = 0;
                    if (sendDataByteServo>253)
                      sendDataByteHorz = 0;

                    last_send_ms  = current_ms;
                    serial2_write_byte(0xFF);                 //send start byte = 255
                    serial2_write_byte(roboticMode);          //send first data byte:   Robotic Mode
                    serial2_write_byte(sendDataByteVert);     //send second data byte:  Verticle 
                    serial2_write_byte(sendDataByteHorz);     //send third parameter:   Horizontal
                    serial2_write_byte(sendDataByteServo);    //send fourth data byte:  Servo 
                    serial2_write_byte(0xFE);                 //send stop byte = 254
                }
                
                //Print Sensor Values to LCD
                if(msgRecieved)
                {
                     sprintf (serial_string, "F: %3d L: %3d R: %3d\n", recvDataByteFront, recvDataByteLeft, recvDataByteRight);
                     serial0_print_string(serial_string);
        
                     lcd_goto (0x00);
                     sprintf (lcd_string, "F: %3d || L: %3d", recvDataByteFront, recvDataByteLeft);
                     lcd_puts (lcd_string);
        
                     lcd_goto (0x40);
                     sprintf (lcd_string, "R: %3d", recvDataByteRight);
                     lcd_puts (lcd_string);
        
                     msgRecieved = false;
                }
                
        }
        return(1);
}
