/**
* @mainpage ZumoBot Project
* @brief    PROJECT LATTE -- Sumo version of the robot project.
*/
/**
 * @file    main.c
 * @brief   
 * @details  ** You should enable global interrupt for operating properly. **<br>&nbsp;&nbsp;&nbsp;CyGlobalIntEnable;<br>
*/

#include <project.h>
#include <stdio.h>
#include "Motor.h"
#include "Ultra.h"
#include "Reflectance.h"
#include "IR.h"

// Declare used functions.
int rread(void);
void motor_start();
void motor_forward(uint8 speed,uint32 delay);
void motor_turn(uint8 l_speed, uint8 r_speed, uint32 delay);
void reflectance_set_threshold(uint16_t l3, uint16_t l1, uint16_t r1, uint16_t r3);
void Measure_Voltage();
void Custom_forward(uint8 speed);
void Custom_backward(uint8 speed);
void Turn(uint32 turn, int dir_flag);
void Ultrasharp_turn(uint32 delay, int dir_flag);

int main()
{
    // ==================== CRUCIAL DEFINITIONS ======================================= //
    
    // (Maximum) movement speed of the robot.
    uint8 speed = 200;
    
    // Reflectance thresholds (determined experimentally) for use in different movement behaviours.
    int black_threshold_l3 = 15000; // actual line edge value: somewhere betwen 20 000 - 21 000.
    int black_threshold_l1 = 18000; // 'sure bet' working value: 17 500 // actual line edge value: ~16 000 // 18 000
    int black_threshold_r1 = 22500; // 'sure bet' working value: 22 000 // actual line edge value: ~18 000 // 22 600
    int black_threshold_r3 = 15000; // actual line edge value: somewhere between 20 000 - 21 500.
 
    // ==================== OTHER STUFFS ======================================= //
        
    // Needed for using the button to start the robot's movement routine.
    int buttonPress = 0;
    uint8 button; // make button exist
    
    // Needed for stopping at the first black line and waiting for IR signal.
    int firstStop_flag = 0;

    // Direction flag for correct turning behaviour. '0' = 'left turn', '1' = 'right turn'.
    int dir_flag;
    
    // ==================== ACTUAL START ======================================= //
 
    // Initialize various starting thingies
    struct sensors_ ref;
    struct sensors_ dig;
    CyGlobalIntEnable; 
    UART_1_Start();
    ADC_Battery_Start();   
    Ultra_Start();
    
    Measure_Voltage(); // measure battery voltage at robot start
    
    motor_start(); // start the motor
    motor_forward(0, 0); // stop the motor at robot start, as it seems to run at a low speed if simply started up
    
    // Initialize IR sensors.
    sensor_isr_StartEx(sensor_isr_handler); 
    reflectance_set_threshold(black_threshold_l3, black_threshold_l1, black_threshold_r1, black_threshold_r3);  
    reflectance_start();
    IR_led_Write(1);

    BatteryLed_Write(0); // Switch led off 
  
    // Wait for button press to start the robot's movement routine.
    while (buttonPress == 0) 
    {
        button = SW1_Read();
        
        if (button == 0) 
        {
            buttonPress = 1;
            CyDelay(10);
        }
    }
    
    while (firstStop_flag == 0)
    {
    
        // Go forward at low speed until meeting the first horizontal black line.
        // Then wait for the IR signal to proceed.
        Custom_forward(speed/2.5);
        
        reflectance_read(&ref);
        reflectance_digital(&dig);
        
        if (dig.l3 == 0 && dig.r3 == 0)
        {
            motor_forward(0,0);
            firstStop_flag = 1;
        }
        
        CyDelay(1);
    
    }
    
    // Wait for IR signal. Upon getting it, proceed forward.
    get_IR();

    // ==== SUMO LOGIC ('THE SPIRAL HUNTER™') ============================================================== //

	int turnFactor = 12000;
	int outwardFlag = 1;
    int turn_flag = 1;   
    uint32 turnDel;

    // Full speed ahead for 0.7 seconds (to reach the center).    
    Custom_forward(speed);
    CyDelay(700); // <== experimental value; enough to make it to the center, or close to it.

    // Turn to the left (direction is arbitrary at this juncture).
	dir_flag = 0;
    
    // Loop for running the main 'hunt' logic in.
    while (1) 
    {
        
        CyDelay(1);
   
        if (turn_flag == 1) 
        {

            // This logic results in the robot moving in a spiral pattern (until it detects the other robot, which is when 
            // it speeds straight ahead in a 'hunting move'). The first spiral will run from the center-point to the outer edge; 
            // then back to the center; then back out again; etc. (However, see NOTE at the very bottom.)
            if (outwardFlag == 1) 
            {
            	turnFactor--; // drops to zero in a bit over 12 seconds
            	if (turnFactor <= 0) 
        	    {
    	            outwardFlag = 0;
                    turnFactor = 0;
            	}
                
             } else if (outwardFlag == 0) {
                
        	    turnFactor++; // ~12 seconds to get back to 12000
        	    if (turnFactor >= 12000)
        	    {
        	        outwardFlag = 1;
                    turnFactor = 12000;
                }
             }

        // Execute the turn.    
        Turn(turnFactor/50, dir_flag);
        
        }
        
        // read blackness value
        reflectance_read(&ref);
        reflectance_digital(&dig);
     
        // 'Ramming' logic.
        // If the ultrasound sensor detects an object within 20 'units', 
        // drive towards it at full speed and disable turn logic (until meeting black line).
        if (Ultra_GetDistance() < 20) // <== experimental 'hunt distance'
        {
            Custom_forward(speed);
    	    turn_flag = 0;
        }
        
                    
        // (These ifs could be refined further, but it's more work than it's worth, imo.)      
        // If both sensors are activated, back up for a bit and then re-start outward turn.
        if (dig.l3 == 0 && dig.r3 == 0) {
            
            Custom_backward(140);
            CyDelay(400); // temp value
            MotorDirLeft_Write(0);
            MotorDirRight_Write(0);
            dir_flag = 0;
	        turnFactor = 12000;
	        outwardFlag = 1;
	        turn_flag = 1;
        
          // If the left sensor is activated, turn sharply to the right and then begin inward spiral turn.  
        }  else if (dig.l3 == 0 && ref.r3 >= 10000) {
        
            Custom_backward(140);
            CyDelay(400); // temp value
            MotorDirLeft_Write(0);
            MotorDirRight_Write(0);
            dir_flag = 0;
	        turnFactor = 12000;
	        outwardFlag = 1;
	        turn_flag = 1;
     
        } else if (dig.r3 == 0 && ref.l3 >= 10000) {
        
            Custom_backward(140);
            CyDelay(400); // temp value
            MotorDirLeft_Write(0);
            MotorDirRight_Write(0);
            dir_flag = 0;
	        turnFactor = 12000;
	        outwardFlag = 1;
	        turn_flag = 1;

        }  else if (dig.l3 == 0) {
            
            turnDel = 520000/ref.l3; // 480k is an experimental constant; leads to a very sharp turn
            Ultrasharp_turn(turnDel,1);
            MotorDirLeft_Write(0);
            MotorDirRight_Write(0);
            dir_flag = 1;
    	    turnFactor = 5000; // some 'head start' is needed to make the turn sharp enough
    	    outwardFlag = 0;
    	    turn_flag = 1;
                
          // If the right sensor is activated, turn sharply to the left and then begin inward spiral turn.    
        } else if (dig.r3 == 0) {
                
            turnDel = 520000/ref.r3; // // 480k is an experimental constant; leads to a very sharp turn
            Ultrasharp_turn(turnDel,0);
            MotorDirLeft_Write(0);
            MotorDirRight_Write(0);
            dir_flag = 0;
	        turnFactor = 5000; // some 'head start' is needed to make the turn sharp enough
	        outwardFlag = 0;
	        turn_flag = 1;
        } 

    	// NOTE: The robot will get 'desynched' after one or more 'hunt' episodes, because the angle of approach to the black line affects the new 'starting point' 
    	// of the 'reset' turn logic, meaning that the robot will overrun the center-point when turnFactor reaches max value... Then the new spiral will be 
    	// similarly 'desynchronized'; etc. The perfect spiral pattern that was in effect before the hunt(s) can never be reached again, because there's no 
    	// way to find the center-point again after the very beginning. However, it should not matter a whole lot, because the overall movement pattern will 		
    	// remain spiral-ish in spite of these distortions.
        
    }
        
}
    
// ===================================================================================================================== //
// ===================================================================================================================== //

/* Don't remove the functions below */
int _write(int file, char *ptr, int len)
{
    (void)file; /* Parameter is not used, suppress unused argument warning */
	int n;
	for(n = 0; n < len; n++) {
        if(*ptr == '\n') UART_1_PutChar('\r');
		UART_1_PutChar(*ptr++);
	}
	return len;
}

int _read (int file, char *ptr, int count)
{
    int chs = 0;
    char ch;
 
    (void)file; /* Parameter is not used, suppress unused argument warning */
    while(count > 0) {
        ch = UART_1_GetChar();
        if(ch != 0) {
            UART_1_PutChar(ch);
            chs++;
            if(ch == '\r') {
                ch = '\n';
                UART_1_PutChar(ch);
            }
            *ptr++ = ch;
            count--;
            if(ch == '\n') break;
        }
    }
    return chs;
}
/* [] END OF FILE */
