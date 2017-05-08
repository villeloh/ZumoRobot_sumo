
/**
 * @file    Custom_Motor.c
 * @brief   Custom file for defining motor movement functions.
*/

#include <project.h>
#include <stdio.h>
#include "Motor.h"

void Custom_forward(uint8 speed);
void Custom_backward(uint8 speed);
void Turn(uint32 turn, int dir_flag);
void Ultrasharp_turn(uint32 delay, int dir_flag);

// Custom 'forward' function to calibrate the forward moving direction (default motor_forward() veers 
// to the left due to the physical differences between the two motors).
// NOTE: The motors should always be stopped by using the default function, because this function 
// will leave the left motor running if used for that purpose!
void Custom_forward(uint8 speed)
{
    MotorDirLeft_Write(0); // set LeftMotor forward mode
    MotorDirRight_Write(0); // set RightMotor forward mode
    PWM_WriteCompare1(speed + 15); 
    PWM_WriteCompare2(speed);
}

// Custom function for backward movement, similar to the forward function defined above.
void Custom_backward(uint8 speed)
{
    MotorDirLeft_Write(1); // set LeftMotor backward mode
    MotorDirRight_Write(1); // set RightMotor backward mode
    PWM_WriteCompare1(speed + 15); 
    PWM_WriteCompare2(speed);
}

// A new turn function that takes direction as an argument.
// NOTE: the normalization of 'turn' within acceptable limits (0-240) is done in main.c, to make it more transparent.
void Turn(uint32 turn, int dir_flag)
{   
    if (dir_flag == 0) 
    {
        // Turns left
        PWM_WriteCompare1(255 - turn); 
        PWM_WriteCompare2(240);

    }     
    else if (dir_flag == 1) {      
        
        // Turns right
        PWM_WriteCompare2(240 - turn); 
        PWM_WriteCompare1(255);
        
    }
}

// Used in the sumo logic, to get away quickly from the black line. 
// Turns very fast (at max speed) by running the motors in opposite directions.
void Ultrasharp_turn(uint32 delay, int dir_flag)
{
    // left turn
    if (dir_flag == 0)
    { 
        MotorDirLeft_Write(1);
        MotorDirRight_Write(0);
        PWM_WriteCompare1(255);
        PWM_WriteCompare2(240);
        CyDelay(delay);
    
      // right turn
    } else if (dir_flag == 1) {
    
        MotorDirLeft_Write(0);
        MotorDirRight_Write(1);
        PWM_WriteCompare1(255);
        PWM_WriteCompare2(240);
        CyDelay(delay);
    
    }

}
