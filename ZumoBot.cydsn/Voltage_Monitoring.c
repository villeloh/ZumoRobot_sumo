
/**
 * @file    Voltage_Monitoring.c
 * @brief   Custom file for measuring battery voltage (+ playing some warning sounds & flashing lights).
 * @details  <br><br>
    <p>
    <B>General</B><br>
    The function Measure_Voltage() was used during development to measure the robot's battery voltage level -- once at robot start and then every 80 seconds 
    (or thereabouts) while the robot was on. The logic was deleted from main.c in order not to interfere with the race and sumo contest; however, it is  
    included here for review.<br>
    <br><br>
    </p>
*/

#include <project.h>
#include <stdio.h>
#include "Motor.h"
#include "Beep.h"

float ADC_result_to_volts(int16 adcresult);
void Beep(uint32 length, uint8 pitch);
void BatteryLed_Write(uint8 value);
uint8 BatteryLed_Read();
void motor_stop();
void Measure_Voltage();

// Subroutine for custom unit conversion (used below in Measure_Voltage()).
float ADC_result_to_volts(int16 adcresult)
{
    float volts;
    
    // returns the actual battery voltage (4.1 - 5.6 V)
    volts = (adcresult / 4095.0) * (1.5 * 5.0);
    
    return volts;
    
}

// For measuring the voltage of the batteries at regular intervals. (Used in main.c.)
void Measure_Voltage()
{
    int16 adcresult = 0;
    float volts = 0.0;
    
    int i;
    
    ADC_Battery_StartConvert();
    if(ADC_Battery_IsEndConversion(ADC_Battery_WAIT_FOR_RESULT)) 
    {   // wait for get ADC converted value
        adcresult = ADC_Battery_GetResult16();
        volts = ADC_result_to_volts(adcresult); // convert value to volts
        
        // Different types of beep sounds for different voltage levels. If the charge is over 5.0 V, there will be 
        // three long and low-sounding beeps; from 5.0 to 4.5 V, six medium-length, medium-pitch beeps; from 4.5 V to 
        // 4.1 V, 9 loud and sharp beeps; and finally if the voltage reaches 4.1 V, there will be a never-ending 
        // flurry of extremely short and sharp beeps.
        // EDIT: Added flashing light effects to go with the beeps :)
        if (volts > 0.5) // a lenient 'null check' is needed to prevent alarms when the robot is off but the chip is on. NOTE: we still had continuous beeping
                         // as startup occasionally, so the problem was not fully fixed with this check.
        {
        
            if (volts >= 5.0)
            { for (i = 0; i < 3; i++) 
                {
                    Beep(80, 60);
                    BatteryLed_Write(1);
                    BatteryLed_Read();
                    CyDelay(600);
                    BatteryLed_Write(0);
                }
            } else if (volts < 5.0 && volts >= 4.5) {
             for (i = 0; i < 6; i++) 
                {
                    Beep(40, 80);
                    BatteryLed_Write(1);
                    BatteryLed_Read();
                    CyDelay(500);
                    BatteryLed_Write(0);
                }            
            } else if (volts < 4.5 && volts >= 4.1) {
            
                for (i = 0; i < 9; i++) 
                {
                    Beep(30, 95);
                    BatteryLed_Write(1);
                    BatteryLed_Read();
                    CyDelay(400);
                    BatteryLed_Write(0);
                }       
            } else if (volts < 4.1) {
                
                motor_stop(); // what else needs to be stopped?
            
                while (1) 
                {
                    Beep(20, 110);
                    BatteryLed_Write(1);
                    BatteryLed_Read();
                    CyDelay(200);
                    BatteryLed_Write(0);
                    printf("%f \n", volts);
                    printf("DANGER! Battery LOW! Please charge ASAP!!!\n");
                }                      
            }
                  
            // Print bit & voltage values
            printf("%d %f\r\n",adcresult, volts);
        }
            CyDelay(500);
    }
    
}
