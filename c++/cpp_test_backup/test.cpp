#include <stdio.h>    // Used for printf() statements
#include <wiringPi.h> // Include WiringPi library!

// Pin number declarations. We're using the Broadcom chip pin numbers.

const int rightStepPin = 4; // right motor step, relative to robot camera
const int rightDirPin = 27;  // right motor direction, High = moves inward towards RPI, 
const int buttonPin = 5; // Active-low button - Broadcom pin 17, P1 pin 11



int flipDir (int pin)
{

    /*if(digitalRead(pin))
    {
	return LOW;
    }

    else
    {
        return HIGH;
    }
    */

	return (digitalRead(pin)-1)*(digitalRead(pin)-1);

}


int main(void)
{
    // Setup stuff:
    wiringPiSetupGpio(); // Initialize wiringPi -- using Broadcom pin numbers

    pinMode(rightStepPin, OUTPUT); 	// Set right motor step pin to output mode
    pinMode(rightDirPin, OUTPUT);     	// Set right motor direction pin to output mode
    pinMode(buttonPin, INPUT);      	// Set button as INPUT
    pullUpDnControl(buttonPin, PUD_UP); // Enable pull-up resistor on button
    
    float timeDelay = 0.3; // time in ms (0.0003 seconds)
    
    digitalWrite(rightDirPin, HIGH);

    printf("MOTOR IS RUNNING.\n");

    // Loop (while(1)):
    while(1)
    {
	
        if (digitalRead(buttonPin)) // Button is released if this returns 1
        {
            //digitalWrite(rightDirPin, flipDir(rightDirPin));     // Flip Motor Direction
        }

	else
	{
	    delay(100);

            while (!digitalRead(buttonPin)) // If digitalRead returns 0, button is pressed
            {
                // Do some blinking on the ledPin:
                digitalWrite(rightStepPin, HIGH); // Turn LED ON
                delay(timeDelay); // Wait 75ms
                digitalWrite(rightStepPin, LOW); // Turn LED OFF
                delay(timeDelay); // Wait 75ms again

            }
	    //printf("DIRECTION : %d\n", digitalRead(rightDirPin));

	    digitalWrite(rightDirPin, flipDir(rightDirPin));
	}
    }

    return 0;
}
