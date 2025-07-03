#include <PID_v1.h>     // calling the pid library which I found on the Arduino library Brett Beauregard under "PID"
#define LDR_IN 3 // A3 is defined as the LDR input    
#define LED_OUT 3 // D3 is defined as the LED output
double Setpoint, Input, Output,  LEDoutput;     //doubles provide a high accuracy in the value of the variable
double Kp = 0.8, Ki = 0.5, Kd = 0.01;   // these are the PID values. YOU MUST CHANGE THEM! Suggest you set all but one of them to zero and then increase that one until the input looks reasonable.
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);   //standard text to call up the PID code.




void setup()
{
  Serial.begin(9600);                                                //Begin Serial
  Input = analogRead(LDR_IN);     //read the value at A3 analogue pin input.
//LEDoutput = 50;    // sets the initial brightness of the LED  
Setpoint = 900;                // set the setpoint at 950 or change it. Note that this is just a value that bears no relationship to anything apart from being in the range of the values at A3 coming from the LDR i.e. from 0-1023. You should set your setpoint to whatever value you want. The PID will then need to control the output of the LED such that the LDR value (input) is at the setpoint value.
  myPID.SetMode(AUTOMATIC);    //standard code
  myPID.SetTunings(Kp, Ki, Kd);   //standard code
  myPID.SetOutputLimits(-100,-100);           //Set Output limits to -127 and 127 max. This does two things. First is that it limits the values Output can take to between -127 and 127. You can pick these to be any number. Secondly and more importantly without this line the PID code would set the Output at zero for any values for which the Input was less than the Setpoint and a negative Output should have been generated.
}




void loop()
{
  Input = analogRead(LDR_IN);     //read the value at A3 analogue pin input.
  myPID.Compute();     // This makes the PID compute the Output value. If the Input is the same as the Setpoint then the Output value should be zero as the error is zero.
    LEDoutput = 45 + Output;      // Very important part. The way PID works is that it looks at the input and the setpoint and determines the “distance” between them for the proportional term, the rate of change of the input for the differential term, and the amount of “area” in terms of time x “distance” the value of the input has been above or below the setpoint to obtain the integral term. It then multiplies each of these terms by the ki, ki and kd values and sums them up to obtain an Output value. This Output is then used to adjust whatever device is being used to alter the Input value. In this case it is an LED. You can see how I define LEDoutput and then write this to the LED. It is very important to note that I set the value of 58 as being very close to the value I know the LED brightness needs to have when the LDR has a value equal to the setpoint of 950. YOUR VALUE WILL BE DIFFERENT THAN 58.




     if(LEDoutput > 255)     // 255 is the max PWM signal for the LED and if you try to write a bigger number, it wouldnt be interpreted properly by the LED.
  {LEDoutput=255;}    // if the number is bigger than 255 then you set it to 255.
 
  if(LEDoutput < 1)     // 0 is the minimum PWM signal so if you try to write to the LED it will be confused. Here if the LEDoutput is below 1 then the value of 1 is used.
  {LEDoutput=1;}
 
    analogWrite(LED_OUT, LEDoutput);   //Write the value of LEDoutput to the LED from pin 3
    Serial.print("Setpoint is ");
    Serial.print(Setpoint);
    Serial.print("    Input is ");
    Serial.print(Input);
    Serial.print("    Output is ");
    Serial.print(Output);      // Note that the Output value doesnt need to be zero when the Input is equal to the Setpoint. Any non-zero value of Output will be there to fine tune the LEDoutput value (which will be close to 58) in order to get an Input = Setpoint.
    Serial.print("    LEDoutput is ");
    Serial.println(LEDoutput);
    Serial.println(1400);
   delay(40.8);    //You can adjust this. I just set it at 300 so that there was time to look at the values but there doesnt even need to be a delay.
}





// This code is a simple example of using the PID library to control an LED based on the input from an LDR (Light Dependent Resistor).