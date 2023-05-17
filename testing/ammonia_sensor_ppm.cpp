/*

 * Program to measure gas in ppm using MQ sensor

 * Program by: B.Aswinth Raj

 * Website: www.circuitdigest.com

 * Dated: 28-12-2017

 */
#include <Arduino.h>

#define RL 47  //The value of resistor RL is 47K
#define m -0.263 //Enter calculated Slope 
#define b 0.42 //Enter calculated intercept
#define Ro 20 //Enter found Ro value

#define AMMONIA_SENSOR 34 //Sensor is connected to A4

void setup() {
    Serial.begin(115200);
    pinMode(AMMONIA_SENSOR, INPUT);

    delay(2000); //Wait for display to show info
}


void loop() {

    float VRL; //Voltage drop across the MQ sensor
    float Rs; //Sensor resistance at gas concentration 
    float ratio; //Define variable for ratio


    VRL = analogRead(AMMONIA_SENSOR)*(5.0/1023.0); //Measure the voltage drop and convert to 0-5V
    Rs = ((5.0*RL)/VRL)-RL; //Use formula to get Rs value
    ratio = Rs/Ro;  // find ratio Rs/Ro
    
    float ppm = pow(10, ((log10(ratio)-b)/m)); //use formula to calculate ppm
    Serial.print("NH3 (ppm) = "); //Display a ammonia in ppm
    Serial.println(ppm);

    delay(1000);
}