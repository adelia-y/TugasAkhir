/*
 * Program to measure the value of R0 for a know RL at fresh air condition
 * Program by: B.Aswinth Raj
 * Website: www.circuitdigest.com                                       
 * Dated: 28-12-2017
 */
//This program works best at a fresh air room with temperaure Temp: 20℃, Humidity: 65%, O2 concentration 21% and when the value of Rl is 47K
#include <Arduino.h>

#define RL 47  //The value of resistor RL is 47K
#define AMMONIA_SENSOR 34

void setup() {
    Serial.begin(115200); //Initialise serial COM for displaying the value
    pinMode(AMMONIA_SENSOR, INPUT);
}

void loop() {
    float analog_value;
    float VRL;
    float Rs;
    float Ro;
    
    for (int test_cycle = 1 ; test_cycle <= 500 ; test_cycle++) {
        analog_value = analog_value + analogRead(AMMONIA_SENSOR); //add the values for 200
    }
    
    analog_value = analog_value/500.0; //Take average
    
    Serial.print("Raw avg analog val: ");
    Serial.println(analog_value);

    VRL = analog_value*(5.0/1023.0); //Convert analog value to voltage
    Serial.print("Voltage: ");
    Serial.println(VRL);

    //RS = ((Vc/VRL)-1)*RL is the formulae we obtained from datasheet
    Rs = ((5.0/VRL)-1) * RL;
    
    //RS/RO is 3.6 as we obtained from graph of datasheet
    Ro = Rs/3.6;
    
    // Serial.print("Ro at fresh air = ");
    // Serial.println(Ro); //Display calculated Ro
    delay(1000); //delay of 1sec
}