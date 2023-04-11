#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
#include <Wire.h>


// ----- DEFINE VARIABLES
// Interfaces
#define ALARM_PIN 23 // DIGITAL OUTPUT PIN
// #define LCD_PIN_SCL 22 // I2C SCL PIN
// #define LCD_PIN_SDA 21 // I2C SDA PIN
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Chamber
#define AIR_VALVE 5 // DIGITAL OUTPUT PIN WHITE
#define AERATION_PUMP 19 // RED
#define WATER_PUMP 32 // YELLOW
#define WATER_VALVE 33 // GREEN

#define WATER_SENSOR_UPPER 36 // ADC DAC0
#define WATER_SENSOR_LOWER 39 // ADC DAC1
#define AMMONIA_SENSOR 34 // ADC

// Buttons
#define ALARM_BUTTON 16 
#define INC_BUTTON 17
#define DEC_BUTTON 18

#define delay_time 500 // ms

float ammonia_sensor = 0;
float water_sensor_upper = 0;
float water_sensor_lower = 0;

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

// ----- SETUP
void setup() {
    Serial.begin(115200);

    // LCD 20x4 Display Initialization
    lcd.init();
    lcd.backlight();
    lcd.setCursor(5, 1);
    lcd.print("FISHPOND");
    lcd.setCursor(1, 2);
    lcd.print("MONITORING SYSTEM");
    delay(2000);

    pinMode(ALARM_PIN, OUTPUT);

    pinMode(ALARM_BUTTON, INPUT_PULLUP);
    pinMode(INC_BUTTON, INPUT_PULLUP);
    pinMode(DEC_BUTTON, INPUT_PULLUP);

    pinMode(AIR_VALVE, OUTPUT);
    pinMode(AERATION_PUMP, OUTPUT);
    pinMode(WATER_PUMP, OUTPUT);
    pinMode(WATER_VALVE, OUTPUT);

    pinMode(WATER_SENSOR_UPPER, INPUT);
    pinMode(WATER_SENSOR_LOWER, INPUT);
    pinMode(AMMONIA_SENSOR, INPUT);
}


// ----- LOOP
void loop() {
    // Buttons check
    Serial.println("=== BUTTONS CHECK ===");
    if (digitalRead(ALARM_BUTTON) == 0) {
        Serial.println("ALARM BUTTON PRESSED");
    }
    if (digitalRead(INC_BUTTON) == 0) {
        Serial.println("INC BUTTON PRESSED");
    }
    if (digitalRead(DEC_BUTTON) == 0) {
        Serial.println("DEC BUTTON PRESSED");
    }

    // Sensors check
    Serial.println("=== SENSORS CHECK ===");

    Serial.print("AMMONIA SENSOR: ");
    ammonia_sensor = analogRead(AMMONIA_SENSOR);
    ammonia_sensor = mapf(ammonia_sensor, 0, 4095, 0, 5); // map from 0-5 ppm
    Serial.println(ammonia_sensor);
    
    Serial.print("WATER SENSOR UPPER: ");
    water_sensor_upper = analogRead(WATER_SENSOR_UPPER);
    water_sensor_upper = mapf(water_sensor_upper, 0, 4095, 10, 15); // map from 10-15 cm
    Serial.println(water_sensor_upper);

    Serial.print("WATER SENSOR LOWER: ");
    water_sensor_lower = analogRead(WATER_SENSOR_LOWER);
    water_sensor_lower = mapf(water_sensor_lower, 0, 4095, 0, 5); // map from 0-5 cm
    Serial.println(water_sensor_lower);

    // Chamber control check
    Serial.println("=== CHAMBER CONTROL CHECK ===");

    digitalWrite(AIR_VALVE, HIGH);
    delay(delay_time);
    digitalWrite(AIR_VALVE, LOW);
    
    digitalWrite(AERATION_PUMP, HIGH);
    delay(delay_time);
    digitalWrite(AERATION_PUMP, LOW);
    
    digitalWrite(WATER_PUMP, HIGH);
    delay(delay_time);
    digitalWrite(WATER_PUMP, LOW);
    
    digitalWrite(WATER_VALVE, HIGH);
    delay(delay_time);
    digitalWrite(WATER_VALVE, LOW);


    // Output interface check
    Serial.println("=== OUTPUT INTERFACE CHECK ===");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" TEST TEST TEST TEST");
    lcd.setCursor(0, 1);
    lcd.print("TEST TEST TEST TEST ");
    lcd.setCursor(0, 2);
    lcd.print(" TEST TEST TEST TEST");
    lcd.setCursor(0, 3);
    lcd.print("TEST TEST TEST TEST ");

    digitalWrite(ALARM_PIN, HIGH);
    delay(delay_time);
    digitalWrite(ALARM_PIN, LOW);

    Serial.println();
    Serial.println();
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}