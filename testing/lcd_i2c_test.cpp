#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>


LiquidCrystal_I2C lcd(0x27, 20, 4);  


void setup() {
  lcd.init();
  
  // turn on LCD backlight                      
  lcd.backlight();
  
  lcd.setCursor(0, 0);
  lcd.print("Hello, World!");
}

void loop() {
  // put your main code here, to run repeatedly:
}