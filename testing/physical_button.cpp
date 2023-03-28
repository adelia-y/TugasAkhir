#include <Arduino.h>


// ----- DEFINE VARIABLES
// Buttons
#define ALARM_BUTTON 18
#define INC_BUTTON 19
#define DEC_BUTTON 21

// Other variables
#define DEBOUNCE_DELAY 50 // ms

// Button states for debouncing
enum button_states {NOT_PRESSED, BOUNCE, PRESSED};

// Alarm button variables
button_states alarm_button = NOT_PRESSED;
button_states alarm_button_prev = NOT_PRESSED;
int alarm_button_val = 0;
unsigned long alarm_t = 0;
unsigned long alarm_t0 = 0;

// Increase button variables
button_states inc_button = NOT_PRESSED;
button_states inc_button_prev = NOT_PRESSED;
int inc_button_val = 0;
unsigned long inc_t = 0;
unsigned long inc_t0 = 0;

// Decrease button variables
button_states dec_button = NOT_PRESSED;
button_states dec_button_prev = NOT_PRESSED;
int dec_button_val = 0;
unsigned long dec_t = 0;
unsigned long dec_t0 = 0;

// Function declarations
void simple_read_button();
void print_button_states();
void check_button_pressed();
void fsm_alarm_button();
void fsm_inc_button();
void fsm_dec_button();


// ----- SETUP
void setup() {
    Serial.begin(115200);

    pinMode(ALARM_BUTTON, INPUT_PULLUP);
    pinMode(INC_BUTTON, INPUT_PULLUP);
    pinMode(DEC_BUTTON, INPUT_PULLUP);
}


// ----- LOOP
void loop() {

    // Check and cycle button fsm
    fsm_alarm_button();
    fsm_inc_button();
    fsm_dec_button();

    check_button_pressed();
    // print_button_states();
    
    // delay(10);
}

void simple_read_button() {

    alarm_button_val = digitalRead(ALARM_BUTTON);
    inc_button_val = digitalRead(INC_BUTTON);
    dec_button_val = digitalRead(DEC_BUTTON);
    return;
}

void print_button_states() {

    Serial.print("ALARM BUTTON STATE: ");
    Serial.println(alarm_button);
    Serial.print("INCREASE BUTTON STATE: ");
    Serial.println(inc_button);
    Serial.print("DECREASE BUTTON STATE: ");
    Serial.println(dec_button);
    return;
}

void check_button_pressed() {

    if (alarm_button == PRESSED) {
        Serial.println("ALARM BUTTON PRESSED");
    }
    if (inc_button == PRESSED) {
        Serial.println("INCREASE BUTTON PRESSED");
    }
    if (dec_button == PRESSED) {
        Serial.println("DECREASE BUTTON PRESSED");
    }
    return;
}

void fsm_alarm_button() {

    alarm_button_val = digitalRead(ALARM_BUTTON);

    switch (alarm_button) {
        
        case NOT_PRESSED:
            if (alarm_button_val == 0) { // When button is pressed (low-active)
                alarm_button = BOUNCE;
                alarm_t0 = millis();
            }
            break;
        
        case BOUNCE:
            alarm_t = millis();
            if (alarm_t - alarm_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if (alarm_button_val == 0) {
                    alarm_button = PRESSED;
                }
                else {
                    alarm_button = NOT_PRESSED;
                }                
            }
            break;
        
        case PRESSED:
            if (alarm_button_val == 1) { // When button is not pressed
                alarm_button = BOUNCE;
                alarm_t0 = alarm_t;
                alarm_t = millis();
            }
            break;
    
    }
}
// REF: https://fastbitlab.com/fsm-lecture-30-exercise-003-button-software-debouncing-implementation/


void fsm_inc_button() {

    inc_button_val = digitalRead(INC_BUTTON);

    switch (inc_button) {
        
        case NOT_PRESSED:
            if (inc_button_val == 0) { // When button is pressed
                inc_button = BOUNCE;
                inc_t0 = millis();
            }
            break;
        
        case BOUNCE:
            inc_t = millis();
            if (inc_t - inc_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if (inc_button_val == 0) {
                    inc_button = PRESSED;
                }
                else {
                    inc_button = NOT_PRESSED;
                }                
            }
            break;
        
        case PRESSED:
            if (inc_button_val == 1) { // When button is not pressed
                inc_button = BOUNCE;
                inc_t0 = alarm_t;
                inc_t = millis();
            }
            break;
    
    }
}


void fsm_dec_button() {

    dec_button_val = digitalRead(DEC_BUTTON);

    switch (dec_button) {
        
        case NOT_PRESSED:
            if (dec_button_val == 0) { // When button is pressed
                dec_button = BOUNCE;
                dec_t0 = millis();
            }
            break;
        
        case BOUNCE:
            dec_t = millis();
            if (dec_t - dec_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if (dec_button_val == 0) {
                    dec_button = PRESSED;
                }
                else {
                    dec_button = NOT_PRESSED;
                }                
            }
            break;
        
        case PRESSED:
            if (dec_button_val == 1) { // When button is not pressed
                dec_button = BOUNCE;
                dec_t0 = dec_t;
                dec_t = millis();
            }
            break;
    
    }
}
