#include <Arduino.h>


// ----- DEFINE VARIABLES
// Buttons
#define ALARM_BUTTON 18
#define INC_BUTTON 19
#define DEC_BUTTON 21

// Other variables
#define DEBOUNCE_DELAY 50 // ms

// Button states for debouncing
enum button_states {NOT_PRESSED, BOUNCE, PRESSED, FALLING_EDGE, RISING_EDGE};

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

// Testing variables
button_states test_prev_alarm_state = NOT_PRESSED;
button_states test_prev_int_state = NOT_PRESSED;
button_states test_prev_dec_state = NOT_PRESSED;

// ----- DEFINE VARIABLES
// Button FSM functions
void fsm_alarm_button();
void fsm_inc_button();
void fsm_dec_button();

// Test functions
void simple_read_button();
void print_all_button_states();
void print_button_state(button_states button_state);

// ----- SETUP
void setup() {
    Serial.begin(115200);

    pinMode(ALARM_BUTTON, INPUT_PULLUP);
    pinMode(INC_BUTTON, INPUT_PULLUP);
    pinMode(DEC_BUTTON, INPUT_PULLUP);
}


// ----- LOOP
void loop() {

    // Store previous states for printing test results
    test_prev_alarm_state = alarm_button;
    test_prev_int_state = inc_button;
    test_prev_dec_state = dec_button;

    // Check and cycle button fsm
    fsm_alarm_button();
    fsm_inc_button();
    fsm_dec_button();

    print_all_button_states();
    
    // delay(10);
}

// ------ TESTING FUNCTIONS
void simple_read_button() {

    alarm_button_val = digitalRead(ALARM_BUTTON);
    inc_button_val = digitalRead(INC_BUTTON);
    dec_button_val = digitalRead(DEC_BUTTON);
    return;
}

void print_all_button_states() {

    // Only print state when there is a state change
    if (test_prev_alarm_state != alarm_button) {
        Serial.print("ALARM BUTTON STATE: ");
        print_button_state(alarm_button);
    }
    if (test_prev_int_state != inc_button) {
        Serial.print("INCREASE BUTTON STATE: ");
        print_button_state(inc_button);
    }
    if (test_prev_dec_state != dec_button) {
        Serial.print("DECREASE BUTTON STATE: ");
        print_button_state(dec_button);
    }

    return;
}

void print_button_state(button_states button_state) {
    switch (button_state) {
        case NOT_PRESSED:
            Serial.println("NOT_PRESSED");
            break;
        case BOUNCE:
            Serial.println("BOUNCE");
            break;
        case FALLING_EDGE:
            Serial.println("FALLING_EDGE");
            break;
        case PRESSED:
            Serial.println("PRESSED");
            break;
        case RISING_EDGE:
            Serial.println("RISING_EDGE");
            break;
    }
}

// ------ BUTTON FSM
void fsm_alarm_button() {

    alarm_button_val = digitalRead(ALARM_BUTTON);

    switch (alarm_button) {
        
        case NOT_PRESSED:
            if (alarm_button_val == 0) { // When button is pressed (low-active)
                alarm_button_prev = NOT_PRESSED;
                alarm_button = BOUNCE;
                alarm_t0 = millis();
            }
            break;
        
        case BOUNCE:
            alarm_t = millis();
            if (alarm_t - alarm_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if ((alarm_button_val == 0) && (alarm_button_prev == NOT_PRESSED)) {
                    alarm_button = FALLING_EDGE;
                }
                else if ((alarm_button_val == 1) && (alarm_button_prev == PRESSED)) {
                    alarm_button = RISING_EDGE;
                }
                else {
                    alarm_button = alarm_button_prev;
                }
            }
            break;
        
        case FALLING_EDGE:
            if (alarm_button_val == 0) { // button is held down
                alarm_button = PRESSED;
            }
            else { // button is released
                alarm_button_prev = PRESSED;
                alarm_button = BOUNCE;
                alarm_t0 = alarm_t;
                alarm_t = millis();
            }
            break;

        case PRESSED:
            if (alarm_button_val == 1) { // button is released
                alarm_button_prev = PRESSED;
                alarm_button = BOUNCE;
                alarm_t0 = alarm_t;
                alarm_t = millis();
            }
            break;
        
        case RISING_EDGE:
            if (alarm_button_val == 1) { // button is held down
                alarm_button = NOT_PRESSED;
            }
            else { // button is pressed again
                alarm_button_prev = NOT_PRESSED;
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
            if (inc_button_val == 0) { // When button is pressed (low-active)
                inc_button_prev = NOT_PRESSED;
                inc_button = BOUNCE;
                inc_t0 = millis();
            }
            break;
        
        case BOUNCE:
            inc_t = millis();
            if (inc_t - inc_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if ((inc_button_val == 0) && (inc_button_prev == NOT_PRESSED)) {
                    inc_button = FALLING_EDGE;
                }
                else if ((inc_button_val == 1) && (inc_button_prev == PRESSED)) {
                    inc_button = RISING_EDGE;
                }
                else {
                    inc_button = inc_button_prev;
                }
            }
            break;
        
        case FALLING_EDGE:
            if (inc_button_val == 0) { // button is held down
                inc_button = PRESSED;
            }
            else { // button is released
                inc_button_prev = PRESSED;
                inc_button = BOUNCE;
                inc_t0 = inc_t;
                inc_t = millis();
            }
            break;

        case PRESSED:
            if (inc_button_val == 1) { // button is released
                inc_button_prev = PRESSED;
                inc_button = BOUNCE;
                inc_t0 = inc_t;
                inc_t = millis();
            }
            break;
        
        case RISING_EDGE:
            if (inc_button_val == 1) { // button is held down
                inc_button = NOT_PRESSED;
            }
            else { // button is pressed again
                inc_button_prev = NOT_PRESSED;
                inc_button = BOUNCE;
                inc_t0 = inc_t;
                inc_t = millis();
            }
            break;
    }
}


void fsm_dec_button() {

    dec_button_val = digitalRead(DEC_BUTTON);

    switch (dec_button) {
        
        case NOT_PRESSED:
            if (dec_button_val == 0) { // When button is pressed (low-active)
                dec_button_prev = NOT_PRESSED;
                dec_button = BOUNCE;
                dec_t0 = millis();
            }
            break;
        
        case BOUNCE:
            dec_t = millis();
            if (dec_t - dec_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if ((dec_button_val == 0) && (dec_button_prev == NOT_PRESSED)) {
                    dec_button = FALLING_EDGE;
                }
                else if ((dec_button_val == 1) && (dec_button_prev == PRESSED)) {
                    dec_button = RISING_EDGE;
                }
                else {
                    dec_button = dec_button_prev;
                }
            }
            break;
        
        case FALLING_EDGE:
            if (dec_button_val == 0) { // button is held down
                dec_button = PRESSED;
            }
            else { // button is released
                dec_button_prev = PRESSED;
                dec_button = BOUNCE;
                dec_t0 = dec_t;
                dec_t = millis();
            }
            break;

        case PRESSED:
            if (dec_button_val == 1) { // button is released
                dec_button_prev = PRESSED;
                dec_button = BOUNCE;
                dec_t0 = dec_t;
                dec_t = millis();
            }
            break;
        
        case RISING_EDGE:
            if (dec_button_val == 1) { // button is held down
                dec_button = NOT_PRESSED;
            }
            else { // button is pressed again
                dec_button_prev = NOT_PRESSED;
                dec_button = BOUNCE;
                dec_t0 = dec_t;
                dec_t = millis();
            }
            break;
    }
}
