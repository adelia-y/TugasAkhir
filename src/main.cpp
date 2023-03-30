#include <Arduino.h>


// ----- DEFINE VARIABLES
// Buttons
#define ALARM_BUTTON 18
#define INC_BUTTON 19
#define DEC_BUTTON 21
#define TRIG_BUTTON 5

// Other variables
#define DEBOUNCE_DELAY 50 // ms

// Main FSM states
enum states {NORMAL, ALARM, PASSIVE, INC_THRES_NORMAL, DEC_THRES_NORMAL, INC_THRES_PASSIVE, DEC_THRES_PASSIVE};
states main_state = NORMAL;

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

// Trigger button variables
button_states trig_button = NOT_PRESSED;
button_states trig_button_prev = NOT_PRESSED;
int trig_button_val = 0;
unsigned long trig_t = 0;
unsigned long trig_t0 = 0;

// Display states
enum display_states {DISPLAY_IDLE, DISPLAY_MEASUREMENT, DISPLAY_THRESHOLD};
display_states display_state = DISPLAY_IDLE;

// Display variables
unsigned long init_display_time = 0; // ms
unsigned long display_time = 0; // ms
unsigned long display_delay = 2000; // ms

// Alarm states
enum alarm_states {ALARM_ON, ALARM_OFF};
alarm_states alarm_state = ALARM_OFF;

// Chamber states
enum chamber_states {IDLE, FILL, AERATION, MEASUREMENT, DRAIN};
chamber_states chamber_state = IDLE;

// Chamber variables
unsigned long aeration_time = 5000; // ms
unsigned long measure_time = 5000; // ms
unsigned long init_chamber_time = 0; // ms
unsigned long chamber_time = 0; // ms, contain time elapsed to control aeration and measure duration
int water_height = 0; // cm
int water_limit_upper = 10; // cm
int water_limit_lower = 1; // cm
int water_sensor_position = 8; // cm, this is the height of the upper sensor placed in chamber

// Measure states
enum measure_states {AVAILABLE, DETECT, CALIBRATE};
measure_states measure_state = AVAILABLE;

// Ammonia measurement variables
float threshold = 0.5; // measured in mg/L or ppm
float raw_measurement = 0;
float sum_measurement = 0;
float calibrated_measurement = 0;
float current_measurement = 0;
int measure_count = 0;
// unsigned long measurement_interval = 1000; // ms

// Test variables
states test_prev_main_state = NORMAL;

// ----- DECLARE FUNCTIONS
// Button FSM functions
void fsm_alarm_button();
void fsm_inc_button();
void fsm_dec_button();

// Main FSM functions
void determine_state();
void FSM();

// Measurement functions
bool is_above_threshold();

// Test function
void fsm_trigger_button();
void print_main_state(states main_state);

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
    test_prev_main_state = main_state;

    // Check and cycle button fsm
    fsm_alarm_button();
    fsm_inc_button();
    fsm_dec_button();
    fsm_trigger_button();

    determine_state();
    FSM();
    
    // Enable manual alarm triggering for testing 
    if (trig_button == PRESSED) {
        if (main_state != PASSIVE) {
            main_state = ALARM;
        }
    }

    if (test_prev_main_state != main_state) {
        print_main_state(main_state);
    }
    
    // delay(200);
}

// ------ TESTING FUNCTIONS
void print_main_state(states state) {
    switch (state) {
        case NORMAL:
            Serial.println("NORMAL");
            break;
        case ALARM:
            Serial.println("ALARM");
            break;
        case PASSIVE:
            Serial.println("PASSIVE");
            break;
        case INC_THRES_NORMAL:
            Serial.println("INC_THRES_NORMAL");
            break;
        case DEC_THRES_NORMAL:
            Serial.println("DEC_THRES_NORMAL");
            break;
        case INC_THRES_PASSIVE:
            Serial.println("INC_THRES_PASSIVE");
            break;
        case DEC_THRES_PASSIVE:
            Serial.println("DEC_THRES_PASSIVE");
            break;
    }
}

// ------ MAIN FSM
void determine_state() {

    if (alarm_button == FALLING_EDGE) {
        if ((main_state == NORMAL) || (main_state == ALARM)) {
            main_state = PASSIVE;
        }
        else if (main_state == PASSIVE) {
            main_state = NORMAL;
        }
    }

    if (inc_button == FALLING_EDGE) {
        if (main_state == NORMAL) {
            main_state = INC_THRES_NORMAL;
        }
        else if (main_state == PASSIVE) {
            main_state = INC_THRES_PASSIVE;
        }
    }

    if (dec_button == FALLING_EDGE) {
        if (main_state == NORMAL) {
            main_state = DEC_THRES_NORMAL;
        }
        else if (main_state == PASSIVE) {
            main_state = DEC_THRES_PASSIVE;
        }
    }

}

void FSM() {
    
    switch (main_state) {
        case NORMAL:
            // STATE 1: NORMAL MODE - READ, ALARM ENABLED
            // If previous state is inc/dec threshold, wait for some time before returning to display measurement
            if (display_state == DISPLAY_THRESHOLD) { 
                display_time = millis();
                if (init_display_time - display_time > display_delay) {
                    display_state = DISPLAY_MEASUREMENT;
                }
            }

            alarm_state = ALARM_OFF;
            
            if (is_above_threshold()) {
                main_state = ALARM;
            }
            
            break;

        case ALARM:
            // STATE 2: ALARM MODE - READ, ALARM ON
            if (display_state == DISPLAY_THRESHOLD) {
                display_time = millis();
                if (init_display_time - display_time > display_delay) {
                    display_state = DISPLAY_MEASUREMENT;
                }
            }
            
            alarm_state = ALARM_ON;
            
            if (!is_above_threshold()) {
                main_state = NORMAL;
            }
            
            break;
        
        case PASSIVE:
            // STATE 3: PASSIVE MODE - READ, ALARM DISABLED
            if (display_state == DISPLAY_THRESHOLD) {
                display_time = millis();
                if (init_display_time - display_time > display_delay) {
                    display_state = DISPLAY_MEASUREMENT;
                }
            }

            alarm_state = ALARM_OFF;
            break;

        case INC_THRES_NORMAL:
            // STATE 4: INCREASE THRESHOLD TRIGGERED FROM NORMAL STATE
            threshold += 0.1;
            Serial.println(threshold);
            
            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = NORMAL;

            init_display_time = millis();

            break;
        
        case DEC_THRES_NORMAL:
            // STATE 5: DECREASE THRESHOLD TRIGGERED FROM NORMAL STATE
            threshold -= 0.1;
            Serial.println(threshold);

            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = NORMAL;

            init_display_time = millis();

            break;

        case INC_THRES_PASSIVE:
            // STATE 6: INCREASE THRESHOLD TRIGGERED FROM PASSIVE STATE
            threshold += 0.1;
            Serial.println(threshold);
            
            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = PASSIVE;

            init_display_time = millis();

            break;
        
        case DEC_THRES_PASSIVE:
            // STATE 7: DECREASE THRESHOLD TRIGGERED FROM PASSIVE STATE
            threshold -= 0.1;
            Serial.println(threshold);

            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = PASSIVE;

            init_display_time = millis();

            break;
    }
}

// ------ MEASUREMENT FUNCTIONS
bool is_above_threshold() {
    if (current_measurement >= threshold) {
        return true;
    }
    else { // input < threshold
        return false;
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

void fsm_trigger_button() {

    trig_button_val = digitalRead(TRIG_BUTTON);

    switch (trig_button) {
        
        case NOT_PRESSED:
            if (trig_button_val == 0) { // When button is pressed
                trig_button = BOUNCE;
                trig_t0 = millis();
            }
            break;
        
        case BOUNCE:
            trig_t = millis();
            if (trig_t - trig_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if (trig_button_val == 0) {
                    trig_button = PRESSED;
                }
                else {
                    trig_button = NOT_PRESSED;
                }                
            }
            break;
        
        case PRESSED:
            if (trig_button_val == 1) { // When button is not pressed
                trig_button = BOUNCE;
                trig_t0 = trig_t;
                trig_t = millis();
            }
            break;
    
    }
}
