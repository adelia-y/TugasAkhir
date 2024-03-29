#include <Arduino.h>


// ----- DEFINE VARIABLES
// Interfaces
#define LCD_PIN 5
#define ALARM_PIN 6

// Chamber
#define AIR_VALVE 20
#define AERATION_PUMP 21
#define WATER_PUMP 22
#define WATER_VALVE 23
#define WATER_SENSOR_UPPER 24
#define WATER_SENSOR_LOWER 25
#define AMMONIA_SENSOR 26

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
states main_state_prev = NORMAL;

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
enum display_states {DISPLAY_IDLE, DISPLAY_UPDATE};
display_states display_state = DISPLAY_IDLE;

// Display variables
String message = "";
// unsigned long init_display_time = 0; // ms
// unsigned long display_time = 0; // ms
// unsigned long display_delay = 2000; // ms

// Alarm states
enum alarm_states {ALARM_ON, ALARM_OFF};
alarm_states alarm_state = ALARM_OFF;
alarm_states alarm_state_prev = ALARM_OFF;

// Chamber states
enum chamber_states {IDLE, FILL, AERATION, MEASUREMENT, DRAIN};
chamber_states chamber_state = IDLE;
chamber_states chamber_state_prev = IDLE;

// Chamber variables
unsigned long aeration_time = 5000; // ms
unsigned long measure_time = 5000; // ms
unsigned long init_chamber_time = 0; // ms
unsigned long chamber_time = 0; // ms, contain time elapsed to control aeration and measure duration
float water_height = 0; // cm
float water_limit_upper = 10; // cm
float water_limit_lower = 1; // cm
float water_sensor_position = 8; // cm, this is the height of the upper sensor placed in chamber

// Measure states
enum measure_states {AVAILABLE, DETECT, CALIBRATE};
measure_states measure_state = AVAILABLE;

// Ammonia measurement variables
float threshold = 0.5; // measured in mg/L or ppm
bool threshold_change = false; // for display necessities

float raw_measurement = 0;
float sum_measurement = 0;
float calibrated_measurement = 0;
float current_measurement = 0;
float prev_measurement = 0;
int measure_count = 0;
// unsigned long measurement_interval = 1000; // ms

// Test variables
states test_prev_main_state = NORMAL;
chamber_states test_prev_chamber_state = IDLE;

// ----- DECLARE FUNCTIONS
// Button FSM functions
void fsm_alarm_button();
void fsm_inc_button();
void fsm_dec_button();
void fsm_trig_button();

// Main FSM functions
void determine_state();
void FSM();

// Measurement functions
bool is_above_threshold();

// Display FSM functions
void fsm_display();
void lcd_display_status();
void app_display_status();
void lcd_display_measurement();
void app_display_measurement();
void lcd_display_threshold();
void app_display_threshold();
void display_message();

// Alarm FSM functions
void fsm_alarm();
void alarm_off();
void app_sound_off();
void alarm_on();
void app_sound_on();

// Chamber FSM functions
void fsm_chamber();
void chamber_fill();
void chamber_aeration();
void chamber_measure();
void chamber_drain();

// Test function
void print_main_state();
void print_chamber_state();

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
    fsm_trig_button();

    determine_state();
    FSM();
    
    // Enable manual alarm triggering for testing 
    if (trig_button == FALLING_EDGE) {
        if (main_state != PASSIVE) {
            prev_measurement = current_measurement;
            current_measurement = threshold + 0.10;
        }
    }

    // Change other sub-fsm according to main fsm states (display and alarm)
    fsm_display();
    fsm_alarm();
    
    // Cycle chamber & measure fsm, independent of main fsm states
    fsm_chamber();
    
    if (test_prev_chamber_state != chamber_state) {
        print_chamber_state();
        test_prev_chamber_state = chamber_state;
    }
    
    // delay(200);
}

// ------ TESTING FUNCTIONS
void print_main_state() {
    switch (main_state) {
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

void print_chamber_state() {
    switch (chamber_state) {
        case IDLE:
            Serial.println("IDLE");
            break;
        case FILL:
            Serial.println("FILL");
            break;
        case AERATION:
            Serial.println("AERATION");
            break;
        case MEASUREMENT:
            Serial.println("MEASUREMENT");
            break;
        case DRAIN:
            Serial.println("DRAIN");
            break;
    }
}

// ------ MAIN FSM
void determine_state() {

    if (alarm_button == FALLING_EDGE) {
        if ((main_state == NORMAL) || (main_state == ALARM)) {
            main_state_prev = main_state;
            main_state = PASSIVE;
        }
        else if (main_state == PASSIVE) {
            main_state_prev = main_state;
            main_state = NORMAL;
        }
    }

    if (inc_button == FALLING_EDGE) {
        if (main_state == NORMAL) {
            main_state_prev = main_state;
            main_state = INC_THRES_NORMAL;
        }
        else if (main_state == PASSIVE) {
            main_state_prev = main_state;
            main_state = INC_THRES_PASSIVE;
        }
        else {
            message = "NORMAL/PASSIVE MODE ONLY";
        }
    }

    if (dec_button == FALLING_EDGE) {
        if (main_state == NORMAL) {
            main_state_prev = main_state;
            main_state = DEC_THRES_NORMAL;
        }
        else if (main_state == PASSIVE) {
            main_state_prev = main_state;
            main_state = DEC_THRES_PASSIVE;
        }
        else {
            message = "NORMAL/PASSIVE MODE ONLY";
        }
    }

}

void FSM() {
    
    switch (main_state) {
        case NORMAL:
            // STATE 1: NORMAL MODE - READ, ALARM ENABLED
            // display_state = DISPLAY_IDLE;
            
            alarm_state_prev = alarm_state;
            alarm_state = ALARM_OFF;
            
            if (is_above_threshold()) {
                main_state_prev = main_state;
                main_state = ALARM;
            }
            
            break;

        case ALARM:
            // STATE 2: ALARM MODE - READ, ALARM ON
            // display_state = DISPLAY_IDLE;

            alarm_state_prev = alarm_state;
            alarm_state = ALARM_ON;
            
            if (!is_above_threshold()) {
                main_state_prev = main_state;
                main_state = NORMAL;
            }
            
            break;
        
        case PASSIVE:
            // STATE 3: PASSIVE MODE - READ, ALARM DISABLED
            // display_state = DISPLAY_IDLE;

            alarm_state_prev = alarm_state;
            alarm_state = ALARM_OFF;
            break;

        case INC_THRES_NORMAL:
            // STATE 4: INCREASE THRESHOLD TRIGGERED FROM NORMAL STATE
            threshold += 0.1;
            threshold_change = true;
            
            // display_state = DISPLAY_IDLE;

            alarm_state_prev = alarm_state;
            alarm_state = ALARM_OFF;
            
            main_state_prev = main_state;
            main_state = NORMAL;
            break;
        
        case DEC_THRES_NORMAL:
            // STATE 5: DECREASE THRESHOLD TRIGGERED FROM NORMAL STATE
            threshold -= 0.1;

            // display_state = DISPLAY_IDLE;

            alarm_state_prev = alarm_state;
            alarm_state = ALARM_OFF;

            main_state_prev = main_state;
            main_state = NORMAL;
            break;

        case INC_THRES_PASSIVE:
            // STATE 6: INCREASE THRESHOLD TRIGGERED FROM PASSIVE STATE
            threshold += 0.1;

            // display_state = DISPLAY_IDLE;

            alarm_state_prev = alarm_state;
            alarm_state = ALARM_OFF;

            main_state_prev = main_state;
            main_state = PASSIVE;
            break;
        
        case DEC_THRES_PASSIVE:
            // STATE 7: DECREASE THRESHOLD TRIGGERED FROM PASSIVE STATE
            threshold -= 0.1;

            // display_state = DISPLAY_IDLE;

            alarm_state_prev = alarm_state;
            alarm_state = ALARM_OFF;

            main_state_prev = main_state;
            main_state = PASSIVE;
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

void fsm_trig_button() {

    trig_button_val = digitalRead(TRIG_BUTTON);

    switch (trig_button) {
        
        case NOT_PRESSED:
            if (trig_button_val == 0) { // When button is pressed (low-active)
                trig_button_prev = NOT_PRESSED;
                trig_button = BOUNCE;
                trig_t0 = millis();
            }
            break;
        
        case BOUNCE:
            trig_t = millis();
            if (trig_t - trig_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if ((trig_button_val == 0) && (trig_button_prev == NOT_PRESSED)) {
                    trig_button = FALLING_EDGE;
                }
                else if ((trig_button_val == 1) && (trig_button_prev == PRESSED)) {
                    trig_button = RISING_EDGE;
                }
                else {
                    trig_button = trig_button_prev;
                }
            }
            break;
        
        case FALLING_EDGE:
            if (trig_button_val == 0) { // button is held down
                trig_button = PRESSED;
            }
            else { // button is released
                trig_button_prev = PRESSED;
                trig_button = BOUNCE;
                trig_t0 = trig_t;
                trig_t = millis();
            }
            break;

        case PRESSED:
            if (trig_button_val == 1) { // button is released
                trig_button_prev = PRESSED;
                trig_button = BOUNCE;
                trig_t0 = trig_t;
                trig_t = millis();
            }
            break;
        
        case RISING_EDGE:
            if (trig_button_val == 1) { // button is held down
                trig_button = NOT_PRESSED;
            }
            else { // button is pressed again
                trig_button_prev = NOT_PRESSED;
                trig_button = BOUNCE;
                trig_t0 = trig_t;
                trig_t = millis();
            }
            break;
    }
}



// ----- DISPLAY AND ALARM FUNCTIONS
void fsm_display() {
    switch (display_state) {
        case DISPLAY_IDLE:
            // Dont change info on display while idle
            if (
                (main_state != main_state_prev) ||
                (current_measurement != prev_measurement) ||
                (threshold_change) ||
                (message != "")
            ) {
                display_state = DISPLAY_UPDATE;
            }
            break;
        
        case DISPLAY_UPDATE:
            // lcd_display_status();
            // app_display_status();
        
            // lcd_display_measurement();
            // app_display_measurement();

            // lcd_display_threshold();
            // app_display_threshold();
            threshold_change = false;

            // display_message();
            message = ""; // Empty extra message after displaying once
            
            main_state_prev = main_state;
            prev_measurement = current_measurement;

            display_state = DISPLAY_IDLE;
            break;
    }
}


void fsm_alarm() {
    if (alarm_state != alarm_state_prev) {
        switch (alarm_state) {
            case ALARM_OFF:
                alarm_off();
                app_sound_off();
                break;
            
            case ALARM_ON:
                alarm_on();
                app_sound_on();
                break;        
        }
    }
}


// ----- DISPLAY OUTPUT CONTROL
void lcd_display_status() {
    Serial.print("[LCD Display] Status: ");
    print_main_state();
}

void app_display_status() {
    Serial.print("[APP Display] Status: ");
    print_main_state();
}

void lcd_display_measurement() {
    Serial.print("[LCD Display] Ammonia: ");
    Serial.print(current_measurement);
    Serial.println(" ppm");
}

void app_display_measurement() {
    Serial.print("[APP Display] Ammonia: ");
    Serial.print(current_measurement);
    Serial.println(" ppm");
}

void lcd_display_threshold() {
    Serial.print("[LCD Display] Threshold: ");
    Serial.print(threshold);
    Serial.println(" ppm");
}

void app_display_threshold() {
    Serial.print("[LCD Display] Threshold: ");
    Serial.print(threshold);
    Serial.println(" ppm");
}

void display_message() {
    if (message != "") {
        Serial.println(message);
    }
    else {
        Serial.println();
    }
}

// ----- DISPLAY ALARM CONTROL
void alarm_off() {
    Serial.println("[ALARM Sound] OFF.");
    // digitalWrite(ALARM_PIN, LOW);
}

void app_sound_off() {
    Serial.println("[APP Sound] OFF.");
}

void alarm_on() {
    Serial.println("[ALARM Sound] ON.");
    // digitalWrite(ALARM_PIN, HIGH);
}

void app_sound_on() {
    Serial.println("[APP Sound] ON.");
}


// ----- CHAMBER FSM FUNCTIONS
void fsm_chamber() {
    switch (chamber_state) {
        case IDLE:
            // Do nothing
            test_prev_chamber_state = chamber_state;
            chamber_state = FILL;
            break;
        
        case FILL:
            // water_height = digitalRead(WATER_SENSOR_UPPER) + water_sensor_position;
            water_height += 0.001;

            if (water_height >= water_limit_upper) {
                Serial.print("Water height: ");
                Serial.println(water_height);
                
                test_prev_chamber_state = chamber_state;
                chamber_state = AERATION;
                init_chamber_time = millis();
            }

            chamber_fill();
            break;
        
        case AERATION:
            chamber_time = millis();

            if (chamber_time - init_chamber_time > aeration_time) {
                Serial.print("Chamber time: ");
                Serial.println(chamber_time - init_chamber_time);

                test_prev_chamber_state = chamber_state;
                chamber_state = MEASUREMENT;
                init_chamber_time = millis();
            }

            chamber_aeration();
            break;
        
        case MEASUREMENT:
            chamber_time = millis();

            if (chamber_time - init_chamber_time > measure_time) {
                Serial.print("Chamber time: ");
                Serial.println(chamber_time - init_chamber_time);
                
                test_prev_chamber_state = chamber_state;
                chamber_state = DRAIN;
            }
            
            chamber_measure();
            measure_state = DETECT;
            break;
        
        case DRAIN:
            // water_height = digitalRead(WATER_SENSOR_LOWER);
            water_height -= 0.001;

            if (water_height <= water_limit_lower) {
                Serial.print("Water height: ");
                Serial.println(water_height);
                test_prev_chamber_state = chamber_state;
                chamber_state = FILL;
            }
            
            chamber_drain();
            measure_state = CALIBRATE;
            break;
    }
}

void chamber_fill() {
    // Chamber filling procedure    
    digitalWrite(AIR_VALVE, HIGH); // Air valve closed
    digitalWrite(AERATION_PUMP, LOW); // Aeration pump turned off
    digitalWrite(WATER_PUMP, HIGH); // Water pump on
    digitalWrite(WATER_VALVE, HIGH); // Water valve closed
}

void chamber_aeration() {
    // Chamber aeration procedure    
    digitalWrite(AIR_VALVE, HIGH); // Air valve closed
    digitalWrite(AERATION_PUMP, HIGH); // Aeration pump turned on
    digitalWrite(WATER_PUMP, LOW); // Water pump off
    digitalWrite(WATER_VALVE, HIGH); // Water valve closed
}

void chamber_measure() {
    // Chamber measurement procedure
    digitalWrite(AIR_VALVE, HIGH); // Air valve closed
    digitalWrite(AERATION_PUMP, LOW); // Aeration pump turned off
    digitalWrite(WATER_PUMP, LOW); // Water pump off
    digitalWrite(WATER_VALVE, HIGH); // Water valve closed
}

void chamber_drain() {
    // Chamber draining procedure
    digitalWrite(AIR_VALVE, LOW); // Air valve open, release chamber air
    digitalWrite(AERATION_PUMP, LOW); // Aeration pump turned off
    digitalWrite(WATER_PUMP, LOW); // Water pump off
    digitalWrite(WATER_VALVE, LOW); // Water valve open, draining
}