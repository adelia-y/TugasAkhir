#include <Arduino.h>


// ----- DEFINE VARIABLES
// Interfaces
#define LCD_PIN 5
#define ALARM_PIN 6

// Buttons
#define ALARM_BUTTON 1
#define INC_BUTTON 2
#define DEC_BUTTON 3

// Chamber
#define AIR_VALVE 20
#define AERATION_PUMP 21
#define WATER_PUMP 22
#define WATER_VALVE 23
#define WATER_SENSOR_UPPER 24
#define WATER_SENSOR_LOWER 25
#define AMMONIA_SENSOR 26

// Other variables
#define DEBOUNCE_DELAY 50 // ms


// Main FSM states
enum states {NORMAL, ALARM, PASSIVE, INC_THRES_NORMAL, DEC_THRES_NORMAL, INC_THRES_PASSIVE, DEC_THRES_PASSIVE};
states main_state = NORMAL;

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

// Alarm button variables
button_states dec_button = NOT_PRESSED;
button_states dec_button_prev = NOT_PRESSED;
int dec_button_val = 0;
unsigned long dec_t = 0;
unsigned long dec_t0 = 0;

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


// ----- SETUP
void setup() {
    Serial.begin(115200);
    
    pinMode(LCD_PIN, OUTPUT);
    pinMode(ALARM_PIN, OUTPUT);

    pinMode(ALARM_BUTTON, INPUT);
    pinMode(INC_BUTTON, INPUT);
    pinMode(DEC_BUTTON, INPUT);

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

    // Check and cycle button fsm
    fsm_alarm_button();
    fsm_inc_button();
    fsm_dec_button();

    // Check and update main fsm state according to all button states
    determine_state();
    FSM();

    // Change other sub-fsm according to main fsm states (display and alarm)
    fsm_display();
    fsm_alarm();
    
    // Cycle chamber & measure fsm, independent of main fsm states
    fsm_chamber();
    fsm_measure();

}


// ----- MAIN FSM FUNCTION
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
            
            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = NORMAL;

            init_display_time = millis();

            break;
        
        case DEC_THRES_NORMAL:
            // STATE 5: DECREASE THRESHOLD TRIGGERED FROM NORMAL STATE
            threshold -= 0.1;

            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = NORMAL;

            init_display_time = millis();

            break;

        case INC_THRES_PASSIVE:
            // STATE 6: INCREASE THRESHOLD TRIGGERED FROM PASSIVE STATE
            threshold += 0.1;
            
            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = PASSIVE;

            init_display_time = millis();

            break;
        
        case DEC_THRES_PASSIVE:
            // STATE 7: DECREASE THRESHOLD TRIGGERED FROM PASSIVE STATE
            threshold -= 0.1;

            display_state = DISPLAY_THRESHOLD;
            alarm_state = ALARM_OFF;
            main_state = PASSIVE;

            init_display_time = millis();

            break;
    }
}


void determine_state() {

    if (alarm_button == PRESSED) {
        if ((main_state == NORMAL) || (main_state == ALARM)) {
            main_state = PASSIVE;
        }
        else if (main_state == PASSIVE) {
            main_state = NORMAL;
        }
    }

    if (inc_button == PRESSED) {
        if (main_state == NORMAL) {
            main_state = INC_THRES_NORMAL;
        }
        else if (main_state == PASSIVE) {
            main_state = INC_THRES_PASSIVE;
        }
    }

    if (dec_button == PRESSED) {
        if (main_state == NORMAL) {
            main_state = DEC_THRES_NORMAL;
        }
        else if (main_state == PASSIVE) {
            main_state = DEC_THRES_PASSIVE;
        }
    }

}


// ----- INPUT BUTTON FUNCTIONS
void fsm_alarm_button() {

    alarm_button_val = digitalRead(ALARM_BUTTON);
    alarm_button_prev = alarm_button;

    switch (alarm_button) {
        
        case NOT_PRESSED:
            if (alarm_button_val) { // When button value is nonzero
                alarm_button = BOUNCE;
                alarm_t0 = millis();
            }
            break;
        
        case BOUNCE:
            alarm_t = millis();
            if (alarm_t - alarm_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if (alarm_button_val) {
                    alarm_button = PRESSED;
                }
                else {
                    alarm_button = NOT_PRESSED;
                }                
            }
            break;
        
        case PRESSED:
            if (!alarm_button_val) { // When button value is zero
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
    inc_button_prev = inc_button;

    switch (inc_button) {
        
        case NOT_PRESSED:
            if (inc_button_val) { // When button value is nonzero
                inc_button = BOUNCE;
                inc_t0 = millis();
            }
            break;
        
        case BOUNCE:
            inc_t = millis();
            if (inc_t - inc_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if (inc_button_val) {
                    inc_button = PRESSED;
                }
                else {
                    inc_button = NOT_PRESSED;
                }                
            }
            break;
        
        case PRESSED:
            if (!inc_button_val) { // When button value is zero
                inc_button = BOUNCE;
                inc_t0 = alarm_t;
                inc_t = millis();
            }
            break;
    
    }
}


void fsm_dec_button() {

    dec_button_val = digitalRead(DEC_BUTTON);
    dec_button_prev = dec_button;

    switch (dec_button) {
        
        case NOT_PRESSED:
            if (dec_button_val) { // When button value is nonzero
                dec_button = BOUNCE;
                dec_t0 = millis();
            }
            break;
        
        case BOUNCE:
            dec_t = millis();
            if (dec_t - dec_t0 >= DEBOUNCE_DELAY) { // debounce delay has passed
                if (dec_button_val) {
                    dec_button = PRESSED;
                }
                else {
                    dec_button = NOT_PRESSED;
                }                
            }
            break;
        
        case PRESSED:
            if (!dec_button_val) { // When button value is zero
                dec_button = BOUNCE;
                dec_t0 = dec_t;
                dec_t = millis();
            }
            break;
    
    }
}


bool is_above_threshold() {
    if (current_measurement >= threshold) {
        return true;
    }
    else { // input < threshold
        return false;
    }
}


// ----- CHAMBER FSM FUNCTIONS
void fsm_chamber() {
    switch (chamber_state) {
        case IDLE:
            // Do nothing
            break;
        
        case FILL:
            chamber_fill();
            break;
        
        case AERATION:
            chamber_aeration();
            break;
        
        case MEASUREMENT:
            chamber_measure();
            break;
        
        case DRAIN:
            chamber_drain();
            break;
    }
}

void chamber_fill() {
    // Chamber filling procedure
    water_height = digitalRead(WATER_SENSOR_UPPER) + water_sensor_position;
    if (water_height >= water_limit_upper) {
        chamber_state = AERATION;
        init_chamber_time = millis();
    }
    
    digitalWrite(AIR_VALVE, HIGH); // Air valve closed
    digitalWrite(AERATION_PUMP, LOW); // Aeration pump turned off
    digitalWrite(WATER_PUMP, HIGH); // Water pump on
    digitalWrite(WATER_VALVE, HIGH); // Water valve closed
}

void chamber_aeration() {
    // Chamber aeration procedure
    chamber_time = millis();
    if (chamber_time - init_chamber_time > aeration_time) {
        chamber_state = MEASUREMENT;
        init_chamber_time = millis();
    }
    
    digitalWrite(AIR_VALVE, HIGH); // Air valve closed
    digitalWrite(AERATION_PUMP, HIGH); // Aeration pump turned on
    digitalWrite(WATER_PUMP, LOW); // Water pump off
    digitalWrite(WATER_VALVE, HIGH); // Water valve closed
}

void chamber_measure() {
    // Chamber measurement procedure
    chamber_time = millis();
    if (chamber_time - init_chamber_time > measure_time) {
        chamber_state = DRAIN;
    }
    
    digitalWrite(AIR_VALVE, HIGH); // Air valve closed
    digitalWrite(AERATION_PUMP, LOW); // Aeration pump turned off
    digitalWrite(WATER_PUMP, LOW); // Water pump off
    digitalWrite(WATER_VALVE, HIGH); // Water valve closed

    measure_state = DETECT;
}

void chamber_drain() {
    // Chamber draining procedure
    water_height = digitalRead(WATER_SENSOR_LOWER);
    if (water_height <= water_limit_lower) {
        chamber_state = FILL;
    }
    
    digitalWrite(AIR_VALVE, LOW); // Air valve open, release chamber air
    digitalWrite(AERATION_PUMP, LOW); // Aeration pump turned off
    digitalWrite(WATER_PUMP, LOW); // Water pump off
    digitalWrite(WATER_VALVE, LOW); // Water valve open, draining

    measure_state = CALIBRATE;
}


// ----- CHAMBER MEASUREMENT FUNCTIONS
void fsm_measure() {
    switch (measure_state) {
        case AVAILABLE:
            current_measurement = calibrated_measurement;
            sum_measurement = 0;
            measure_count = 0;
            break;
        
        case DETECT:
            read_sensor();
            break;
        
        case CALIBRATE:
            calibrate_reading();
            break;        
    }
}

float read_sensor() {
    // Accumulate reading for measurement averaging
    sum_measurement += digitalRead(AMMONIA_SENSOR);
    measure_count += 1;
}

float calibrate_reading() {
    calibrated_measurement = sum_measurement / measure_count;
    // TO DO: Calibration using regression
}


// ----- DISPLAY AND ALARM FUNCTIONS
void fsm_display() {
    switch (display_state) {
        case DISPLAY_IDLE:
            // Display nothing while idle
            // TO DO: Display something while idle?
            break;
        
        case DISPLAY_MEASUREMENT:
            lcd_display_measurement();
            app_display_measurement();
            break;
        
        case DISPLAY_THRESHOLD:
            lcd_display_threshold();
            app_display_threshold();
            break;
    }
}


void fsm_alarm() {
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


// ----- DISPLAY OUTPUT CONTROL
void lcd_display_measurement() {
    Serial.print(" [LCD Display] Ammonia: ");
    Serial.println(current_measurement);
}

void app_display_measurement() {
    Serial.print(" [APP Display] Ammonia: ");
    Serial.println(current_measurement);
}

void lcd_display_threshold() {
    Serial.print(" [LCD Display] Threshold: ");
    Serial.println(current_measurement);
}

void app_display_threshold() {
    Serial.print(" [LCD Display] Threshold: ");
    Serial.println(current_measurement);
}

// ----- DISPLAY ALARM CONTROL
void alarm_off() {
    digitalWrite(ALARM_PIN, LOW);
}

void app_sound_off() {
    Serial.println("[APP Sound] OFF.");
}

void alarm_on() {
    digitalWrite(ALARM_PIN, HIGH);
}

void app_sound_on() {
    Serial.println("[APP Sound] ON.");
}