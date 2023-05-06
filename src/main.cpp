#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
#include <Wire.h>


// ------ DEFINE VARIABLES --------------------------------------------------------------
// ------ ALARM & LCD
#define ALARM_PIN 23 // DIGITAL OUTPUT PIN
LiquidCrystal_I2C lcd(0x27, 20, 4); // // LCD_PIN_SCL 22 - LCD_PIN_SDA 21

// ------ CHAMBER
#define AIR_VALVE 5 // DIGITAL OUTPUT PIN WHITE
#define AERATION_PUMP 19 // RED
#define WATER_PUMP 32 // YELLOW
#define WATER_VALVE 33 // GREEN

#define WATER_SENSOR_UPPER 36 // ADC DAC0
#define WATER_SENSOR_LOWER 39 // ADC DAC1
#define AMMONIA_SENSOR 34 // ADC

// ------ BUTTONS
#define ALARM_BUTTON 16 
#define INC_BUTTON 17
#define DEC_BUTTON 18

// ------ OTHER VARIABLES
#define DEBOUNCE_DELAY 50 // ms


// ------ DECLARE VARIABLES -------------------------------------------------------------
// ------ MAIN FSM
enum states {NORMAL, ALARM, PASSIVE, INC_THRES_NORMAL, DEC_THRES_NORMAL, INC_THRES_PASSIVE, DEC_THRES_PASSIVE};
states main_state = NORMAL;
states main_state_prev = NORMAL;

// ------ BUTTON FSM
enum button_states {NOT_PRESSED, BOUNCE, PRESSED, FALLING_EDGE, RISING_EDGE};

// Alarm button variables
button_states alarm_button = NOT_PRESSED;
button_states alarm_button_prev = NOT_PRESSED;
button_states alarm_button_prev_debug = NOT_PRESSED;
int alarm_button_val = 0;
unsigned long alarm_t = 0;
unsigned long alarm_t0 = 0;

// Increase button variables
button_states inc_button = NOT_PRESSED;
button_states inc_button_prev = NOT_PRESSED;
button_states inc_button_prev_debug = NOT_PRESSED;
int inc_button_val = 0;
unsigned long inc_t = 0;
unsigned long inc_t0 = 0;

// Decrease button variables
button_states dec_button = NOT_PRESSED;
button_states dec_button_prev = NOT_PRESSED;
button_states dec_button_prev_debug = NOT_PRESSED;
int dec_button_val = 0;
unsigned long dec_t = 0;
unsigned long dec_t0 = 0;

// ------ DISPLAY FSM
enum display_states {DISPLAY_IDLE, DISPLAY_UPDATE};
display_states display_state = DISPLAY_IDLE; 
display_states display_state_prev = DISPLAY_IDLE; 
String message = "";
// unsigned long init_display_time = 0; // ms
// unsigned long display_time = 0; // ms
// unsigned long display_delay = 2000; // ms

// ------ ALARM FSM
enum alarm_states {ALARM_ON, ALARM_OFF};
alarm_states alarm_state = ALARM_OFF;
alarm_states alarm_state_prev = ALARM_OFF;

// ------ CHAMBER FSM
enum chamber_states {FILL, AERATION, MEASUREMENT, DRAIN};
chamber_states chamber_state = FILL;
chamber_states chamber_state_prev = FILL;

unsigned long aeration_time = 5000; // ms
unsigned long measure_time = 5000; // ms
unsigned long init_chamber_time = 0; // ms
unsigned long chamber_time = 0; // ms, contain time elapsed to control aeration and measure duration
unsigned long duration = 0; // ms, contain duration of aeration and measurement for testing

float water_height = 0; // cm
float water_limit_upper = 10; // cm
float water_limit_lower = 1; // cm
float upper_water_sensor_position = 8; // cm, this is the height of the upper sensor placed in chamber
float lower_water_sensor_position = 0; // cm, this is the height of the lower sensor placed in chamber
float water_sensor_length = 5; // cm, this is the max length of water sensor detection

// ------ MEASURE FSM
enum measure_states {AVAILABLE, DETECT, CALIBRATE};
measure_states measure_state = AVAILABLE;
measure_states measure_state_prev = AVAILABLE;

float threshold = 0.5; // measured in mg/L or ppm
bool threshold_change = false; // for display necessities

float min_ammonia_sensor = 0; // ppm 
float max_ammonia_sensor = 5; // ppm

float raw_measurement = 0;
float sum_measurement = 0;
float calibrated_measurement = 0;
float current_measurement = 0;
float prev_measurement = 0;
int measure_count = 0;
// unsigned long measurement_interval = 1000; // ms

// Test variables
// states test_prev_main_state = NORMAL;
// chamber_states test_prev_chamber_state = IDLE;
// measure_states test_prev_measure_state = AVAILABLE;


// ------ DECLARE FUNCTIONS -------------------------------------------------------------
// ------ MAIN FSM
void determine_state();
void FSM();

// ------ BUTTON FSM
void fsm_alarm_button();
void fsm_inc_button();
void fsm_dec_button();

// ------ DISPLAY FSM
void fsm_display();
void lcd_display_status();
void lcd_display_measurement();
void lcd_display_threshold();
void display_message();

// ------ ALARM FSM
void fsm_alarm();
void alarm_off();
void app_sound_off();
void alarm_on();
void app_sound_on();

// ------ CHAMBER FSM
void fsm_chamber();
void chamber_fill();
void chamber_aeration();
void chamber_measure();
void chamber_drain();

void read_upper_water_height();
void read_lower_water_height();

// ------ MEASURE FSM
bool is_above_threshold();
void fsm_measure();
void read_sensor();
void calibrate_reading();

// ------ HELPER FUNCTIONS
// Other functions
void lcd_clear_line(int line);
float mapf(float x, float in_min, float in_max, float out_min, float out_max);

// Debug functions
void test_main_fsm();
void test_alarm_button_state();
void test_inc_button_state();
void test_dec_button_state();
void test_display_state();
void test_alarm_state();
void test_chamber_state();
void test_measure_state();

// Getting states as string
String get_main_state(states state);
String get_button_state(button_states state);
String get_display_state(display_states state);
String get_alarm_state(alarm_states state);
String get_chamber_state(chamber_states state);
String get_measure_state(measure_states state);


// ------ SETUP -------------------------------------------------------------------------
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

    // Pins Initialization
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


// ------ LOOP --------------------------------------------------------------------------
void loop() {

    // Check and cycle button fsm
    fsm_alarm_button();
    fsm_inc_button();
    fsm_dec_button();

    determine_state();
    FSM();

    // Change other sub-fsm according to main fsm states (display and alarm)
    fsm_display();
    fsm_alarm();
    
    // Cycle chamber & measure fsm, independent of main fsm states
    fsm_chamber();
    fsm_measure();

    // Debug in serial monitor
    test_main_fsm();
    test_alarm_button_state();
    test_inc_button_state();
    test_dec_button_state();
    test_display_state();
    test_alarm_state();
    test_chamber_state();
    test_measure_state();
}


// ------ HELPER FUNCTIONS --------------------------------------------------------------
// Getting states as string
String get_main_state(states state) {
    switch (state) {
        case NORMAL:
            return "NORMAL";
            break;
        case ALARM:
            return "ALARM";
            break;
        case PASSIVE:
            return "PASSIVE";
            break;
        case INC_THRES_NORMAL:
            return "INC_THRES_NORMAL";
            break;
        case DEC_THRES_NORMAL:
            return "DEC_THRES_NORMAL";
            break;
        case INC_THRES_PASSIVE:
            return "INC_THRES_PASSIVE";
            break;
        case DEC_THRES_PASSIVE:
            return "DEC_THRES_PASSIVE";
            break;
    }
}
String get_button_state(button_states state) {
    switch (state) {
        case NOT_PRESSED:
            return "NOT_PRESSED";
            break;
        case BOUNCE:
            return "BOUNCE";
            break;
        case PRESSED:
            return "PRESSED";
            break;
        case FALLING_EDGE:
            return "FALLING_EDGE";
            break;
        case RISING_EDGE:
            return "RISING_EDGE";
            break;
    }    
}

String get_display_state(display_states state) {
    switch (state) {
        case DISPLAY_IDLE:
            return "DISPLAY_IDLE";
            break;
        case DISPLAY_UPDATE:
            return "DISPLAY_UPDATE";
            break;
    }
}

String get_alarm_state(alarm_states state) {
    switch (state) {
        case ALARM_ON:
            return "ALARM_ON";
            break;
        case ALARM_OFF:
            return "ALARM_OFF";
            break;
    }
}

String get_chamber_state(chamber_states state) {
    switch (state) {
        case IDLE:
            return "IDLE";
            break;
        case FILL:
            return "FILL";
            break;
        case AERATION:
            return "AERATION";
            break;
        case MEASUREMENT:
            return "MEASUREMENT";
            break;
        case DRAIN:
            return "DRAIN";
            break;
    }
}

String get_measure_state(measure_states state) {
    switch (state) {
        case AVAILABLE:
            return "AVAILABLE";
            break;
        case DETECT:
            return "DETECT";
            break;
        case CALIBRATE:
            return "CALIBRATE";
            break;
    }
}

// Helper functions
void lcd_clear_line(int line) {
    lcd.setCursor(0, line);
    for(int n = 0; n < 20; n++) {
        lcd.print(" ");
    }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Debug functions
void test_main_fsm() {
    if (main_state != main_state_prev) {
        Serial.print("Main FSM state change: ");
        Serial.print(get_main_state(main_state_prev) + " -> ");
        Serial.println(get_main_state(main_state));
    }
}

void test_alarm_button_state() {
    if (alarm_button != alarm_button_prev_debug) {
        Serial.print("Alarm button state change: ");
        Serial.println(get_button_state(alarm_button));
    }
    alarm_button_prev_debug = alarm_button;
}
void test_inc_button_state() {
    if (inc_button != inc_button_prev_debug) {
        Serial.print("Increase button state change: ");
        Serial.println(get_button_state(inc_button));
    }
    inc_button_prev_debug = inc_button;
}

void test_dec_button_state() {
    if (dec_button != dec_button_prev_debug) {
        Serial.print("Decrease button state change: ");
        Serial.println(get_button_state(dec_button));
    }
    dec_button_prev_debug = dec_button;
}

void test_display_state() {
    if (display_state != display_state_prev) {
        Serial.print("Display state change: ");
        Serial.print(get_display_state(display_state_prev) + " -> ");
        Serial.println(get_display_state(display_state));
    }
}

void test_alarm_state() {
    if (alarm_state != alarm_state_prev) {
        Serial.print("Alarm state change: ");
        Serial.print(get_alarm_state(alarm_state_prev) + " -> ");
        Serial.println(get_alarm_state(alarm_state));
    }
}

void test_chamber_state() {
    if (chamber_state != chamber_state_prev) {
        Serial.print("Chamber state change: ");
        Serial.print(get_chamber_state(chamber_state_prev) + " -> ");
        Serial.println(get_chamber_state(chamber_state));
        
        // Chamber cycle order: FILL -> AERATION -> MEASUREMENT -> DRAIN
        switch (chamber_state) {
            case FILL:
            case DRAIN:
                Serial.print("Water height: ");
                Serial.println(water_height);
                break;
            case AERATION:
            case MEASUREMENT:
                Serial.print("Duration: ");
                Serial.println(duration);
                break;
        }
    }
}

void test_measure_state() {
    if (measure_state != measure_state_prev) {
        Serial.print("Measure state change: ");
        Serial.print(get_measure_state(measure_state_prev) + " -> ");
        Serial.println(get_measure_state(measure_state));
        
        if (measure_state == AVAILABLE) {
            Serial.print("Current measurement: ");
            Serial.println(current_measurement);
            Serial.print("Measure count: ");
            Serial.println(measure_count);
        }
    }
    measure_state_prev = measure_state;
}

// ------ MAIN FSM ----------------------------------------------------------------------
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
            message = "MATIKAN ALARM";
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
            message = "MATIKAN ALARM";
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


// ------ BUTTON FSM --------------------------------------------------------------------
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


// ------ DISPLAY FSM -------------------------------------------------------------------
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
                display_state_prev = display_state;
                display_state = DISPLAY_UPDATE;
            }
            display_state_prev = display_state;
            break;
        
        case DISPLAY_UPDATE:
            lcd_display_status();
            lcd_display_measurement();
            lcd_display_threshold();

            // app_send_data();
            
            threshold_change = false;

            display_message();
            message = "";
            
            main_state_prev = main_state;
            prev_measurement = current_measurement;

            display_state_prev = display_state;
            display_state = DISPLAY_IDLE;
            break;
    }
}

void lcd_display_status() {
    Serial.print("[LCD Display] Status: ");
    Serial.println(get_main_state(main_state));

    lcd_clear_line(0);
    lcd.setCursor(0, 0);
    lcd.print("Status: ");
    lcd.print(get_main_state(main_state));
}

void lcd_display_measurement() {
    Serial.print("[LCD Display] Ammonia: ");
    Serial.print(current_measurement);
    Serial.println(" ppm");

    lcd_clear_line(1);
    lcd.setCursor(0, 1);
    lcd.print("Ammonia: ");
    lcd.print(current_measurement);
    lcd.print(" ppm");
}

void lcd_display_threshold() {
    Serial.print("[LCD Display] Threshold: ");
    Serial.print(threshold);
    Serial.println(" ppm");

    lcd_clear_line(2);
    lcd.setCursor(0, 2);
    lcd.print("Threshold: ");
    lcd.print(threshold);
    lcd.print(" ppm");
}

void display_message() {
    if (message != "") {
        Serial.print("[LCD Display] ");
        Serial.println(message);
        
        lcd_clear_line(3);
        lcd.setCursor(0, 3);
        lcd.print(message);
    }
    else {
        Serial.print("[LCD Display] ");
        Serial.println();

        lcd_clear_line(3);
    }
}

void app_display_status() {
    Serial.print("[APP Display] Status: ");
    Serial.println(get_main_state(main_state));
}

void app_display_measurement() {
    Serial.print("[APP Display] Ammonia: ");
    Serial.print(current_measurement);
    Serial.println(" ppm");
}

void app_display_threshold() {
    Serial.print("[APP Display] Threshold: ");
    Serial.print(threshold);
    Serial.println(" ppm");
}

// ------ ALARM FSM ---------------------------------------------------------------------
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

void alarm_off() {
    Serial.println("[ALARM Sound] OFF.");
    digitalWrite(ALARM_PIN, LOW);
}

void app_sound_off() {
    Serial.println("[APP Sound] OFF.");
}

void alarm_on() {
    Serial.println("[ALARM Sound] ON.");
    digitalWrite(ALARM_PIN, HIGH);
}

void app_sound_on() {
    Serial.println("[APP Sound] ON.");
}


// ------ CHAMBER FSM -------------------------------------------------------------------
void fsm_chamber() {
    switch (chamber_state) {

        case FILL:
            chamber_state_prev = chamber_state;
            read_upper_water_height();

            if (water_height >= water_limit_upper) {
                chamber_state = AERATION;
                init_chamber_time = millis();
            }

            chamber_fill();
            break;
        
        case AERATION:
            chamber_state_prev = chamber_state;
            chamber_time = millis();

            if (chamber_time - init_chamber_time > aeration_time) {
                duration = chamber_time - init_chamber_time;
                chamber_state = MEASUREMENT;
                init_chamber_time = millis();
            }

            chamber_aeration();
            break;
        
        case MEASUREMENT:
            chamber_state_prev = chamber_state;
            chamber_time = millis();

            if (chamber_time - init_chamber_time > measure_time) {
                duration = chamber_time - init_chamber_time;
                chamber_state = DRAIN;
            }
            
            chamber_measure();
            measure_state = DETECT;
            break;
        
        case DRAIN:
            chamber_state_prev = chamber_state;
            read_lower_water_height();

            if (water_height <= water_limit_lower) {
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


// ------ MEASUREMENT FUNCTIONS ---------------------------------------------------------
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
            measure_state = AVAILABLE;
            break;
    }
}

void read_sensor() {
    // Accumulate reading for measurement averaging
    raw_measurement = analogRead(AMMONIA_SENSOR);
    raw_measurement = mapf(
        raw_measurement, 0, 4095,
        min_ammonia_sensor, max_ammonia_sensor);
    
    sum_measurement += raw_measurement;
    measure_count += 1;
}

void calibrate_reading() {
    calibrated_measurement = sum_measurement / measure_count;
    // TO DO: Calibration using regression
}

void read_upper_water_height() {
    water_height = analogRead(WATER_SENSOR_UPPER);
    water_height = mapf(
        water_height, 0, 4095,
        upper_water_sensor_position, (upper_water_sensor_position + water_sensor_length));
}

void read_lower_water_height() {
    water_height = analogRead(WATER_SENSOR_LOWER);
    water_height = mapf(
        water_height, 0, 4095,
        lower_water_sensor_position, (lower_water_sensor_position + water_sensor_length));
}

bool is_above_threshold() {
    if (current_measurement >= threshold) {
        return true;
    }
    else { // input < threshold
        return false;
    }
}