#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


BluetoothSerial SerialBT;

unsigned long init_time;
unsigned long send_interval = 3000; // ms

// Data to be sent
String bt_send;
// current_measurement (4 char) / threshold (3 char) / status (1 char)
// status
// 1. NORMAL
// 2. ALARM
// 3. PASSIVE
// ex: "1.12 0.05 2"

float current_measurement = 1.12; // ppm
float threshold = 0.05; // ppm
enum states {NORMAL, ALARM, PASSIVE, INC_THRES_NORMAL, DEC_THRES_NORMAL, INC_THRES_PASSIVE, DEC_THRES_PASSIVE};
states state = NORMAL;

// Data to be received
int bt_receive;

int alarm_button = 0;
int inc_button = 0;
int dec_button = 0;

void prepare_data_send();
void bt_send_string(String string);
void bt_button_handler();

void test_changing_data();


void setup() {
    Serial.begin(115200);
    SerialBT.begin("ESP32_TA222302003"); // Bluetooth device name
    Serial.println("Success initialization. Pair device with bluetooth.");
    
    init_time = millis();
}


void loop() {

    // SEND TO ANDROID
    if (millis() - init_time > send_interval) { // Send data every few seconds
        prepare_data_send();
        
        Serial.print("SENDING DATA: ");
        Serial.println(bt_send);
        
        bt_send_string(bt_send);
        
        init_time = millis();
        test_changing_data();
    }
    

    // RECEIVE FROM ANDROID
    if (SerialBT.available()) {
        bt_button_handler();
        
        Serial.print("RECEIVING DATA: ");
        Serial.println(bt_receive);
        
        if (alarm_button == 1) {
            Serial.println("ALARM BUTTON PRESSED");
            alarm_button = 0;
        }
        else if (inc_button == 1) {
            Serial.println("INC BUTTON PRESSED");
            inc_button = 0;
        }
        else if (dec_button == 1) {
            Serial.println("DEC BUTTON PRESSED");
            dec_button = 0;
        }

    }
    
}

void prepare_data_send() {
    // Prepare status data
    String temp_state;
    if (state == NORMAL) {
        temp_state = "1";
    }
    else if (state == ALARM) {
        temp_state = "2";
    }
    else if (state == PASSIVE) {
        temp_state = "3";
    }
    
    bt_send = String(current_measurement, 2) + " " + String(threshold, 2) + " " + temp_state;
}

void bt_send_string(String string) {
    int i = 0;
    int count_char = string.length();
    while (i <= count_char) {
        char one_char = string[i];
        SerialBT.write(one_char);
        i++;
    }
}

void bt_button_handler() {
    bt_receive = SerialBT.read();

    switch (bt_receive) {
        case 1:
            alarm_button = 1;
            break;
        case 2:
            inc_button = 1;
            break;
        case 3:
            dec_button = 1;
            break;
        default:
            break;
    }
}

void test_changing_data() {
    // Change current measurement
    if (current_measurement <= 2) {
        current_measurement += 0.01;
    }
    else {
        current_measurement = 0;
    }

    // Change threshold
    if (threshold <= 1) {
        threshold += 0.01;
    }
    else {
        threshold = 0;
    }

    // Change state
    if (state == NORMAL) {
        state = PASSIVE;
    }
    else if (state == PASSIVE) {
        state = ALARM;
    }
    else {
        state = NORMAL;
    }
}