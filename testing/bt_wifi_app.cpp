#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#include <WiFi.h>

// BLUETOOTH
BluetoothSerial SerialBT;

unsigned long init_time;
unsigned long send_interval = 10000; // ms

// Data to be sent
String bt_send;
const int send_length = 10;
char char_bt_send[send_length + 1];
// current_measurement (4 char) / threshold (4 char) / status (1 char) / wifi_status (1 char)
// status: (1) NORMAL (2) ALARM (3) PASSIVE
// wifi_status: (0) WL_IDLE_STATUS (1) WL_NO_SSID_AVAIL (2) WL_SCAN_COMPLETED (3) WL_CONNECTED (4) WL_CONNECT_FAILED (5) WL_CONNECTION_LOST (6) WL_DISCONNECTED
// ex: "1.12 0.05 2 0"

float current_measurement = 1.12; // ppm
float threshold = 0.05; // ppm
enum states {NORMAL, ALARM, PASSIVE, INC_THRES_NORMAL, DEC_THRES_NORMAL, INC_THRES_PASSIVE, DEC_THRES_PASSIVE};
states state = NORMAL;

// Data to be received
// int bt_receive;
String bt_receive;

int alarm_button = 0;
int inc_button = 0;
int dec_button = 0;

void prepare_data_send();
void bt_send_string(String string);
String bt_receive_string();
void bt_button_handler();

void test_changing_data();

// WIFI
String ssid;
String password;
wl_status_t wifi_status;

int disconnect_wifi_button = 0;

void extract_wifi_credentials();
void print_wifi_status();

void setup() {
    Serial.begin(115200);
    
    SerialBT.begin("ESP32_TA222302003"); // Bluetooth device name
    Serial.println("Success initialization. Pair device with bluetooth.");
    init_time = millis();

    WiFi.mode(WIFI_STA);
}


void loop() {

    
    // SEND TO ANDROID
    if (millis() - init_time > send_interval) { // Send data every few seconds
        prepare_data_send();
        
        Serial.print("SENDING DATA: ");
        Serial.println(char_bt_send);
        
        bt_send_string(char_bt_send);
        
        init_time = millis();
        test_changing_data();
    }
    

    // RECEIVE FROM ANDROID
    if (SerialBT.available()) {
        
        bt_receive = bt_receive_string();
        Serial.print("RECEIVING STRING: ");
        Serial.println(bt_receive);

        if (bt_receive.length() == 1) {
            bt_button_handler();
        }
        else { // connect to wifi
            extract_wifi_credentials();
            WiFi.begin(ssid.c_str(), password.c_str());
        }
                
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
        else if (disconnect_wifi_button == 1) {
            if (WiFi.status() == WL_CONNECTED) {
                WiFi.disconnect();
            }
            else {
                Serial.println("WIFI IS NOT CONNECTED YET");
            }
            disconnect_wifi_button = 0;
        }
    }

    // Detect wifi status change
    if (WiFi.status() != wifi_status) {
        wifi_status = WiFi.status();
        print_wifi_status();
        if (wifi_status == WL_CONNECTED) {
            Serial.println(WiFi.localIP());
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

    String temp_wifi_status;
    switch (wifi_status) {
        case WL_IDLE_STATUS:
            temp_wifi_status = "0";
            break;
        case WL_NO_SSID_AVAIL:
            temp_wifi_status = "1";
            break;
        case WL_CONNECTED:
            temp_wifi_status = "2";
            break;
        case WL_CONNECT_FAILED:
            temp_wifi_status = "3";
            break;
        case WL_CONNECTION_LOST:
            temp_wifi_status = "4";
            break;
        case WL_DISCONNECTED:
            temp_wifi_status = "5";
            break;
    }
    
    bt_send = String(current_measurement, 2) + String(threshold, 2) + temp_state + temp_wifi_status;
    bt_send.toCharArray(char_bt_send, (send_length + 1));
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

String bt_receive_string() {
    String received_string; 
    while (SerialBT.available()) {
        byte one_byte_char = SerialBT.read();
        char one_char = (char) one_byte_char;
        received_string += one_char;
    }
    if (received_string.length() > 0) {
        return received_string;
    }
}

void bt_button_handler() {
    // bt_receive = SerialBT.read();
    char button_choice = bt_receive[0];

    switch (button_choice) {
        case '0':
            disconnect_wifi_button = 1;
            break;
        case '1':
            alarm_button = 1;
            break;
        case '2':
            inc_button = 1;
            break;
        case '3':
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

void extract_wifi_credentials() {
    ssid = "";
    password = "";
    int i = 0;
    for (i = 0; i < bt_receive.length(); i++) {
        if (bt_receive[i] == '+') {
            break;
        }
        ssid += bt_receive[i];
    }
    for (i = i+1; i < bt_receive.length(); i++) {
        password += bt_receive[i];
    }
}

void print_wifi_status() {
    switch (wifi_status) {
    case WL_IDLE_STATUS:
        Serial.println("WiFi is connecting/idle.");
        break;
    case WL_NO_SSID_AVAIL:
        Serial.println("WiFi SSID not available.");
        break;
    case WL_CONNECTED:
        Serial.println("WiFi is connected.");
        break;
    case WL_CONNECT_FAILED:
        Serial.println("WiFi connect failed.");
        break;
    case WL_CONNECTION_LOST:
        Serial.println("WiFi connection lost.");
        break;
    case WL_DISCONNECTED:
        Serial.println("WiFi not connected.");
        break;
    }
}