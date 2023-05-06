#include "WiFi.h"
#include "ESPAsyncWebServer.h"

#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>


// ----------------- WIFI ACCESS POINT MODE
const char *ap_ssid = "ESP32TA003";
const char *ap_pass = "testpassword";

AsyncWebServer server(80);
IPAddress IP;
bool server_on = false;

const char* PARAM_INPUT_SSID = "inputssid";
const char* PARAM_INPUT_PASS = "inputpass";

// HTML web page to handle ssid and password input fields
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
    <title>ESP Input Form</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    </head><body>
    <form action="/get">
        SSID: <input type="text" name="inputssid"><br>
        Password: <input type="text" name="inputpass"><br>
        <input type="submit" value="Submit">
    </form>
    </body></html>)rawliteral";

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}


// ----------------- WIFI STATION MODE
String sta_ssid;
String sta_pass;
wl_status_t wifi_status;
WiFiClientSecure sta_client;

// ----------------- TELEGRAM
#define BOTtoken "6029608117:AAHkMBMgChMXfPc9Vo5SsX5w0gGQ8YVnCDo"
#define CHAT_ID "1999563723"

UniversalTelegramBot bot(BOTtoken, sta_client);

// Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

// Function to handle messages from telegram
void handleNewMessages(int numNewMessages);

// WIFI FUNCTIONS
void setup_wifi_ap_mode();
void setup_webserver();
void setup_wifi_sta_mode();
void print_wifi_status();


void setup(){
    Serial.begin(115200);

    setup_wifi_ap_mode();
    setup_webserver();

    wifi_status = WiFi.status();
}

    // WL_IDLE_STATUS
    // WL_NO_SSID_AVAIL
    // WL_SCAN_COMPLETED
    // WL_CONNECTED
    // WL_CONNECT_FAILED
    // WL_CONNECTION_LOST
    // WL_DISCONNECTED
 
void loop(){
    if (WiFi.getMode() == WIFI_MODE_STA) {
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("TELEGRAM");
            if (millis() > lastTimeBotRan + botRequestDelay)  {
                int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

                while(numNewMessages) {
                    Serial.println("got response");
                    handleNewMessages(numNewMessages);
                    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
                }
                lastTimeBotRan = millis();
            }
        }
        // else if (WiFi.status() == WL_DISCONNECTED) {
        //     Serial.println("STA mode, but WiFi not connected, enabling AP.");
        //     setup_wifi_ap_mode();
        // }
    }
    else if (WiFi.getMode() == WIFI_MODE_AP) {
        if (!server_on) {
            setup_webserver();
        }
    }
    if (WiFi.status() != wifi_status) {
        Serial.print("WiFi status changed: ");
        print_wifi_status();
    }
}

void setup_wifi_ap_mode() {
    WiFi.disconnect(); // disable sta mode
    Serial.println("Creating AP");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_pass);
    IP = WiFi.softAPIP();
    
    Serial.print("AP Created with IP Gateway ");
    Serial.println(IP);
}

void setup_wifi_sta_mode() {
    WiFi.softAPdisconnect(); // disable ap mode
    server.end();
    server_on = false;
    
    Serial.println("Connecting to WiFi Network (STA)");
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(sta_ssid.c_str(), sta_pass.c_str());
    
    #ifdef ESP32
        sta_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
    #endif
    
    while(WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.print("Connected to WiFi network with local IP : ");
    Serial.println(WiFi.localIP());
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

void setup_webserver() {
    Serial.println("Setting up webserver.");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
        }
    );
    server_on = true;

    // Send a GET request to <ESP_IP>/get?input1=<inputMessage>
    server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
        String inputSSID;
        String inputPASS;
        // GET value on <ESP_IP>/get?input1=<inputMessage>
        if ((request->hasParam(PARAM_INPUT_SSID)) && (request->hasParam(PARAM_INPUT_PASS))) {
            inputSSID = request->getParam(PARAM_INPUT_SSID)->value();
            inputPASS = request->getParam(PARAM_INPUT_PASS)->value();
            if ((inputSSID == "") || (inputPASS == "")) {
                Serial.println("Provide both WiFi SSID and password.");
                request->send(200, "text/html", "Please provide both WiFi SSID and password.<br><a href=\"/\">Return to Home Page</a>");
            }
            else {
                Serial.print("STA SSID: ");
                Serial.println(inputSSID);
                Serial.print("STA PASS: ");
                Serial.println(inputPASS);
                sta_ssid = inputSSID;
                sta_pass = inputPASS;
                setup_wifi_sta_mode();
            }
        }
        else {
            inputSSID = "No SSID";
            inputPASS = "No PASS";
        }
        request->send(200, "text/html", "Connecting to WiFi with SSID: " + inputSSID +
                                        "<br><a href=\"/\">Return to Home Page</a>");
        }
    );
    server.onNotFound(notFound);
    server.begin();
}

void handleNewMessages(int numNewMessages) {
    Serial.println("handleNewMessages");
    Serial.println(String(numNewMessages));

    for (int i=0; i<numNewMessages; i++) {
        // Chat id of the requester
        String chat_id = String(bot.messages[i].chat_id);
        if (chat_id != CHAT_ID) {
            bot.sendMessage(chat_id, "Unauthorized user", "");
            continue;
        }

        // Print the received message
        String text = bot.messages[i].text;
        Serial.println(text);

        String from_name = bot.messages[i].from_name;

        if (text == "/start") {
            String welcome = "Welcome, " + from_name + ".\n";
            welcome += "Use the following commands to control your outputs.\n\n";
            welcome += "/state to request current device state\n";
            welcome += "/setup_wifi to setup WiFi SSID and password\n";
            bot.sendMessage(chat_id, welcome, "");
        }

        if (text == "/state") {
            bot.sendMessage(chat_id, "System is running normally.", "");
        }

        if (text == "/setup_wifi") {
        bot.sendMessage(chat_id, "Setup Wifi by connecting to device AP and go to " + String(IP), "");
        setup_wifi_ap_mode();
        }
    }
}