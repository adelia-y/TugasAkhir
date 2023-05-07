#include "WiFi.h"
#include "ESPAsyncWebServer.h"

#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

// WIFI
enum wifi_modes {STA, AP};
wifi_modes wifi_mode;
bool setup_sta = false;
bool setup_ap = false;
// ----------------- WIFI ACCESS POINT MODE
const char *ap_ssid = "ESP32TA003";
const char *ap_pass = "testpassword";

AsyncWebServer server(80);
IPAddress IP = IPAddress (22, 23, 1, 3);
IPAddress gateway = IPAddress (22, 23, 1, 3);
IPAddress NMask = IPAddress (255, 255, 255, 0);
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
unsigned long connection_init = 0; // ms
unsigned long connection_timeout = 15000; // ms

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

 
void loop(){
    if (wifi_mode == STA) {
        if (WiFi.status() == WL_CONNECTED) {
            if (millis() > lastTimeBotRan + botRequestDelay)  {
                int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

                while(numNewMessages) {
                    Serial.println("Got response from Telegram.");
                    handleNewMessages(numNewMessages);
                    numNewMessages = bot.getUpdates(bot.last_message_received + 1);
                }
                lastTimeBotRan = millis();
            }
        }
        else if ((millis() - connection_init) > connection_timeout) {
            Serial.println("Connection timeout. WiFi in STA mode, but not connected. Enabling AP.");
            setup_ap = true;
        }
    }
    else if (wifi_mode == AP) {
        if (!server_on) {
            setup_webserver();
        }
    }

    if (WiFi.status() != wifi_status) {
        wifi_status = WiFi.status();
        print_wifi_status();
        if (wifi_status == WL_CONNECTED) {
            Serial.print("Connected to WiFi network with local IP: ");
            Serial.println(WiFi.localIP());
        }
    }

    if (setup_sta) {
        setup_wifi_sta_mode();
    }
    else if (setup_ap) {
        setup_wifi_ap_mode();
    }
}

void setup_wifi_ap_mode() {
    wifi_mode = AP;
    Serial.println("Switching to AP mode.");

    WiFi.disconnect(); // disable sta mode
    Serial.println("Creating Access Point...");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_pass);
    
    WiFi.softAPConfig(IP, IP, NMask);
    IP = WiFi.softAPIP();
    Serial.print("AP created with IP gateway: ");
    Serial.println(IP);

    setup_ap = false;
}

void setup_wifi_sta_mode() {
    wifi_mode = STA;
    Serial.println("Switching to STA mode.");

    server.end();
    WiFi.softAPdisconnect(); // disable ap mode
    server_on = false;
    Serial.println("Webserver terminated.");
    
    Serial.println("Connecting to WiFi Network...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(sta_ssid.c_str(), sta_pass.c_str());
    connection_init = millis();

    #ifdef ESP32
        sta_client.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
    #endif

    setup_sta = false;
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
        Serial.println("WiFi is not connected.");
        break;
    }
}

void setup_webserver() {
    Serial.println("Setting up webserver.");
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
    });
    server_on = true;
    Serial.println("Webserver running at 22.23.1.3.");

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
                request->send(200, "text/html", "Please provide both WiFi SSID and password.<br><a href=\"/\">Return to home page.</a>");
            }
            else {
                request->send(200, "text/html", "Connecting to WiFi with SSID: " + inputSSID +
                                    "<br>Exit this page and go to Telegram bot ta222301001_bot."
                                    "<br><a href=\"/\">If the bot doesn't respond within 20 seconds, return to this home page.</a>");
                Serial.print("STA SSID: ");
                Serial.println(inputSSID);
                Serial.print("STA PASS: ");
                Serial.println(inputPASS);
                sta_ssid = inputSSID;
                sta_pass = inputPASS;
                setup_sta = true;
            }
        }
        else {
            inputSSID = "No SSID";
            inputPASS = "No PASS";
        }
    });
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
        bot.sendMessage(chat_id, "Setup Wifi by connecting to device WiFi ESP32TA003 and go to: 22.23.1.3", "");
        setup_ap = true;
        }
    }
}