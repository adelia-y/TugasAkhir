#include "WiFi.h"
#include "WebServer.h"
// #include "SPIFFS.h"
// #include "ESPAsyncWebServer.h"

#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>   // Universal Telegram Bot Library written by Brian Lough: https://github.com/witnessmenow/Universal-Arduino-Telegram-Bot
#include <ArduinoJson.h>

// WIFI
const char *sta_ssid = "Hotel Del Luna";
const char *sta_pass = "babablackship3";
const char *ap_ssid = "ESP32TA003";
const char *ap_pass = "amoniamonitoring";

WiFiServer ap_server(80); // Set web server port number to 80
WiFiClient ap_client;

String request; // HTTP request
IPAddress IP; 

// TELEGRAM
#define BOTtoken "6029608117:AAHkMBMgChMXfPc9Vo5SsX5w0gGQ8YVnCDo"
#define CHAT_ID "1999563723"

WiFiClientSecure sta_client;
UniversalTelegramBot bot(BOTtoken, sta_client);

// Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

const int ledPin = 2;
bool ledState = LOW;

void handleNewMessages(int numNewMessages);

void html();

void setup_ap_mode();
void setup_sta_mode();

void setup() {
    Serial.begin(115200);  /*Baud rate for serial communication*/
    
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, ledState);

    setup_ap_mode();
}


void loop() {
    
    if (WiFi.getMode() == WIFI_MODE_STA) {
        if (WiFi.status() == WL_CONNECTED) {
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
        else {
            Serial.println("STA mode, but WiFi not connected, enabling AP.");
            setup_ap_mode();
        }
    }
    else if (WiFi.getMode() == WIFI_MODE_AP) {
        ap_client = ap_server.available();
        
        if (!ap_client) {return;}

        while (ap_client.connected()) {
            if (ap_client.available()) {
                char c = ap_client.read();
                request += c;

                if (c == '\n') {
                    if (request.indexOf("GET /WIFI_ON") != -1) {
                        setup_sta_mode();
                    }
                    if (request.indexOf("GET /WIFI_OFF") != -1) {
                        if (WiFi.status() == WL_CONNECTED) {
                            WiFi.disconnect();
                            while(WiFi.status() != WL_DISCONNECTED) {
                                Serial.print(".");
                                delay(100);
                            }
                            Serial.println("WIFI DISCONNECTED");
                        }
                    }
                html();
                break;
                }
            }
        }
        delay(1);
        request="";
        ap_client.stop();
    }
}

void html() {
    ap_client.println("HTTP/1.1 200 OK");
    ap_client.println("Content-Type: text/html");
    ap_client.println("Connection: close");
    ap_client.println();

    ap_client.println("<!DOCTYPE HTML>");
    ap_client.println("<html>");

    ap_client.println("<head>");
    ap_client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
    ap_client.println("<link rel=\"icon\" href=\"data:,\">");
    ap_client.println("<style>");
    ap_client.println("html { font-family: Roboto; display: inline-block; margin: 0px auto; text-align: center;}");
    ap_client.println(".button {background-color: #4CAF50; border: none; color: white; padding: 15px 32px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer;");
    ap_client.println("text-decoration: none; font-size: 25px; margin: 2px; cursor: pointer;}");
    ap_client.println(".button_ON {background-color: white; color: black; border: 2px solid #4CAF50;}");
    ap_client.println(".button_OFF {background-color: white; color: black; border: 2px solid #f44336;}");
    ap_client.println("</style>");
    ap_client.println("</head>");
    ap_client.println("<body>");
    ap_client.println("<h2>Ammonia Monitoring</h2>");
    ap_client.println("<p>Click to Turn ON and OFF WiFi</p>");

    ap_client.print("<p><a href=\"/WIFI_ON\n\"><button class=\"button button_ON\">ON</button></a></p>"); 
    // if(WiFi.mode() == WL_DISCONNECTED) {
    // }
    // else {
    //     ap_client.print("<p><a href=\"/WIFI_OFF\n\"><button class=\"button button_OFF\">OFF</button></a></p>"); 
    // } 

    ap_client.println("</body>");
    ap_client.println("</html>");     
}

// void html() {
//     client.println("HTTP/1.1 200 OK");
//     client.println("Content-Type: text/html");
//     client.println("Connection: close");
//     client.println();

//     client.println("<!DOCTYPE HTML>");
//     client.println("<html>");
//     client.println("<head>");
//     client.println("<meta content=\"text/html; charset=ISO-8859-1\" http-equiv=\"content-type\">");
//     client.println("<meta name = \"viewport\" content = \"width = device-width, initial-scale = 1.0, maximum-scale = 1.0, user-scalable=0\">");
//     client.println("<title>WiFi Creds Form</title>");
//     client.println("<style>");
//     client.println("body { background-color: #808080; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; text-align:center; }");
//     client.println("</style>");
//     client.println("</head>");
//     client.println("<body>");
//     client.println("<h3>Enter your WiFi credentials</h3>");
//     client.println("<form action=\"/\" method=\"post\">");
//     client.println("<p>");
//     client.println("<label>SSID:&nbsp;</label>");
//     client.println("<input maxlength=\"30\" name=\"ssid\"><br>");
//     client.println("<label>Key:&nbsp;&nbsp;&nbsp;&nbsp;</label><input maxlength=\"30\" name=\"password\"><br><br>");
//     client.println("<input type=\"submit\" value=\"Save\">");
//     client.println("</p>");
//     client.println("</form>");
//     client.println("</body>");
//     client.println("</html>");
// }

void handleNewMessages(int numNewMessages) {
  Serial.println("handleNewMessages");
  Serial.println(String(numNewMessages));

  for (int i=0; i<numNewMessages; i++) {
    // Chat id of the requester
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID){
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
      welcome += "/led_on to turn GPIO ON \n";
      welcome += "/led_off to turn GPIO OFF \n";
      welcome += "/state to request current GPIO state \n";
      bot.sendMessage(chat_id, welcome, "");
    }

    if (text == "/led_on") {
      bot.sendMessage(chat_id, "LED state set to ON", "");
      ledState = HIGH;
      digitalWrite(ledPin, ledState);
    }
    
    if (text == "/led_off") {
      bot.sendMessage(chat_id, "LED state set to OFF", "");
      ledState = LOW;
      digitalWrite(ledPin, ledState);
    }
    
    if (text == "/state") {
      if (digitalRead(ledPin)){
        bot.sendMessage(chat_id, "LED is ON", "");
      }
      else{
        bot.sendMessage(chat_id, "LED is OFF", "");
      }
    }

    if (text == "/wifi") {
      bot.sendMessage(chat_id, "Setup Wifi", "");
      setup_ap_mode();
    }
  }
}

void setup_ap_mode() {
    WiFi.disconnect(); // disable sta mode
    Serial.println("Creating AP");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ap_ssid, ap_pass);
    IP = WiFi.softAPIP();
    
    Serial.print("AP Created with IP Gateway ");
    Serial.println(IP);
    
    ap_server.begin();
}

void setup_sta_mode() {
    WiFi.softAPdisconnect(); // disable ap mode
    Serial.println("Connecting to WiFi Network (STA)");
    
    WiFi.mode(WIFI_STA);
    WiFi.begin(sta_ssid, sta_pass);
    
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