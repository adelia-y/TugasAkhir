#include "WiFi.h"
#include "ESPAsyncWebServer.h"
 
const char *ssid = "ESP32TA003";
const char *password = "testpassword";
 
AsyncWebServer server(80);

const char* PARAM_INPUT_SSID = "inputssid";
const char* PARAM_INPUT_PASS = "inputpass";

// HTML web page to handle 3 input fields (input1, input2, input3)
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

void setup(){
    Serial.begin(115200);

    WiFi.softAP(ssid, password);

    Serial.println();
    Serial.print("IP address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
        }
    );

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
            }
            else {
                Serial.println(inputSSID);
                Serial.println(inputPASS);
            }
        }
        else {
            inputSSID = "No SSID";
            inputPASS = "No PASS";
        }
        request->send(200, "text/html", "HTTP GET request sent to your ESP with value: " + inputSSID + " " + inputPASS +
                                        "<br><a href=\"/\">Return to Home Page</a>");
        }
    );
    server.onNotFound(notFound);
    server.begin();
}
 
void loop(){}