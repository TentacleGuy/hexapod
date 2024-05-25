/******************************************************************************
Created with PROGRAMINO IDE for Arduino
Project     : Webserver.ino

Libraries   : SoftwareSerial.h
Hexapod_Lib.h
ESP8266WiFi.h
WiFiClient.h
ESP8266WebServer.h
ESP8266mDNS.h
ESP8266HTTPClient.h

Author      : UlliS
******************************************************************************

[ INFO: This example is ONLY for NodeMCU ]

- The sample greate a HTTP-Webserver to control the Hexapod over a Webpage

******************************************************************************/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>

#define NODEMCU
#include <Hexapod_Lib.h>

const char* ssid = "FRITZ!Box 3390";        // SSID - your WiFi's name 
const char* password = "mypassword";        // Password

ESP8266WebServer server(80);

/******************************************************************************
Website

You can use following structure to put website into Arduino code:
String webPage = R"=====(content of web page )=====";
and you don't have to care about: ' " # < > / \  

So below is Website code
******************************************************************************/
String getWebPage = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
    <title>Bootstrap Example</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=yes">
    <link rel="stylesheet" href="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css">
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.2.0/jquery.min.js"></script>
    <script src="https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js"></script>
</head>
<style>
td {
    padding: 5px;
}
</style>
<body>
    <div class="container">
        <div align="center">
            <div class="well well-sm">
                <h4>RoboBug</h4>
            </div>
            <form action='/' method='POST'>
                <button style="width: 60px;" type="button submit" name='ON' value='1' class="btn btn-primary">On</button>
                <button style="width: 60px;" type="button submit" name='OFF' value='1' class="btn btn-primary">Off</button>
                <button style="width: 60px;" type="button submit" name='UP' value='1' class="btn btn-primary">Up</button>
                <button style="width: 65px;" type="button submit" name='DOWN' value='1' class="btn btn-primary">Down</button>
                <table>
                    <br></br>
                    <tr>
                        <td>&nbsp;</td>
                        <td><button style="height: 80px; width: 80px;" type="button submit" name='FWD' value='1' class="btn btn-primary">Forward</button></td>
                        <td>&nbsp;</td>
                    </tr>
                    <tr>
                        <td><button style="height: 80px; width: 80px;" type="button submit" name='LEFT' value='1' class="btn btn-primary">Left</button></td>
                        <td><button style="height: 80px; width: 80px;" type="button submit" name='STOP' value='1' class="btn btn-primary">Stop</button></td>
                        <td><button style="height: 80px; width: 80px;" type="button submit" name='RIGHT' value='1' class="btn btn-primary">Right</button></td>
                    </tr>
                    <tr>
                        <td>&nbsp;</td>
                        <td><button style="height: 80px; width: 80px;" type="button submit" name='BWD' value='1' class="btn btn-primary">Backward</button></td>
                        <td>&nbsp;</td>
                    </tr>
                </table>
            </form>
        </div>
    </div>
</body>
</html>
)=====";


/******************************************************************************
handleRoot
******************************************************************************/
void handleRoot() 
{
    digitalWrite(LED_BUILTIN, HIGH);
    server.send(200, "text/html", getWebPage);
    
    String message = "URI";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET)?"GET":"POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    
    for (uint8_t i=0; i<server.args(); i++)
    {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    
    Serial.println(message);
    
    // only the name is used
    if(server.argName(0)=="ON")
    {
        ROBOT_PWR_ON();  
    }
    else if(server.argName(0)=="OFF")
    {
        ROBOT_PWR_OFF();
    }
    else if(server.argName(0)=="UP")
    {
        ROBOT_HEIGHT(65);
    }
    else if(server.argName(0)=="DOWN")
    {
        ROBOT_HEIGHT(0);
    }
    else if(server.argName(0)=="FWD")
    {
        ROBOT_WALK_FWD();
    }
    else if(server.argName(0)=="LEFT")
    {
        ROBOT_TURN_LEFT();
    }
    else if(server.argName(0)=="STOP")
    {
        ROBOT_STOP();
    }
    else if(server.argName(0)=="RIGHT")
    {
        ROBOT_TURN_RIGHT();
    }
    else if(server.argName(0)=="BWD")
    {
        ROBOT_WALK_BWD();
    }
    else
        ROBOT_STOP();
    
    digitalWrite(LED_BUILTIN, LOW);
}

/******************************************************************************
handleNotFound
******************************************************************************/
void handleNotFound()
{
    digitalWrite(LED_BUILTIN, HIGH);
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET)?"GET":"POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";
    
    for (uint8_t i=0; i<server.args(); i++)
    {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }
    
    server.send(404, "text/plain", message);
    digitalWrite(LED_BUILTIN, LOW);
}

/******************************************************************************
INIT
******************************************************************************/
void setup(void)
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // open serial communications and wait for port to open:
    Serial.begin(SERIAL_STD_BAUD);
    while(!Serial) 
    {
        ;  // wait for serial port to connect. Needed for native USB port only 
    }
    
    WiFi.begin(ssid, password);
    Serial.println("");
    
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    if (MDNS.begin("esp8266")) 
    {
        Serial.println("MDNS responder started");
    }
    
    server.on("/", handleRoot);
    
    server.onNotFound(handleNotFound);
    
    server.begin();
    Serial.println("HTTP server started");
    
    // set the data rate for the SoftwareSerial port (User-Board to Locomotion-Controller)
    SERIAL_CMD.begin(SERIAL_CMD_BAUD);
    
    // reset the Locomotion-Controller
    ROBOT_RESET();
    delay(250);
    ROBOT_RESET();
    delay(150);
    ROBOT_RESET();
    
    // wait for Boot-Up
    delay(1500);
    ROBOT_INIT();
    ROBOT_SPEED(50);                        // set speed to 50
    ROBOT_HEIGHT(0);                        // sit down
    ROBOT_GAINT_MODE(TRIPOD_8);             // tripod 6 gaint mode
    ROBOT_BALANCE_MODE_OFF();               // balance mode off
    
    // print a hello world over the USB connection
    Serial.println("> Hello here is the C-Control Hexapod!");
    
    // blink for ready
    for(int i = 0; i < 5; i++)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
}

/******************************************************************************
MAIN
******************************************************************************/
void loop(void)
{
    #ifdef NODEMCU
        delay(10); // only for NodeMCU with Lib 2.3 to supress the watchdog reset
    #endif
    
    server.handleClient();
}
