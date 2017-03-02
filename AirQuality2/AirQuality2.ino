#include <plotly_streaming_wifi.h>

#include <SPI.h>
#include <WiFi.h>

//Sensor Setup
#define airquality_sensor_pin 1
#define gas_sensor_pin 1

// Sign up to plotly here: https://plot.ly
// View your API key and streamtokens here: https://plot.ly/settings
#define nTraces 2
// View your tokens here: https://plot.ly/settings
// Supply as many tokens as data traces
// e.g. if you want to ploty A0 and A1 vs time, supply two tokens
char *tokens[nTraces] = {"9h4p52cvdf", "v5hidkar00"};
// arguments: username, api key, streaming token, filename
plotly graph("eccentricayman", "v5jyve1326", tokens, "out", nTraces);

//byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
//byte my_ip[] = { 165, 155, 200, 122 }; // google will tell you: "public ip address"

int status = WL_IDLE_STATUS;     // the Wifi radio's status
char ssid[] = "ncpsp"; //  your network SSID (name) 
char pass[] = "475D783003@RACKID78M475"; // // your network password

void wifi_connect(){
    // attempt to connect using WPA2 encryption:
    Serial.println("... Attempting to connect to WPA network...");
    status = WiFi.begin(ssid, pass);
    // if you're not connected, stop here:
    if ( status != WL_CONNECTED) { 
      Serial.println("... Couldn't get a WiFi connection, trying again");
      wifi_connect();
    } 
    // if you are connected, print out info about the connection:
    else {
      Serial.println("... Connected to network");
    }
}

void setup() {
  graph.maxpoints = 100;
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  wifi_connect();

  graph.fileopt="overwrite"; // See the "Usage" section in https://github.com/plotly/arduino-api for details
  bool success;
  success = graph.init();
  if(!success){while(true){}}
  graph.openStream();
}

unsigned long x;
int y;

void loop() {
    int airquality_value = analogRead(airquality_sensor_pin);
    int gas_value = analogRead(gas_sensor_pin);
    float volume = (float)gas_value/1024*5.0*1000;
    graph.plot(millis(), airquality_value, tokens[0]);
    graph.plot(millis(), volume, tokens[1]);
    delay(50);
}

