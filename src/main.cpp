#include <Arduino.h>

#include <OneWire.h> 
#include <DallasTemperature.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>

const int maxNTemps = 16;
float temps[maxNTemps];
DeviceAddress addrs[maxNTemps];
int curNTemps = 0, curX = 0, curY = 0;

byte mac[] = {
              0xDE, 0xAD, 0xBE, 0x12, 0x33, 0x55
};
IPAddress ip(192, 168, 20, 80);

EthernetServer server(80);

/********************************************************************/
#define PWM_PIN CONTROLLINO_D0
#define X_PIN CONTROLLINO_A3
// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 20 // CONTROLLINO_PIN_HEADER_SDA
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
/********************************************************************/

struct addrDesc {
    char* addr;
    char* desc;
};

struct addrDesc addrDescs[] = {
                               {"280e2a7107000015", "board temp"},
                               {"28ffffffb6ffd622", "boiler->tank"},
                               {"28ffff62fff52217", "tank->boiler"},
                               {"28ffff0effdb2217", "tank->house"},
                               {"28ffffff80082317", "house->tank"},
                               {"", ""}
};

char* getSensorName(char* addr) {
    for (int n = 0; *addrDescs[n].addr; n++) {
        addrDesc* d = addrDescs + n;
        if (!strcmp(addr, d->addr))
            return d->desc;
    }
    return 0;
}

void setup(void) 
{ 
    // pinMode(CONTROLLINO_RTC_CHIP_SELECT, OUTPUT);
    // pinMode(CONTROLLINO_ETHERNET_CHIP_SELECT, OUTPUT);
    
    // digitalWrite(CONTROLLINO_RTC_CHIP_SELECT, LOW);       // inactive
    // digitalWrite(CONTROLLINO_ETHERNET_CHIP_SELECT, HIGH); // inactive
 
    for (int i = 0; i < maxNTemps; i++)
        temps[i] = 0;
    // start serial port 
    Serial.begin(115200); 

    Serial.println("IVAN4TH'S BOILER CONTROL"); 
    // Start up the library 
    sensors.begin();
    pinMode(PWM_PIN, OUTPUT);
    pinMode(X_PIN, INPUT);
    //Ethernet.begin(mac, ip);
    if (!Ethernet.begin(mac))
        Serial.println("ethernet fail...");

    // Check for Ethernet hardware present
    //if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    //  Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    //  while (true) {
    //    delay(1); // do nothing, no point running without Ethernet hardware
    //  }
    //}
    //if (Ethernet.linkStatus() == LinkOFF) {
    //  Serial.println("Ethernet cable is not connected.");
    //}

    // start the server
    server.begin();
    Serial.print("server is at ");
    Serial.println(Ethernet.localIP());
}

enum reqState {
               ReqStart,
               ReqQuery,
               ReqNewLine,
               ReqNonblankLine,
               ReqDone
};

void writeResponse(EthernetClient* client) {
    // send a standard http response header
    client->println("HTTP/1.1 200 OK");
    client->println("Content-Type: text/html");
    client->println("Connection: close");  // the connection will be closed after completion of the response
    client->println("Refresh: 15");  // refresh the page automatically every 5 sec
    client->println();
    client->println("<!DOCTYPE HTML>");
    client->println("<html>");
    // output the value of each analog input pin
    for (int n = 0; n < curNTemps; n++) {
        //int sensorReading = analogRead(analogChannel);
        char addrStr[17];
        char* addr = addrs[n];
        snprintf(addrStr, 17, "%02x%02x%02x%02x%02x%02x%02x%02x",
                 addr[0],
                 addr[1],
                 addr[2],
                 addr[3],
                 addr[4],
                 addr[5],
                 addr[6],
                 addr[7]);
        char* sensorName = getSensorName(addrStr);
        if (!sensorName)
            sensorName = "temp";
        client->print(sensorName);
        client->print("=");
        client->print(temps[n]);
        client->print("; address=");
        client->print(addrStr);

        client->println("<br />");
    }
    client->print("x = ");
    client->print(curX);
    client->print("; ");
    client->print("y = ");
    client->print(curY);
    client->println();
    client->println("</html>");
}

void handleEthernet() {
    // listen for incoming clients
    EthernetClient client = server.available();
    if (client) {
        Serial.println("new client");
        // an http request ends with a blank line
        //bool currentLineIsBlank = true;
        reqState s = ReqStart;
        char digits[16];
        int nDigits = 0;
        while (client.connected() && s != ReqDone) {
            if (client.available()) {
                char c = client.read();
                Serial.write(c);
                switch (s) {
                case ReqStart:
                    //Serial.print("reqStart: ");
                    //Serial.println(c);
                    if (c == '\n')
                        s = ReqNewLine;
                    else if (c == '?')
                        s = ReqQuery;
                    break;
                case ReqQuery:
                    //Serial.print("reqQuery: ");
                    //Serial.println(c);
                    if (c == '\n') {
                        s = ReqNewLine;
                    } else if (isdigit(c)) {
                        digits[nDigits++] = c;
                        if (nDigits == 15)
                            s = ReqNonblankLine;
                    } else
                        s = ReqNonblankLine;
                    break;
                case ReqNewLine:
                    //Serial.print("reqNewLine: ");
                    //Serial.println(c);
                    if (c == '\n') {
                        //Serial.print("nDigits=");
                        //Serial.println(nDigits);
                        if (nDigits > 0) {
                            digits[nDigits] = 0;
                            curY = atoi(digits);
                            if (curY < 0)
                                curY = 0;
                            else if (curY > 255)
                                curY = 255;
                        }
                        writeResponse(&client);
                        s = ReqDone;
                    } else if (c != '\r')
                        s = ReqNonblankLine;
                    break;
                case ReqNonblankLine:
                    //Serial.print("reqNonblankLine: ");
                    //Serial.println(c);
                    if (c == '\n')
                        s = ReqNewLine;
                    break;
                }
            }
        }
        // give the web browser time to receive the data
        delay(1);
        // close the connection:
        client.stop();
        Serial.println("client disconnected");
    }
}

void loop(void) 
{
    handleEthernet();
    ///Serial.print("x: ");
    curX = analogRead(X_PIN);
    ///Serial.println(curX);
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus 
    /********************************************************************/
    // Serial.print(" Requesting temperatures..."); 
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    // Serial.println("DONE"); 
    /********************************************************************/
    int count = sensors.getDeviceCount();
    if (count > maxNTemps)
        count = maxNTemps;
    curNTemps = count;
    float maxT = 0;
    for (int i = 0; i < count; i++) {
        ///Serial.print(i);
        ///Serial.print(": t = "); 
        float t = sensors.getTempCByIndex(i);
        if (t > maxT)
            maxT = t;
        ///Serial.println(t);
        temps[i] = t;
        sensors.getAddress(addrs[i], i);
    }
    /*
      float coef = (maxT - 23) / 10.0;
      int target = (1 - coef) * 255;
      if (target < 0)
      target = 0;
      else if (target > 255)
      target = 255;
      curY = target;
    */
    ///target=0;
    analogWrite(PWM_PIN, curY);
    ///Serial.print("target: ");
    ///Serial.println(curY);
    ///Serial.println("");
    //delay(200); 
} 
