#include <Arduino.h>

#include <OneWire.h> 
#include <DallasTemperature.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>

#include "tbh.h"

const int minX = 65;
const int maxX = 344;
const int maxPWM = 255;
const float maxTargetTemp = 100;
const float minTargetTemp = 0;
const float noControlledTemp = -1000;

// good/possible good for tank->house outflow stabilization:
// static constexpr double defaultTargetTemp = 25;
// static constexpr double defaultKp = 0.07;// good: 0.001; then: 0.02; 0.05 better; 0.08 even better
// static constexpr double defaultKi = 0.0004;// ~good: 0.0003;

static constexpr double defaultTargetTemp = 24;
static constexpr double defaultKp = 0.07;
static constexpr double defaultKi = 0.0001; // acceptable: 6

static int defaultTbhInterval = 1000;

TBH tbh(defaultTargetTemp, defaultKp, defaultKi, defaultTbhInterval);

float targetTemp = defaultTargetTemp;
bool controlTemp = true;

const int maxNTemps = 16;
struct tempItem {
    DeviceAddress addr;
    float t;
    const char* name;
};
tempItem tempItems[maxNTemps];
int curNTemps = 0, curX = 0, curY = 0;

byte mac[] = {
    0xDE, 0xAD, 0xBE, 0x12, 0x33, 0x55
};
// IPAddress ip(192, 168, 20, 80);
IPAddress brokerIP(192, 168, 20, 61);

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

EthernetClient ethClient;
// EthernetServer server(80);
PubSubClient client(brokerIP, 1883, callback, ethClient);

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
    DeviceAddress addr;
    const char* desc;
    bool controlled;
};

struct addrDesc addrDescs[] = {
    { { 0x28, 0x0e, 0x2a, 0x71, 0x07, 0x00, 0x00, 0x15 }, "temp-board", false},
    { { 0x28, 0xff, 0xb6, 0xd6, 0x22, 0x17, 0x04, 0xab }, "temp-boiler-to-tank", false},
    { { 0x28, 0xff, 0x62, 0xf5, 0x22, 0x17, 0x04, 0x12 }, "temp-tank-to-boiler", false},
    { { 0x28, 0xff, 0x0e, 0xdb, 0x22, 0x17, 0x04, 0x47 }, "temp-tank-to-house", false}, // was: true
    { { 0x28, 0xff, 0x80, 0x08, 0x23, 0x17, 0x04, 0x0c }, "temp-house-to-tank", true}, // was: false
    { { 0x28, 0xff, 0xc0, 0x62, 0xa6, 0x16, 0x03, 0x7e }, "temp-tank-a", false},
    { { 0x28, 0xff, 0x6b, 0xa0, 0x24, 0x17, 0x03, 0xea }, "temp-tank-b", false},
    { { 0x28, 0xff, 0xaf, 0x3e, 0x81, 0x14, 0x02, 0xeb }, "temp-tank-c", false},
    { {}, 0}
};

const char* getSensorName(uint8_t* addr, bool* controlled) {
    for (int n = 0; addrDescs[n].desc; n++) {
        addrDesc* d = addrDescs + n;
        if (memcmp(addr, d->addr, 8))
            continue;
        if (controlled)
            *controlled = d->controlled;
        return d->desc;
    }
    return 0;
}


void publishFloat(const char* name, double v) {
    char topic[128];
    snprintf(topic, 128, "/devices/boiler/controls/%s", name);
    char valueStr[64];
    dtostrf(v, 1, 2, valueStr);
    client.publish(topic, valueStr);
}

void publishCur() {
    publishFloat("valveY", curY * 100.0 / maxPWM);
    publishFloat("targetTemp", targetTemp);
    publishFloat("ki", tbh.ki() * 1000);
    publishFloat("kp", tbh.kp() * 1000);
    publishFloat("interval", tbh.interval());
}

void setTargetTemp(double v) {
    controlTemp = true;
    targetTemp = v;
    if (targetTemp > maxTargetTemp)
        targetTemp = maxTargetTemp;
    else if (targetTemp < minTargetTemp)
        targetTemp = minTargetTemp;
    tbh.setTarget(targetTemp);
}

void setCurY(double v) {
    controlTemp = false;
    curY = v * maxPWM / 100.;
    if (curY < 0)
        curY = 0;
    else if (curY > maxPWM)
        curY = maxPWM;
}

void setKi(double v) {
    controlTemp = true;
    tbh.setKi(v / 1000);
}

void setKp(double v) {
    controlTemp = true;
    tbh.setKp(v / 1000);
}

void setInterval(double v) {
    controlTemp = true;
    tbh.setInterval((unsigned long)v);
}

typedef void (*SetValueFunc)(double);

struct TopicItem {
    const char* topic;
    SetValueFunc handler;
};

TopicItem topicItems[] = {
    { "/devices/boiler/controls/targetTemp/on", setTargetTemp },
    { "/devices/boiler/controls/valveY/on", setCurY },
    { "/devices/boiler/controls/ki/on", setKi },
    { "/devices/boiler/controls/kp/on", setKp },
    { "/devices/boiler/controls/interval/on", setInterval },
    { 0, 0 },
};

TopicItem* getTopicItem(const char *topic) {
    for (TopicItem* item = topicItems; item->topic; item++) {
        if (!strcmp(topic, item->topic))
            return item;
    }
    return 0;
}

// Callback function
void callback(char* topic, byte* payload, unsigned int length) {
    TopicItem* topicItem = getTopicItem(topic);
    if (!topicItem) {
        Serial.print("WARNING: unhandled topic: ");
        Serial.println(topic);
        return;
    }
    char* buf = (char*)malloc(length + 1);
    buf[length] = 0;
    memcpy(buf, payload, length);
    Serial.print("*** Accepting value: ");
    Serial.print(topic);
    Serial.print("=");
    Serial.println(buf);
    topicItem->handler(atof(buf));
    publishCur();
    free(buf);
}

void setup(void) 
{ 
    // pinMode(CONTROLLINO_RTC_CHIP_SELECT, OUTPUT);
    // pinMode(CONTROLLINO_ETHERNET_CHIP_SELECT, OUTPUT);
    
    // digitalWrite(CONTROLLINO_RTC_CHIP_SELECT, LOW);       // inactive
    // digitalWrite(CONTROLLINO_ETHERNET_CHIP_SELECT, HIGH); // inactive
 
    // for (int i = 0; i < maxNTemps; i++)
    //     temps[i].t = 0;
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
    else {
        Serial.print("local ip: ");
        Serial.println(Ethernet.localIP());
    }

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
    // server.begin();
    // Serial.print("server is at ");
    // Serial.println(Ethernet.localIP());

    
    Ethernet.begin(mac);
    if (client.connect("arduinoClient")) {
        client.publish("/devices/boiler/msg", "booted");
        // client.subscribe("/devices/boiler/controls/valveY/on");
        for (TopicItem* item = topicItems; item->topic; item++)
            client.subscribe(item->topic);
    }
}

enum reqState {
    ReqStart,
    ReqQuery,
    ReqNewLine,
    ReqNonblankLine,
    ReqDone
};

void loop(void) 
{
    client.loop();

    // handleEthernet();
    ///Serial.print("x: ");
    curX = analogRead(X_PIN);
    float x = (curX - minX) * 100.0 / (maxX - minX);
    x = max(0, min(x, 100));
    publishFloat("valveX", x);
    publishCur();

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
    float controlledTemp = noControlledTemp;
    for (int i = 0; i < count; i++) {
        ///Serial.print(i);
        ///Serial.print(": t = "); 
        sensors.getAddress(tempItems[i].addr, i);
        DeviceAddress& addr = tempItems[i].addr;
        float t = sensors.getTempC(tempItems[i].addr);
        // if (t == DEVICE_DISCONNECTED_C) {
        //     // ...
        //     continue;
        // }
        if (t > maxT)
            maxT = t;
        ///Serial.println(t);
        tempItems[i].t = t;

        bool controlled;
        tempItems[i].name = getSensorName(tempItems[i].addr, &controlled);
        if (!tempItems[i].name) {
            char addrStr[64];
            DeviceAddress& addr = tempItems[i].addr;
            snprintf(addrStr, 64, "Unrecognized address: %02x%02x%02x%02x%02x%02x%02x%02x",
                     addr[0], addr[1], addr[2], addr[3],
                     addr[4], addr[5], addr[6], addr[7]);
            Serial.println(addrStr);
        } else if (controlled)
            controlledTemp = t;

        char addrStr[64];
        snprintf(addrStr, 64, "Got addr: %02x%02x%02x%02x%02x%02x%02x%02x; name = %s; temp = ",
                 addr[0], addr[1], addr[2], addr[3],
                 addr[4], addr[5], addr[6], addr[7],
                 tempItems[i].name ? tempItems[i].name : "<none>");
        Serial.print(addrStr);
        Serial.println(t);

    }

    if (controlledTemp == noControlledTemp)
        Serial.println("WARNING: couldn't read the controlled temp");
    else {
        if (controlTemp && tbh.handle(controlledTemp, millis())) {
            curY = tbh.output() * maxPWM;
            if (curY < 0)
                curY = 0;
            else if (curY > maxPWM)
                curY = maxPWM;
        }
        analogWrite(PWM_PIN, curY);
        publishCur();
    }

    for (int i = 0; i < count; i++) {
        if (!tempItems[i].name || tempItems[i].t == DEVICE_DISCONNECTED_C)
            continue;
        publishFloat(tempItems[i].name, tempItems[i].t);
    }
} 

// TODO: fix reading sensor data
// TODO: reconnect
