#include <Arduino.h>

#include <OneWire.h> 
#include <DallasTemperature.h>
#include <Controllino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include "tbh.h"
#include "average.h"

const int minX = 65;
const int maxX = 344;
const int maxPWM = 255;
const float maxTargetTemp = 100;
const float minTargetTemp = 0;
const int publishInterval = 0; // set to e.g. 3000 to reduce publish rate
const int nTempAvg = 50; // was: 20
const int nPressureAvg = 50;
const int mqttReconnectInterval = 5000;

// good/possible good for tank->house outflow stabilization:
// static constexpr double defaultTargetTemp = 25;
// static constexpr double defaultKp = 0.07;// good: 0.001; then: 0.02; 0.05 better; 0.08 even better
// static constexpr double defaultKi = 0.0004;// ~good: 0.0003;

static constexpr double defaultTargetTemp = 15;
static constexpr double defaultKp = 0.08; // TO SET: 0.640; was: 0.07; also trying: 0
static constexpr double defaultKi = 0; // 0.00015 ~ok; 0.008 (also try 0.016); 0.005 - oscillates at 15c; was trying: 0.003; 0.0015; trying: 0.0005
// actually: use ki=0.00006 when *below* the target temp by < 2 degC; otherwise 0.001 or higher
//static constexpr double pressureCoefA = 0.023031193008219;
//static constexpr double pressureCoefB = -2.2042703867744 - 0.04 + 0.3 - 0.08;

static constexpr double pressureCoefA = 0.01984801999082288;
static constexpr double pressureCoefB = -1.4249809837242795;

unsigned long lastPublished = 0;

static int defaultTbhInterval = 1000;

double targetTemp = defaultTargetTemp;
double curY = 0;

const int maxNTemps = 16;
struct tempItem {
    DeviceAddress addr;
    float t;
    const char* name;
    bool controlled;
};
tempItem tempItems[maxNTemps];
int tempCount = 0, curX = 0;
bool scanned = false;

Average tempAvg(nTempAvg);
Average pAvg(nPressureAvg);
TBH tbh(defaultTargetTemp, defaultKp, defaultKi, defaultTbhInterval);

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
#define PRESSURE_PIN CONTROLLINO_A1
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
    { { 0x28, 0xff, 0x0e, 0xdb, 0x22, 0x17, 0x04, 0x47 }, "temp-tank-to-house", true},
    { { 0x28, 0xff, 0x80, 0x08, 0x23, 0x17, 0x04, 0x0c }, "temp-house-to-tank", false},
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
    if (!client.connected())
        return;
    char topic[128];
    snprintf(topic, 128, "/devices/boiler/controls/%s", name);
    char valueStr[64];
    dtostrf(v, 1, 4, valueStr);
    client.publish(topic, valueStr);
}

void publishCur() {
    publishFloat("valveY", curY * 100.0 / maxPWM);
    publishFloat("targetTemp", targetTemp);
    publishFloat("ki", tbh.ki() * 1000);
    publishFloat("kp", tbh.kp() * 1000);
}

void setTargetTemp(double v) {
    tbh.setAutoMode(true);
    targetTemp = v;
    if (targetTemp > maxTargetTemp)
        targetTemp = maxTargetTemp;
    else if (targetTemp < minTargetTemp)
        targetTemp = minTargetTemp;
    tbh.setTarget(targetTemp);
}

void setCurY(double v) {
    tbh.setAutoMode(false);
    curY = v * maxPWM / 100.;
    if (curY < 0)
        curY = 0;
    else if (curY > maxPWM)
        curY = maxPWM;
    analogWrite(PWM_PIN, curY);
}

void setKp(double v) {
    tbh.setAutoMode(true);
    tbh.setKp(v / 1000);
}

void setKi(double v) {
    tbh.setAutoMode(true);
    tbh.setKi(v / 1000);
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

long lastReconnectAttempt = 0;

bool reconnect()
{
    if (client.connect("arduinoClient")) {
        client.publish("/devices/boiler/msg", "online");
        for (TopicItem* item = topicItems; item->topic; item++)
            client.subscribe(item->topic);
    }
    return client.connected();
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
    pinMode(PRESSURE_PIN, INPUT);
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
    delay(1500);
    lastReconnectAttempt = 0;
}

void scanSensors()
{
    int count = sensors.getDeviceCount();
    if (count > maxNTemps)
        count = maxNTemps;
    int n = 0;
    for (int i = 0; i < count; i++) {
        ///Serial.print(i);
        ///Serial.print(": t = "); 
        if (!sensors.getAddress(tempItems[n].addr, i)) {
            Serial.print("WARNING: couldn't get address for index ");
            Serial.println(i);
        }
        tempItems[n].name = getSensorName(tempItems[n].addr, &tempItems[n].controlled);
        char addrStr[64];
        DeviceAddress& addr = tempItems[n].addr;
        snprintf(addrStr, 64, "%02x%02x%02x%02x%02x%02x%02x%02x",
                 addr[0], addr[1], addr[2], addr[3],
                 addr[4], addr[5], addr[6], addr[7]);
        if (tempItems[n].name) {
            Serial.print("Found sensor: ");
            Serial.print(tempItems[n].name);
            Serial.print(" at ");
            Serial.println(addrStr);
            n++;
        } else {
            Serial.print("Unrecognized sensor: ");
            Serial.println(addrStr);
        }
    }
    tempCount = n;
    Serial.print("Got ");
    Serial.print(tempCount);
    Serial.println(" sensors.");
    scanned = tempCount > 0;
}

void mqttClientLoop()
{
    if (client.connected()) {
        client.loop();
        return;
    }

    long now = millis();
    if (now - lastReconnectAttempt > mqttReconnectInterval) {
        lastReconnectAttempt = now;
        if (reconnect())
            lastReconnectAttempt = 0;
    }
}

void loop(void) 
{
    mqttClientLoop();

    bool shouldPublish = false;
    unsigned long curTime = millis();
    if (curTime - lastPublished >= publishInterval) {
        lastPublished = curTime;
        shouldPublish = true;
    }

    // handleEthernet();
    ///Serial.print("x: ");
    ///Serial.println(curX);
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus 
    /********************************************************************/
    // Serial.print(" Requesting temperatures..."); 
    sensors.requestTemperatures(); // Send the command to get temperature readings 
    // Serial.println("DONE"); 
    /********************************************************************/
    if (!scanned) {
        scanSensors();
        return;
    }

    int controlledTempIndex;
    for (int i = 0; i < tempCount; i++) {
        if (!shouldPublish && !tempItems[i].controlled)
            continue;

        ///Serial.print(i);
        ///Serial.print(": t = "); 
        // DeviceAddress& addr = tempItems[i].addr;
        float t = sensors.getTempC(tempItems[i].addr);
        if (t == DEVICE_DISCONNECTED_C) {
            Serial.print("WARNING: sensor disconnected: ");
            Serial.println(tempItems[i].name);
            tempItems[i].t = t;
            continue;
        }
        tempItems[i].t = t;
        ///Serial.println(t);

        if (tempItems[i].controlled)
            controlledTempIndex = i;

        // char addrStr[64];
        // snprintf(addrStr, 64, "Got addr: %02x%02x%02x%02x%02x%02x%02x%02x; name = %s; temp = ",
        //          addr[0], addr[1], addr[2], addr[3],
        //          addr[4], addr[5], addr[6], addr[7],
        //          tempItems[i].name ? tempItems[i].name : "<none>");
        // Serial.print(addrStr);
        // Serial.println(t);

    }

    if (controlledTempIndex < 0) {
        Serial.println("WARNING: couldn't read the controlled temp");
    } else {
        double curTemp = tempItems[controlledTempIndex].t;
        tempAvg.add(curTemp);
        double avgTemp = tempAvg.value();
        tempItems[controlledTempIndex].t = avgTemp; // publish averaged temp

        Serial.print("Measured=");
        Serial.print(curTemp);
        Serial.print("; avg=");
        Serial.print(avgTemp);
        Serial.print("; setpoint=");
        Serial.println(targetTemp);
 
        if (tbh.handle(avgTemp, millis())) {
            curY = tbh.output() * maxPWM;
            if (curY < 0)
                curY = 0;
            else if (curY > maxPWM)
                curY = maxPWM;
            analogWrite(PWM_PIN, curY);
            if (shouldPublish)
                publishCur();
            else
                publishFloat("valveY", curY * 100.0 / maxPWM);
        }
    }
    if (shouldPublish) {
        curX = analogRead(X_PIN);
        float x = (curX - minX) * 100.0 / (maxX - minX);
        x = max(0, min(x, 100));
        publishFloat("valveX", x);

        double p = analogRead(PRESSURE_PIN);
        publishFloat("pressureRaw", p);
        double curP = pressureCoefA * p + pressureCoefB;
        pAvg.add(curP);
        double avgP = pAvg.value();
        publishFloat("pressure", avgP);

        for (int i = 0; i < tempCount; i++) {
            if (!tempItems[i].name || tempItems[i].t == DEVICE_DISCONNECTED_C)
                continue;
            publishFloat(tempItems[i].name, tempItems[i].t);
        }
    }
} 

// TODO: fix reading sensor data
// TODO: reconnect
// TODO: eeprom
