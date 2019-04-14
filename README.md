# Boiler control

This is my wood boiler control. It currently runs on
[Controllino MEGA](https://www.controllino.biz/controllino-mega/).

The primary tasks are currently doing temperature and pressure
measurements and controlling a mixing valve with 0-10V control voltage
(`valveY` - target valve position, `valveX` - actual valve position)
to maintain the specified (`targetTemp`) value of `temp-tank-to-house`
and operating feed valve to maintain the minimum necessary pressure in
the system. Some of the other tasks to be handled shortly are some
LEDs for alerts (temperature / pressure), controlling circulator pumps
and radiator valves.

The controller is interfaced using MQTT (sorry for inconsistent
parameter names, I was too lazy to update my HomeAssistant config):

```
/devices/boiler/controls/temp-board 26.1225
/devices/boiler/controls/targetTemp 33.0000
/devices/boiler/controls/valveY 48.5920
/devices/boiler/controls/pressure 0.4412
/devices/boiler/controls/temp-tank-to-boiler 20.3700
/devices/boiler/controls/temp-boiler-to-tank 25.3750
/devices/boiler/controls/temp-tank-to-house 32.9375
/devices/boiler/controls/temp-house-to-tank 31.8750
/devices/boiler/controls/temp-tank-a 38.8075
/devices/boiler/controls/temp-tank-b 35.4425
/devices/boiler/controls/temp-tank-c 34.0000
/devices/boiler/controls/valveX 49.8208
/devices/boiler/controls/valve-y 0.0000
/devices/boiler/controls/target-temp 14.0000
/devices/boiler/controls/kp 12.5000
/devices/boiler/controls/ki 0.0200
/devices/boiler/controls/kd 50.0000
/devices/boiler/controls/enable-valve-control 1
/devices/boiler/controls/feedValveOpen 0
/devices/boiler/controls/feed-low-pressure-threshold 0.4000
/devices/boiler/controls/feed-high-pressure-threshold 0.6000
/devices/boiler/controls/enable-feed-valve-control 1
```

This is WiP. More details will be added later.
