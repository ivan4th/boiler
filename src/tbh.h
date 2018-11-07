#ifndef __TBH_H__
#define __TBH_H__

// Take Back Half
class TBH {
public:
    TBH(double target, double kp, double ki, unsigned long interval);
    void setAutoMode(bool autoMode) { _autoMode = autoMode; }
    bool autoMode() const { return _autoMode; }
    void setTarget(double target) { _setPoint = target; }
    void setKp(double kp) { _kp = kp; }
    void setKi(double ki) { _ki = ki; }
    void setInterval(unsigned long interval) { _interval = interval; }
    bool handle(double input, unsigned long t);
    double output() const { return _output; }
    double kp() const { return _kp; }
    double ki() const { return _ki; }
    unsigned long interval() const { return _interval; }
private:
    bool _autoMode = true;
    double _setPoint;
    double _kp;
    double _ki;
    double _intTerm = 0;
    double _output = 0;
    unsigned long _interval;
    unsigned long _nextUpdate = 0;
    double _lastInput = -1;
    bool _gotLastInput = false;
    unsigned long _lastZeroCross = 0;
    double _lastZeroCrossIntTerm = 0;
    bool _gotZeroCross = false;
    static constexpr double minIntTerm = 0;
    static constexpr double maxIntTerm = 1;
};

#endif
