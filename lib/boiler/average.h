#ifndef __AVERAGE_H__
#define __AVERAGE_H__

class RunningAverage {
public:
    RunningAverage(int count);
    ~RunningAverage();
    void add(double v);
    double value() const;
private:
    int total, n, p;
    double* items;
    double sum;
};

#endif
