#ifndef __AVERAGE_H__
#define __AVERAGE_H__

class Average {
public:
    Average(int count);
    ~Average();
    void add(double v);
    double value() const;
private:
    int total, n, p;
    double* items;
    double sum;
};

#endif
