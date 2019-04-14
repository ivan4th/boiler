#include "average.h"

RunningAverage::RunningAverage(int count): total(count), n(0), p(0), sum(0)
{
    items = new double[count];
}

RunningAverage::~RunningAverage()
{
    delete items;
}

void RunningAverage::add(double v)
{
    if (n == total)
        sum -= items[p];
    else
        n++;
    items[p++] = v;
    if (p == total)
        p = 0;
    sum += v;
}

double RunningAverage::value() const
{
    return sum / n;
}
