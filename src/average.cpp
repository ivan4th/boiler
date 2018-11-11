#include "average.h"

Average::Average(int count): total(count), n(0), p(0), sum(0)
{
    items = new double[count];
}

Average::~Average()
{
    delete items;
}

void Average::add(double v)
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

double Average::value() const
{
    return sum / n;
}
