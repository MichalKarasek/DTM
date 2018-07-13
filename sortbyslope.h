#ifndef SORTBYSLOPE_H
#define SORTBYSLOPE_H

#include "triangle.h"

class sortBySlope
{
public:
    sortBySlope();
    bool operator()(Triangle &tr1, Triangle &tr2)
    {
        return (tr1.getSlope() < tr2.getSlope());
    }
};

#endif // SORTBYSLOPE_H
