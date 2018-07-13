#ifndef SORTBYXASC_H
#define SORTBYXASC_H
#include "qpoint3d.h"

class sortByXAsc
{
public:
    sortByXAsc();
    bool operator()(QPoint3D &p1, QPoint3D &p2)
    {
        const double EPS = 1.0e-4;
        if(fabs(p1.getX() - p2.getX())<EPS)
            return (p1.getY() < p2.getY());
        else
            return (p1.getX() < p2.getX());
    }
};

#endif // SORTBYXASC_H
