#ifndef SORTBYYASC_H
#define SORTBYYASC_H
#include <QPoint>

class sortByYAsc
{
public:
    sortByYAsc();
    bool operator()(QPoint &p1, QPoint &p2)
    {
        const double EPS = 1.0e-4;
        if(std::abs(p1.y() - p2.y())<EPS)
            return (p1.x() < p2.x());
        else
            return (p1.y() < p2.y());

    }
};

#endif // SORTBYYASC_H
