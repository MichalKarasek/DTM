#ifndef SORTBYEXPOSITION_H
#define SORTBYEXPOSITION_H
#include "triangle.h"

class SortByExposition
{
public:
    SortByExposition();
    bool operator()(Triangle &tr1, Triangle &tr2)
    {
        return (tr1.getExposition() < tr2.getExposition());
    }
};

#endif // SORTBYEXPOSITION_H
