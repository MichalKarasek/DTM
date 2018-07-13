#include "edge.h"
#include "qdebug.h"

std::list<Edge>::iterator Edge::find_pos(std::list<Edge>::iterator first, std::list<Edge>::iterator last, Edge &e)
{
    Edge c;
    while(first!=last)
    {
        c = *first;
        if(c.getEnd().getX() == e.getEnd().getX() && c.getEnd().getY() == e.getEnd().getY() && c.getStart().getX() == e.getStart().getX() && c.getStart().getY() == e.getStart().getY())
        {
            return first;
        }
        ++first;
    }
    return last;
}

QPoint3D Edge::crossEdge(Edge &a, Edge &b)
{
    double x1 = a.getStart().getX();
    double y1 = a.getStart().getY();
    double x2 = a.getEnd().getX();
    double y2 = a.getEnd().getY();

    double x3 = b.getStart().getX();
    double y3 = b.getStart().getY();
    double x4 = b.getEnd().getX();
    double y4 = b.getEnd().getY();

    double x = ((x2*y1-x1*y2)*(x4-x3)-(x4*y3-x3*y4)*(x2-x1))/((x2-x1)*(y4-y3)-(x4-x3)*(y2-y1));
    double y = ((x2*y1-x1*y2)*(y4-y3)-(x4*y3-x3*y4)*(y2-y1))/((x2-x1)*(y4-y3)-(x4-x3)*(y2-y1));

    return QPoint3D(x,y,0);
}
