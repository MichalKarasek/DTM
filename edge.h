#ifndef EDGE_H
#define EDGE_H

#include<list>
#include<QPointF>
#include "qpoint3d.h"

class Edge
{
    private:
        QPoint3D start, end;

    public:
        Edge():start(0.0,0.0,0.0), end(0.0,0.0,0.0){}
        Edge(QPoint3D &a, QPoint3D &b):start(a), end(b){}
        inline QPoint3D & getStart(){return start;}
        inline QPoint3D & getEnd(){return end;}
        static std::list<Edge>::iterator find_pos(std::list<Edge>::iterator, std::list<Edge>::iterator, Edge&);
        void switchOrientation(){
            QPoint3D c = start;
            start = end;
            end = c;
        }
        bool operator == (const Edge &) {return true;}
        static QPoint3D crossEdge(Edge &, Edge &);
};

#endif // EDGE_H
