#ifndef ALGORITHMS_H
#define ALGORITHMS_H
#include <vector>
#include <QPolygon>
#include <stack>
#include "edge.h"
#include "qpoint3d.h"
#include "triangle.h"
#include "edge.h"
#include "qdebug.h"

class Algorithms
{
public:
    Algorithms();
    static int getPosition(QPoint3D &q,QPoint3D &a, QPoint3D &b);
    static int getPosition(QPoint &q,QPoint &a, QPoint &b);
    static double getCircleRadius(QPoint3D &p1, QPoint3D &p2,QPoint3D &p3);
    static int getDelaunayPoint(Edge &e, std::vector <QPoint3D> &points);
    static int getNearestPoint(QPoint3D &p, std::vector <QPoint3D> &points);
    static double getAngle(QPoint3D &,QPoint3D &,QPoint3D &, QPoint3D &);
    static double getAngle(QPoint &,QPoint &,QPoint &, QPoint &);
    static QPolygon grahamScan(std::vector<QPoint> &points);
    static bool sortByAngle(QPoint &a, QPoint &b);
    static std::vector<QPoint3D> generate_Cumulus(uint);
    static std::vector<QPoint3D> generate_Hillrest(uint);
    static std::vector<QPoint3D> generate_Valley(uint);
    static std::vector<QPoint3D> generate_Ridge(uint);
    static double distance(QPoint &a, QPoint &b){

        double dx = b.x() - a.x();
        double dy = b.y() - a.y();
        return sqrt(dx*dx + dy*dy);
    }
    static QPoint nextToTop(std::stack<QPoint> &stack){

        QPoint p = stack.top();
        stack.pop();
        QPoint q = stack.top();
        stack.push(p);
        return q;
    }
    /*inline static void getedgestartX(Edge &a){qDebug()<<QString("%1").arg(a.getStart().getX(),0,'q',10);}
    inline static void getedgestartY(Edge &a){qDebug()<<QString("%1").arg(a.getStart().getY(),0,'q',10);}
    inline static void getedgeendX(Edge &a){qDebug()<<QString("%1").arg(a.getEnd().getX(),0,'q',10);}
    inline static void getedgenedY(Edge &a){qDebug()<<QString("%1").arg(a.getEnd().getY(),0,'q',10);}*/

    static std::vector<Edge> dt(std::vector <QPoint3D> &points);
    static QPoint3D getConPoint(QPoint3D &p1,QPoint3D &p2, double z);
    static bool compare(Edge &, Edge &);
    static std::vector<Edge> createContours(std::vector<Triangle>&dt, double zmin, double zmax, double h);
    static std::vector<Triangle> convertDTM(std::vector<Edge>&dt);
    static double getSlope(Triangle &tr);
    static double getExposition(Triangle &tr);
    static std::pair<double, double> minmaxZ(std::vector<QPoint3D> &);
    static int getRayPos(QPoint3D &, std::vector<QPoint3D> &, bool closed = true);
};

#endif // ALGORITHMS_H
