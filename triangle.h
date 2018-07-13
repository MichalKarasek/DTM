#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "qpoint3d.h"
#include "edge.h"
#include <QPointF>

class Triangle
{
private:
    QPoint3D p1,p2,p3;
    double slope, exposition;
    QPoint3D centroid;
    int class_cat;

public:
    Triangle(QPoint3D &p1_,QPoint3D &p2_,QPoint3D &p3_, double slope_, double exposition_, QPoint3D centroid_, int class_cat_ = 0):p1(p1_),p2(p2_),p3(p3_),slope(slope_),exposition(exposition_),
        centroid(centroid_),class_cat(class_cat_){}

    inline QPoint3D getP1(){return p1;}
    inline QPoint3D getP2(){return p2;}
    inline QPoint3D getP3(){return p3;}
    inline QPoint3D getCentroid(){return centroid;}
    inline double getSlope()const{return slope;}
    inline double getExposition(){return exposition;}
    inline double getClassCat(){return class_cat;}
    inline void setP1(const QPoint3D &p1){this -> p1 = p1;}
    inline void setP2(const QPoint3D &p2){this -> p2 = p2;}
    inline void setP3(const QPoint3D &p3){this -> p3 = p3;}
    inline void setSlope(double slope){this -> slope = slope;}
    inline void setExposition(double exposition){this -> exposition = exposition;}
    inline void setCentroid(const QPoint3D centroid){this->centroid = centroid;}
    inline void setClassCat(int class_cat){this->class_cat = class_cat;}
    static QPoint3D computeCentroid (Edge &, Edge &);
    static std::vector<int> classifySlope(std::vector<Triangle> &, const size_t);
    static void classifyArea(std::vector<Triangle> &, std::vector<int> &);
};

#endif // TRIANGLE_H
