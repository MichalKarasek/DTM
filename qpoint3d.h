#ifndef QPOINT3D_H
#define QPOINT3D_H

#include <QPointF>

class QPoint3D : public QPointF
{
    private:
        double z;

    public:
        QPoint3D():QPointF(0.0,0.0),z(0.0){}
        QPoint3D(double x, double y, double zz):QPointF(x,y),z(zz){}

        double getX(){return this->x();}
        double getY(){return y();}
        double getZ(){return z;}
};

#endif // QPOINT3D_H

