#include "widget.h"
#include "qpoint3d.h"
#include "algorithms.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication A(argc, argv);
    Widget w;
    w.show();

    /*QPoint3D a(150,50,0);
    QPoint3D b(100,100,0);
    QPoint3D c(150,100,0);
    QPoint3D d(100,0,0);

    Edge aa(a,b);
    Edge bb(c,d);

    QPoint3D centr = Edge::crossEdge(aa,bb);
    qDebug()<<centr.getX()<<centr.getY();*/

    return A.exec();
}
