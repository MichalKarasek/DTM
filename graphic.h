#ifndef GRAPHIC_H
#define GRAPHIC_H

#include <QGraphicsView>
#include <QWidget>
#include <QtWidgets>

class graphic : public QGraphicsView
{

public:
    graphic(QWidget* parent = 0);
    ~graphic(){}

protected:

    virtual void wheelEvent(QWheelEvent *event);
};

#endif // GRAPHIC_H
