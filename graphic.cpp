#include "graphic.h"
#include <QPoint>

graphic::graphic(QWidget *parent) : QGraphicsView(parent)
{
    setResizeAnchor(QGraphicsView::AnchorUnderMouse);
}

void graphic::wheelEvent(QWheelEvent* e)
{
    double scaleFactor = 1.3;

    if(e->delta() > 0)
    {
        scale(scaleFactor, scaleFactor);
    }
    else
    {
          scale(1/scaleFactor, 1/scaleFactor);
    }
}
