#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "edge.h"
#include "qpoint3d.h"
#include "qgraphicsscene.h"
#include "triangle.h"

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:

    QPoint3D process_line(QString &, bool option = true);
    void paint_data(std::vector<QPoint3D> &);
    void paint_triangles(std::vector<Triangle> &);
    void paint_mask();
    void process_triangles();
    void paint_slope();
    void paint_contour();
    void paint_exposition();
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private slots:

    void on_openfile_clicked();

    void on_dmt_clicked();

    void on_createcontour_clicked();

    void on_cutbymask_clicked();

    void on_getExposition_clicked();

    void on_getSlope_clicked();

    void on_generateShape_clicked();

private:

    void clear_scene();
    void clear_data();

    //Data structures
    Ui::Widget *ui;
    std::vector<QPoint3D> DMT_points;
    std::vector<Edge> tin;
    std::vector<Triangle> triangles;
    QGraphicsScene *scene;
    std::pair<double, double> minmax;
    std::vector<Edge> contour;
    std::vector<QPoint3D> mask;
    std::vector<Triangle> exposition;
    std::vector<Triangle> slope;
};

#endif // WIDGET_H
