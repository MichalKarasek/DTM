#include <QGroupBox>
#include "widget.h"
#include "ui_widget.h"
#include "qdebug.h"
#include "qfiledialog.h"
#include "qerrormessage.h"
#include "algorithms.h"
#include "sortbyslope.h"
#include "edge.h"
#include "sortbyexposition.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowTitle(tr("DMT generator 1.0"));
    ui->openfile->setText(tr("Open file"));
    ui->getExposition->setText(tr("Exposition"));
    ui->getSlope->setText(tr("Slope"));
    ui->createcontour->setText(tr("Create contours"));
    ui->cutbymask->setText(tr("Cut DMT by mask"));
    ui->dmt->setText(tr("Create DMT"));
    ui->generateShape->setText(tr("Generate shape"));

    ui->getExposition->setEnabled(false);
    ui->getSlope->setEnabled(false);
}

Widget::~Widget()
{
    delete scene;
    delete ui;
}

void Widget::on_openfile_clicked()
{
    clear_scene();
    scene = new QGraphicsScene(this);
    ui->canvas->setScene(scene);
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open polygon .txt file"), "", tr("Text files(*.txt);;All Files (*)"));

    if(fileName.isEmpty())
    {
        return;
    }
    else
    {
        QFile file(fileName);
        QTextStream in(&file);

        clear_data();

        // Check if file is open
        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QErrorMessage *message = new QErrorMessage(this);
            message->setWindowTitle(tr("No file found"));
            message->showMessage(tr("Failed to open the source file."));
            message->setAttribute(Qt::WA_DeleteOnClose, true);
            return;
        }
        else
        {
             // Extract information from file.txt into vector of QPolygonF objects
            while(!in.atEnd())
            {
                QString line = in.readLine();
                DMT_points.push_back(process_line(line, true));
            }
        }
        file.close();
    }
    paint_data(DMT_points);
    minmax = Algorithms::minmaxZ(DMT_points);
}

QPoint3D Widget::process_line(QString &line, bool option)
{
    QString x_cor, y_cor, z_cor;

    QStringList list = line.split(" ",QString::SkipEmptyParts);

    if(list.size() == 4 && option)
    {
        x_cor = list.at(1);
        y_cor = list.at(2);
        z_cor = list.at(3);
        return QPoint3D(x_cor.toDouble()*10, y_cor.toDouble()*10, z_cor.toDouble());
    }
    else
    {
        x_cor = list.at(1);
        y_cor = list.at(2);
        return QPoint3D(x_cor.toDouble()*10, y_cor.toDouble()*10, 0);
    }
}

void Widget::paint_data(std::vector<QPoint3D> &polygon)
{
    for(auto &point:polygon)
    {
        scene->addEllipse(point.getX(), point.getY(), 4, 4, QPen(Qt::red), QBrush(Qt::blue));
    }
}

void Widget::paint_mask()
{
    for(size_t i=0; i < mask.size() -1; ++i)
    {
        scene->addLine(mask.at(i).getX(),mask.at(i).getY(),mask.at(i+1).getX(),mask.at(i+1).getY(), QPen(Qt::green));
    }
}

void Widget::clear_data()
{
    // Process all data structures and empty them before starting new project
    if(DMT_points.size())
        DMT_points.clear();

    if(contour.size())
        contour.clear();

    if(triangles.size())
        triangles.clear();

    if(tin.size())
        tin.clear();

    if(mask.size())
        mask.clear();

    if(exposition.size())
        exposition.clear();

    if(slope.size())
        slope.clear();
}

void Widget::process_triangles()
{
    //std::vector<Triangle>::iterator it = triangles.begin();
    std::vector<Triangle> tr;
    QPoint3D centroid;
    //Triangle *local;
    //int iter = 0;

    for(auto &triangle:triangles)
    {
        centroid = triangle.getCentroid();
        if(Algorithms::getRayPos(centroid, mask))
        {
            tr.push_back(triangle);
        }
        /*while(Algorithms::getRayPos(centroid, mask))
        {
            if(&*it == &triangles.back())
            {
                qDebug()<<"at end";
                triangles.pop_back();
                it = triangles.begin();
                continue;
            }
            else
            {
                triangles.erase(it);
                local = &*it;
                centroid = local->getCentroid();
                qDebug()<<"remove centroid: "<<centroid.getX()<<" "<<centroid.getY()<<"iterace"<<iter;
            }
            ++iter;
        }
        ++it;*/
    }

    triangles.clear();
    triangles = tr;

    if(!contour.empty())
    {
        contour.clear();
        contour = Algorithms::createContours(triangles, minmax.first, minmax.second, 1);
    }

    scene->clear();
    paint_data(DMT_points);
    paint_triangles(tr);
    paint_contour();
}

void Widget::paint_triangles(std::vector<Triangle> &tr)
{
    for(auto &triangle:tr)
    {
        scene->addLine(triangle.getP1().getX(),triangle.getP1().getY(),triangle.getP2().getX(),triangle.getP2().getY(),QPen(Qt::magenta));
        scene->addLine(triangle.getP2().getX(),triangle.getP2().getY(),triangle.getP3().getX(),triangle.getP3().getY(),QPen(Qt::magenta));
        scene->addLine(triangle.getP3().getX(),triangle.getP3().getY(),triangle.getP1().getX(),triangle.getP1().getY(),QPen(Qt::magenta));
    }
}

void Widget::on_dmt_clicked()
{
    if(triangles.size()) return;
    tin = Algorithms::dt(DMT_points);
    triangles = Algorithms::convertDTM(tin);
    paint_triangles(triangles);

    //Allow user to create slope and exposition
    ui->getExposition->setEnabled(true);
    ui->getSlope->setEnabled(true);
}

void Widget::paint_slope()
{
    std::sort(triangles.begin(), triangles.end(), sortBySlope());
    std::vector<int> position;
    QPolygonF polygon;
    QColor color(0,250,250);
    int slope_class = 1;

    // -----------------------------------------------------------------------------------------------------------------------------------------//
    // ZADAVANI KATEGORII NA KOLIK SE SLOPE ROZDELI NECHAT NA UZIVATELI - S PREDDEFINOVANYM DEFAULTEM - NAVIC UPRAVIT ZVYSOVANI RGB HODNOT BARVY//
    // -----------------------------------------------------------------------------------------------------------------------------------------//
    position = Triangle::classifySlope(triangles, 8);
    Triangle::classifyArea(triangles, position);

    for(size_t i = 0; i < triangles.size(); ++i)
    {
        polygon << QPointF(triangles.at(i).getP1().getX(),triangles.at(i).getP1().getY()) << QPointF(triangles.at(i).getP2().getX(), triangles.at(i).getP2().getY())
                << QPointF(triangles.at(i).getP3().getX(), triangles.at(i).getP3().getY());
        if(triangles.at(i).getClassCat() == slope_class)
        {
            scene->addPolygon(polygon, QPen(Qt::red), QBrush(color));
        }
        else
        {
            if(slope_class != position.size() - 1)
            {
                color.setBlue(color.blue()-25);
                color.setGreen(color.green()-25);
                ++slope_class;
                scene->addPolygon(polygon, QPen(Qt::red), QBrush(color));
            }
            else
            {
                scene->addPolygon(polygon, QPen(Qt::red), QBrush(color));
            }
        }
        polygon.clear();
    }
}

void Widget::paint_contour()
{
    for(auto &edge:contour)
    {
        if((int)edge.getEnd().getZ()%5 == 0)
        {
            scene->addLine(edge.getStart().getX(), edge.getStart().getY(), edge.getEnd().getX(), edge.getEnd().getY(), QPen(QColor(160,90,30), 3));
        }
        else
        {
            scene->addLine(edge.getStart().getX(), edge.getStart().getY(), edge.getEnd().getX(), edge.getEnd().getY(), QPen(QColor(160,90,30)));
        }
    }
}

void Widget::paint_exposition()
{
    std::sort(triangles.begin(), triangles.end(), SortByExposition());
    std::vector<int> classify = {60, 120, 180, 240, 300, 361};
    QPolygonF polygon;
    QColor color(42, 42, 0);

    for(size_t i = 0; i < triangles.size(); ++i)
    {
        polygon << QPointF(triangles.at(i).getP1().getX(),triangles.at(i).getP1().getY()) << QPointF(triangles.at(i).getP2().getX(), triangles.at(i).getP2().getY())
                << QPointF(triangles.at(i).getP3().getX(), triangles.at(i).getP3().getY());
        if(triangles.at(i).getExposition() < classify.at(0))
        {
            scene->addPolygon(polygon, QPen(Qt::red), QBrush(color));
        }
        else
        {
            color.setRed(color.red()+42);
            color.setGreen(color.green()+42);
            classify.erase(classify.begin());
            scene->addPolygon(polygon, QPen(Qt::red), QBrush(color));
        }
        polygon.clear();
    }
}

void Widget::clear_scene()
{
    if(scene)
        delete scene;
}

void Widget::on_createcontour_clicked()
{
    if(contour.size()) return;
    contour = Algorithms::createContours(triangles, minmax.first, minmax.second, 1);
    paint_contour();
}

void Widget::on_cutbymask_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open polygon .txt file"), "", tr("Text files(*.txt);;All Files (*)"));
    if(fileName.isEmpty())
        return;
    else {

        QFile file(fileName);
        QTextStream in(&file);

        // Check if file is open
        if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            QErrorMessage *message = new QErrorMessage(this);
            message->setWindowTitle(tr("No file found"));
            message->showMessage(tr("Failed to open the source file."));
            message->setAttribute(Qt::WA_DeleteOnClose, true);
            return;
        }
        else
        {
             // Extract information from file.txt into vector of QPolygonF objects
            while(!in.atEnd())
            {
                QString line = in.readLine();
                mask.push_back(process_line(line, false));
            }
        }
        file.close();
    }
    paint_mask();
    process_triangles();
}

void Widget::on_getExposition_clicked()
{
    for(auto &triangle:triangles)
    {
        triangle.setExposition(Algorithms::getExposition(triangle));
    }
    paint_exposition();
}

void Widget::on_getSlope_clicked()
{
    for(auto &triangle:triangles)
    {
       triangle.setSlope(Algorithms::getSlope(triangle));
    }
    paint_slope();
}

void Widget::on_generateShape_clicked()
{
    clear_scene();
    clear_data();
    scene = new QGraphicsScene(this);
    ui->canvas->setScene(scene);
    ui->pointNum->setEnabled(true);
    ui->shapes->setEnabled(true);
    int choice = ui->shapes->currentIndex();
    int points = ui->pointNum->text().toInt();

    if(!points || points <= 500)
    {
        QErrorMessage *er_message = new QErrorMessage();
        er_message->setModal(true);
        er_message->setWindowTitle(tr("Size of shape incorrect"));
        er_message->showMessage(tr("Input must be u_int and bigger then 500"));
        er_message->setAttribute(Qt::WA_DeleteOnClose, true);

        ui->pointNum->clear();
    }

    switch(choice)
    {
        case 0:{
                DMT_points = Algorithms::generate_Cumulus(points);
                break;
                }
        case 1:{
                DMT_points = Algorithms::generate_Hillrest(points);
                break;
                }
        case 2:{
                DMT_points = Algorithms::generate_Valley(points);
                break;
                }
        case 3:{
                DMT_points = Algorithms::generate_Ridge(points);
                break;
                }
    }

    paint_data(DMT_points);
    minmax = Algorithms::minmaxZ(DMT_points);
}
