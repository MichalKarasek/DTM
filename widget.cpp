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
#include "inputform.h"

Widget::Widget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Widget)
{
    ui->setupUi(this);
    setWindowTitle(tr("DMT generator 1.0"));
    scene = new QGraphicsScene(this);
    ui->canvas->setScene(scene);
    ui->openfile->setText(tr("Open file"));
    ui->getExposition->setText(tr("Exposition"));
    ui->getSlope->setText(tr("Slope"));
    ui->createcontour->setText(tr("Create contours"));
    ui->cutbymask->setText(tr("Cut DMT by mask"));
    ui->dmt->setText(tr("Create DMT"));
    ui->generateShape->setText(tr("Generate shape"));
    ui->pointNum->setToolTip(tr("Number of points to gengerate shape from"));

    ui->getExposition->setEnabled(false);
    ui->getSlope->setEnabled(false);
    ui->createcontour->setEnabled(false);
    ui->cutbymask->setEnabled(false);
    ui->dmt->setEnabled(false);
}

Widget::~Widget()
{
    delete scene;
    delete ui;
}

void Widget::on_openfile_clicked()
{
    clearScene();
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open polygon .txt file"), "", tr("Text files(*.txt);;All Files (*)"));

    if(fileName.isEmpty())
    {
        return;
    }
    else
    {
        QFile file(fileName);
        QTextStream in(&file);

        clearData();

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
                DMT_points.push_back(processLine(line, true));
            }
        }
        file.close();
    }
    paintData(DMT_points);
    minmax = Algorithms::minmaxZ(DMT_points);
    ui->dmt->setEnabled(true);
}

QPoint3D Widget::processLine(QString &line, bool option)
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

void Widget::paintData(std::vector<QPoint3D> &polygon)
{
    for(auto &point:polygon)
    {
        scene->addEllipse(point.getX(), point.getY(), 4, 4, QPen(Qt::red), QBrush(Qt::blue));
    }
}

void Widget::paintMask()
{
    for(size_t i=0; i < mask.size() -1; ++i)
    {
        scene->addLine(mask.at(i).getX(),mask.at(i).getY(),mask.at(i+1).getX(),mask.at(i+1).getY(), QPen(Qt::green));
    }
}

void Widget::clearData()
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

void Widget::processTriangles()
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
    paintData(DMT_points);
    paintTriangles(tr);
    paintContour();
}

void Widget::paintTriangles(std::vector<Triangle> &tr)
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
    paintTriangles(triangles);

    //Allow user to create slope and exposition
    ui->getExposition->setEnabled(true);
    ui->getSlope->setEnabled(true);
    ui->createcontour->setEnabled(true);
    ui->cutbymask->setEnabled(true);
}

void Widget::paintSlope()
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
    if(contour.size()) paintContour();
}

void Widget::paintContour(const int highlight)
{
    for(auto &edge:contour)
    {
        if((int)edge.getEnd().getZ()%highlight == 0)
        {
            scene->addLine(edge.getStart().getX(), edge.getStart().getY(), edge.getEnd().getX(), edge.getEnd().getY(), QPen(QColor(160,90,30), 3));
        }
        else
        {
            scene->addLine(edge.getStart().getX(), edge.getStart().getY(), edge.getEnd().getX(), edge.getEnd().getY(), QPen(QColor(160,90,30)));
        }
    }
}

void Widget::paintExposition()
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

    if(contour.size()) paintContour();
}

void Widget::clearScene()
{
    if(scene)
    {
        scene->clear();
    }
}

void Widget::on_createcontour_clicked()
{
    if(contour.size()) return;

    inputform *inform = new inputform(this);
    int spacing {};
    int highlight {};

    if(inform->exec() == QDialog::Accepted)
    {
        spacing = inform->getSpacing();
        highlight = inform->getHighlight();
        contour = Algorithms::createContours(triangles, minmax.first, minmax.second, spacing);
        paintContour(highlight);
    }
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
                mask.push_back(processLine(line, false));
            }
        }
        file.close();
    }
    paintMask();
    processTriangles();
}

void Widget::on_getExposition_clicked()
{
    for(auto &triangle:triangles)
    {
        triangle.setExposition(Algorithms::getExposition(triangle));
    }
    paintExposition();
}

void Widget::on_getSlope_clicked()
{
    for(auto &triangle:triangles)
    {
       triangle.setSlope(Algorithms::getSlope(triangle));
    }
    paintSlope();
}

void Widget::on_generateShape_clicked()
{
    clearScene();
    clearData();
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
        ui->dmt->setEnabled(false);
        return;
    }

    switch(choice)
    {
        case 0:{
                DMT_points = Algorithms::generateCumulus(points);
                break;
                }
        case 1:{
                DMT_points = Algorithms::generateHillrest(points);
                break;
                }
        case 2:{
                DMT_points = Algorithms::generateValley(points);
                break;
                }
        case 3:{
                DMT_points = Algorithms::generateRidge(points);
                break;
                }
    }

    paintData(DMT_points);
    minmax = Algorithms::minmaxZ(DMT_points);
    ui->dmt->setEnabled(true);
}
