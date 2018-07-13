#include <list>
#include <algorithm>
#include "algorithms.h"
#include "sortbyxasc.h"
#include <stack>
#include "sortbyyasc.h"

Algorithms::Algorithms()
{

}

int Algorithms::getPosition(QPoint3D &q,QPoint3D &a, QPoint3D &b)
{
    double eps = 1.0e-6;
    double ux = b.getX() - a.getX();
    double uy = b.getY() - a.getY();
    double vx = q.getX() - a.getX();
    double vy = q.getY() - a.getY();

    double det = (ux * vy - uy * vx);
    if(det>eps)
        return 1;

    if(det<-eps)
        return 0;

    return -1;
}

int Algorithms::getPosition(QPoint &q,QPoint &a, QPoint &b)
{
    double eps = 1.0e-6;
    double ux = b.x() - a.x();
    double uy = b.y() - a.y();
    double vx = q.x() - a.x();
    double vy = q.y() - a.y();

    double det = (ux * vy - uy * vx);
    if(det>eps)
        return 1;

    if(det<-eps)
        return 0;

    return -1;
}

double Algorithms::getCircleRadius(QPoint3D &p1, QPoint3D &p2,QPoint3D &p3)
{
    //Return circle radius
    double x1 = p1.getX();
    double y1 = p1.getY();
    double x2 = p2.getX();
    double y2 = p2.getY();
    double x3 = p3.getX();
    double y3 = p3.getY();

    //Compute k1 - k12
    double k1 = x1 * x1 + y1 * y1;
    double k2 = x2 * x2 + y2 * y2;
    double k3 = x3 * x3 + y3 * y3;
    double k4 = y1 - y2;
    double k5 = y1 - y3;
    double k6 = y2 - y3;
    double k7 = x1 - x2;
    double k8 = x1 - x3;
    double k9 = x2 - x3;
    double k10 = x1 * x1;
    double k11 = x2 * x2;
    double k12 = x3 * x3;

    //Mid-point of the circle
    double m = 0.5 * (k12 * (-k4) + k11 * k5 - (k10 + k4 * k5) * k6)/
                     (x3 * (-k4) + x2 * k5 + x1 * (-k6));
    double n = 0.5 * ((k1 * (-k9) + k2 * k8 + k3 * (-k7))/
                      (y1 * (-k9) + y2 * k8 + y3 * (-k7)));

    //Radius
    return sqrt((x1 - m) * (x1 - m) + (y1 - n) * (y1 - n));

}

double Algorithms::getAngle(QPoint3D &p1, QPoint3D &p2, QPoint3D &p3, QPoint3D &p4)
{
    double ux = p2.getX() - p1.getX();
    double uy = p2.getY() - p1.getY();
    double vx = p4.getX() - p3.getX();
    double vy = p4.getY() - p3.getY();

    double s = ux * vx + uy * vy;
    double normU = sqrt(ux * ux + uy * uy);
    double normV = sqrt(vx * vx + vy * vy);

    return std::abs(acos(s/(normU * normV)))*(180/M_PI);
}

double Algorithms::getAngle(QPoint &p1, QPoint &p2, QPoint &p3, QPoint &p4)
{
    double ux = p2.x() - p1.x();
    double uy = p2.y() - p1.y();
    double vx = p4.x() - p3.x();
    double vy = p4.y() - p3.y();

    double s = ux * vx + uy * vy;
    double normU = sqrt(ux * ux + uy * uy);
    double normV = sqrt(vx * vx + vy * vy);

    return std::abs(acos(s/(normU * normV)))*(180/M_PI);
}

int Algorithms::getDelaunayPoint(Edge &e, std::vector <QPoint3D> &points)
{
    QPoint3D p1 = e.getStart();
    QPoint3D p2 = e.getEnd();
    int i_minangle = -1;
    double a_max = 0;

    for(size_t i = 0 ; i < points.size() ; i++)
    {
        int test = getPosition(points[i], p1, p2);
        if( test > 0){
            double angle = getAngle(points[i], p1, points[i], p2);
            if(angle > a_max)
            {
                a_max = angle;
                i_minangle = i;
            }
        }
    }
    return i_minangle;
}

int Algorithms::getNearestPoint(QPoint3D &p, std::vector <QPoint3D> &points)
{
    int i_min = -1;
    double d_min = 1e9;

    for(size_t i = 0; i < points.size(); i++)
    {
        double dy =points[i].getY() - p.getY();
        double dx =points[i].getX() - p.getX();

        if(fabs(dy) > 0  || fabs(dx) > 0)
        {
            double d=sqrt(dy*dy+dx*dx);

            if (d<d_min)
            {
                i_min=i;
                d_min=d;
            }
        }
    }
    return i_min;
}

std::vector<Edge> Algorithms::dt(std::vector <QPoint3D> &points)
{
    std::list<Edge> ael;
    std::vector<Edge> dt;

    //Sort points according to x coordinate
    std::sort(points.begin(),points.end(),sortByXAsc());

    //Find the nearest point
    QPoint3D p1 = points[0];
    int i_nearest = getNearestPoint(p1,points);

    //Nearest point has been found?
    if(i_nearest != -1)
    {
        QPoint3D p2 = points[i_nearest];
        Edge e(p1, p2);

        //Get delaunay point
        int i_dt = getDelaunayPoint(e, points);

        //Delaunay point has been found
        if( i_dt != -1)
        {
            //Create triangle edges
            QPoint3D pi = points[i_dt];
            Edge e2(e.getEnd(),pi);
            Edge e3(pi,e.getStart());

            //Add edges to ael
            ael.push_back(e);
            ael.push_back(e2);
            ael.push_back(e3);

            //Add edges to dt
            dt.push_back(e);
            dt.push_back(e2);
            dt.push_back(e3);
        }

        //Delaunay point has not been found, switch orientation
        else{

            e.switchOrientation();
            i_dt = getDelaunayPoint(e, points);
            if( i_dt != -1)
            {
                //Create triangle edges
                QPoint3D pi = points[i_dt];
                Edge e2(e.getEnd(),pi);
                Edge e3(pi,e.getStart());

                //Addsedges to ael
                ael.push_back(e);
                ael.push_back(e2);
                ael.push_back(e3);

                //Add e3dges to dt
                dt.push_back(e);
                dt.push_back(e2);
                dt.push_back(e3);
            }

            //Probably colinear points
            else
                return dt;
        }
    }

    //Process list of active edges until the list is empty
    while(!ael.empty())
    {
        //Get first edge
       Edge e = ael.back();
       ael.pop_back();
       e.switchOrientation();

       int i_dt = getDelaunayPoint(e,points);

       //Delaunay point has been found
       if (i_dt != -1)
       {
           //Create remaining edges
           Edge e2(e.getEnd(),points[i_dt]);
           Edge e3(points[i_dt],e.getStart());

           //Add triangle to DT
           dt.push_back(e);
           dt.push_back(e2);
           dt.push_back(e3);

           e2.switchOrientation();
           e3.switchOrientation();

           //Find edge with the opposite orientation
           std::list<Edge>::iterator i_e2 = Edge::find_pos(ael.begin(),ael.end(),e2);

           //Edge e2 has not been found
           if(i_e2 == ael.end())
           {
               e2.switchOrientation();
               ael.push_back(e2);
           }
           //Edge e2 has been found
           else {
               ael.erase(i_e2);
           }

           std::list<Edge>::iterator i_e3 = Edge::find_pos(ael.begin(),ael.end(),e3);

           //Edge e3 has not been found
           if(i_e3 == ael.end())
           {
               e3.switchOrientation();
               ael.push_back(e3);
           }
           //Edge e3 has been found
           else {
               ael.erase(i_e3);
           }
       }
    }
    return dt;
}

QPoint3D Algorithms::getConPoint(QPoint3D &p1, QPoint3D &p2, double z)
{
    double x1 = p1.getX();
    double x2 = p2.getX();
    double y1 = p1.getY();
    double y2 = p2.getY();
    double z1 = p1.getZ();
    double z2 = p2.getZ();

    double xb = ((x2 - x1)/(z2 - z1))*(z-z1) + x1;
    double yb = ((y2 - y1)/(z2 - z1))*(z-z1) + y1;

    QPoint3D c(xb,yb,z);

    return c;
}

bool Algorithms::compare(Edge &a, Edge &b)
{
    //Check if two edges are identical
    double eps = 1e-5;

    if((std::abs(a.getEnd().getX() - b.getEnd().getX())<eps) && (std::abs(a.getEnd().getY() - b.getEnd().getY())<eps) &&
            (std::abs(a.getStart().getX()-b.getStart().getX())<eps) && (std::abs(a.getStart().getY()-b.getStart().getY())<eps))
        return true;

    return false;
}

std::vector<Edge> Algorithms::createContours(std::vector<Triangle> &dt, double zmin, double zmax, double h)
{
    std::vector<Edge> contours;
    double eps = 5e-4;

    for(size_t i = 0; i < dt.size(); ++i)
    {
        //Get triangle vertices
        QPoint3D p1=dt[i].getP1();
        QPoint3D p2=dt[i].getP2();
        QPoint3D p3=dt[i].getP3();

        //Create all contour lines for given triangle
        for(double z = zmin; z <= zmax; z+=h)
        {
            //Get height differences between points and plane
            double dz1 = p1.getZ() - z;
            double dz2 = p2.getZ() - z;
            double dz3 = p3.getZ() - z;

            //Points in the plane?
            bool b1 = (fabs(dz1)<eps);
            bool b2 = (fabs(dz2)<eps);
            bool b3 = (fabs(dz3)<eps);

            //Edges in the plane
            bool b12 = b1 && b2;
            bool b23 = b2 && b3;
            bool b31 = b3 && b1;

            //Edges intersected by the plane
            bool bi12 = dz1 * dz2 <0;
            bool bi23 = dz2 * dz3 <0;
            bool bi31 = dz3 * dz1 <0;

            // case 1 - triangle is coplanar
            if (b12 && b23 )
                continue;

            // case 2 -  triangle edge colinear
            else if(b12 || b23 || b31){
                //First triangle edge is colinear
                if(b12)
                {
                    contours.push_back(Edge(p1,p2));
                }

                //Second triangle edge is colinear
                else if(b23)
                {
                    contours.push_back(Edge(p2,p3));
                }

                //Third triangle edge is colinear
                else
                {
                    contours.push_back(Edge(p3,p1));
                }

            }

            //case 3 - onlz one point lies on plane
            else if(b1||b2||b3)
                continue;

            // case 4 - contour line passing through a point and intersecting edge
            else if(b1 && bi23 || b2 && bi31  || b3 && bi12){
                //p1 x (p2,p3)
                if(b1 && bi23)
                {
                    QPoint3D i23 = getConPoint(p2,p3,z);
                    contours.push_back(Edge(p1,i23));
                }
                //p2 x (p3,p1)
                else if(b2 && bi31)
                {
                    QPoint3D i31 = getConPoint(p3,p1,z);
                    contours.push_back(Edge(p2,i31));
                }
                //p3 x (p1,p2)
                else if(b3 && bi12)
                {
                    QPoint3D i12 = getConPoint(p1,p2,z);
                    contours.push_back(Edge(p3,i12));
                }
            }

            // case 5 - contour line intersects both edges
            else if(bi12 && bi31 || bi23 && bi12 || bi23 && bi31)
            {
                    //(p1,p2) x (p3,p1)
                    if(bi12 && bi31)
                    {
                        QPoint3D i12 = getConPoint(p1,p2,z);
                        QPoint3D i31 = getConPoint(p3,p1,z);

                        contours.push_back(Edge(i12,i31));
                    }

                    //(p2,p3) x (p1,p2)
                    else if(bi23 && bi12)
                    {
                        QPoint3D i23 = getConPoint(p2,p3,z);
                        QPoint3D i12 = getConPoint(p1,p2,z);

                        contours.push_back(Edge(i23,i12));
                    }

                    //(p2,p3) x (p3,p1)
                    else if(bi23 && bi31)
                    {
                        QPoint3D i23 = getConPoint(p2,p3,z);
                        QPoint3D i31 = getConPoint(p3,p1,z);

                        contours.push_back(Edge(i23,i31));
                    }
                }
            }
        }
    return contours;
}

std::vector<Triangle> Algorithms::convertDTM(std::vector<Edge>&dt)
{
    std::vector<Triangle> dtt;

    for(size_t i = 0; i < dt.size()-2; i += 3 ){
        QPoint3D p1 = dt[i].getStart();
        QPoint3D p2 = dt[i].getEnd();
        QPoint3D p3 = dt[i+1].getEnd();

        QPoint3D centroid = Triangle::computeCentroid(dt[i],dt[i+1]);

        Triangle t(p1,p2,p3,0,0,centroid);
        dtt.push_back(t);
    }
    return dtt;
}

double Algorithms::getSlope(Triangle &tr)
{
    QPoint3D p1 = tr.getP1();
    QPoint3D p2 = tr.getP2();
    QPoint3D p3 = tr.getP3();

    double ux = p1.getX() - p2.getX();
    double uy = p1.getY() - p2.getY();
    double uz = p1.getZ() - p2.getZ();
    double vx = p3.getX() - p2.getX();
    double vy = p3.getY() - p2.getY();
    double vz = p3.getZ() - p2.getZ();

    double nx = uy*vz-vy*uz;
    double ny = -(ux*vz-vx*uz);
    double nz = ux*vy-vx*uy;

    return acos(fabs(nz)/ sqrt(nx*nx + ny*ny + nz*nz)) * 180/ atan (1);

}

double Algorithms::getExposition(Triangle &tr)
{
    QPoint3D p1 = tr.getP1();
    QPoint3D p2 = tr.getP2();
    QPoint3D p3 = tr.getP3();
    double pi = 3.14159265359;

    double ux = p1.getX() - p2.getX();
    double uy = p1.getY() - p2.getY();
    double uz = p1.getZ() - p2.getZ();
    double vx = p3.getX() - p2.getX();
    double vy = p3.getY() - p2.getY();
    double vz = p3.getZ() - p2.getZ();

    double nx = uy*vz-vy*uz;
    double ny = -(ux*vz-vx*uz);

    double result = atan(ny/nx)* 180/ pi;

    if(ny > 0 && nx < 0)
    {
        result += 180;
    }
    else if(ny < 0 && nx < 0)
    {
        result -= 180;
    }
    else if(nx == 0 && ny < 0)
    {
        result = -90;
    }
    else if(nx == 0 && ny > 0)
    {
        result = 90;
    }

    if(result < 0)
    {
        result+=360;
    }

    return result;
}

std::pair<double, double> Algorithms::minmaxZ(std::vector<QPoint3D> &vector)
{
    double minZ=1e9;
    double maxZ=0;
    std::pair<double, double> minmax;

    for(auto &point:vector)
    {
        if(point.getZ() < minZ)
            minZ = point.getZ();

        if(point.getZ() > maxZ)
            maxZ = point.getZ();
    }

    minmax.first = std::ceil(minZ);
    minmax.second = std::floor(maxZ);
    return minmax;
}

int Algorithms::getRayPos(QPoint3D &q, std::vector<QPoint3D> &pol, bool closed){

    int k = 0;

    //Add first point as the last one
    if(!closed)
    {
        pol.push_back(pol[0]);
    }

    //Process all polygon segments
    for (size_t i = 0 ; i < pol.size() - 1 ; i++)
    {
        //Point lies on polygon point
        if((q.getX() == pol[i].getX()) && (q.getY() == pol[i].getY()))
            return 1;

        //Get angle
        double om_l = getAngle(pol[i], q, pol[i+1], q);

        //Point lies on polygon side
        if(om_l == 180)
            return 1;

        //Get reduced points of the line segment
        double xi = pol[i].getX()-q.getX();
        double yi = pol[i].getY()-q.getY();
        double xii= pol[i+1].getX()-q.getX();
        double yii= pol[i+1].getY()-q.getY();

        //A suitable line segment
        if( (yi <= 0) && (yii > 0) || (yii <= 0) && (yi > 0) )
        {
            //Point inn the right halfplane
            double xn = (xii * yi - xi * yii) / (yii - yi);
            if (xn > 0)
                k++;
        }
    }
    return k%2;
}

std::vector<QPoint3D> Algorithms::generate_Cumulus(uint n)
{
    std::vector<QPoint3D> Points;

    //scale of terrain
    double XYscale = 1e5;
    int Hscale = 200;

    for(uint i = 0 ; i < n ; i++ )
    {
        // generate radom point;
        double X = 2 * ((double) rand() / (RAND_MAX)) - 1;
        double Y = 2 * ((double) rand() / (RAND_MAX)) - 1;

        // calculate coords of point Pi
        double px = XYscale * X;
        double py = XYscale * Y;
        double pz = Hscale * exp( -(X * X + Y * Y) ) + ((double) rand() / (RAND_MAX));

        // add point into Points
        QPoint3D P(px,py,pz);
        Points.push_back(P);
    }
    return Points;
}

std::vector<QPoint3D> Algorithms::generate_Hillrest(uint n)
{
    std::vector<QPoint3D> Points;

    // scale of terrain
    double XYscale = 1e5;
    double Hscale = 200;

    // random size and rotation
    double w  = ((double) rand() / (RAND_MAX)) * M_PI;

    //to save time in a FOR-loop
    double SIN = sin(w);
    double COS = cos(w);


    for(uint i = 0 ; i < n ; i++)
    {
        //generate radom point;
        double x = 2*((double) rand() / (RAND_MAX)) - 1;
        double y = 2*((double) rand() / (RAND_MAX)) - 1;

        // add random rotation
        double X = COS * x - SIN * y;
        double Y = SIN * x + COS * y;

        // course of the terrain in X and Y axis
        double fx = exp(-x*x);
        double fy = y + cos(M_PI*y + 1)/M_PI + 1.5;

        // calculate coords of point Pi
        double px = XYscale * X;
        double py = XYscale * Y;
        double pz = (Hscale * (fx * fy) + ((double) rand() / (RAND_MAX)));

        // add point into Points
        QPoint3D P(px,py,pz);
        Points.push_back(P);
    }
    return Points;
}

std::vector<QPoint3D> Algorithms::generate_Valley(uint n)
{
    std::vector<QPoint3D> Points;

    // scale of terrain
    double XYscale = 1e5;
    double Hscale = 200;

    // random size and rotation
    double w  = ((double) rand() / (RAND_MAX)) * M_PI;

    //to save time in a FOR-loop
    double SIN = sin(w);
    double COS = cos(w);

     for(uint i = 0 ; i < n ; i++)
     {
        // generate radom point;
        double x = 2 * ((double) rand() / (RAND_MAX)) - 1;
        double y = 2 * ((double) rand() / (RAND_MAX)) - 1;

        // add random rotation
        double X = COS * x - SIN * y;
        double Y = SIN * x + COS * y;

        // course of the terrain in X and Y axis
        double fx =-exp(-x * x);
        double fy = 1 - y / 20;

        // calculate coords of point Pi
        double px = XYscale * X;
        double py = XYscale * Y;
        double pz = (500 + Hscale * (fx * fy) +((double) rand() / (RAND_MAX)));

        // add point into Points
        QPoint3D P(px,py,pz);
        Points.push_back(P);

     }
     return Points;
}

std::vector<QPoint3D> Algorithms::generate_Ridge(uint n)
{
    std::vector<QPoint3D> Points;

    // scale of terrain
    double XYscale = 1e5;
    double Hscale = 200;

    // random size and rotation
    double w  = ((double) rand() / (RAND_MAX)) * M_PI;

    //to save time in a FOR-loop
    double SIN = sin(w);
    double COS = cos(w);

    for(uint i = 0 ; i < n ; i++ )
    {
        // generate radom point;
        double x = 2 * ((double) rand() / (RAND_MAX)) - 1;
        double y = 2 * ((double) rand() / (RAND_MAX)) - 1;

        // add random rotation
        double X = COS * x - SIN * y;
        double Y = SIN * x + COS * y;

        // course of the terrain in X axis
        double fx = exp(-X * X / 4);

        // course of the terrain in X axis
        double YY = Y;
        if(Y < 0)
            YY -= 0.5;
        else
            YY += 0.5;

        double fy = exp(-YY * YY);

        // calculate coords of point Pi
        double px = XYscale * X;
        double py = XYscale * Y;
        double pz = (500 + Hscale * (fx * fy)) + ((double) rand() / (RAND_MAX));

        // add point into Points
        QPoint3D P(px,py,pz);
        Points.push_back(P);
     }
     return Points;
}
