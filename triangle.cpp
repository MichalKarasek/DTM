#include "triangle.h"
#include "edge.h"
#include "qpoint3d.h"
#include "qdebug.h"

QPoint3D Triangle::computeCentroid(Edge &a, Edge &b)
{
    QPoint3D s1((a.getStart().getX()+a.getEnd().getX())/2, (a.getStart().getY()+a.getEnd().getY())/2, 0);
    QPoint3D s2((b.getStart().getX()+b.getEnd().getX())/2, (b.getStart().getY()+b.getEnd().getY())/2, 0);

    Edge e1(s1, b.getEnd());
    Edge e2(s2, a.getStart());
    QPoint3D centr = Edge::crossEdge(e1,e2);
    return centr;
}

std::vector<int> Triangle::classifySlope(std::vector<Triangle> & triangles, const size_t n_clas)
{
    bool opt = true;
    double sum = 0;
    double avg = 0;
    int size = triangles.size();
    int i = 1;
    int first;
    int second;
    std::vector<int> pos;
    std::vector<int> pos_copy;

    while (opt) {
        if(std::abs(triangles.at(size-i).getSlope() - triangles.at(size-i-1).getSlope()) > 2)
        {
            ++i;
        }
        else
        {
            opt = false;
        }
    }

    size -= i-1;

    pos.push_back(0);
    pos.push_back(size);

    pos_copy = pos;

    while(pos.size() < n_clas + 1)
    {
        size = pos.size();
        first = pos_copy.at(0);
        i = 0;

        while(pos_copy.size() > 1)
        {
            pos_copy.erase(pos_copy.begin());
            second = pos_copy.at(0);

            for(int k = first; k < second; ++k)
            {
                sum += triangles.at(k).getSlope();
            }

            avg = sum / (second - first);

            while(triangles.at(i).getSlope() < avg)
            {
                ++i;
            }
            first = second;
            pos.push_back(i);
            i=first;
            sum = 0;
        }
        std::sort(pos.begin(), pos.end());
        pos_copy = pos;
    }
    return pos;
}

void Triangle::classifyArea(std::vector<Triangle> & triangles, std::vector<int> & classes)
{
    size_t i = 0;
    size_t avg = classes.at(0);

    for(size_t j = 0; j < triangles.size(); ++j)
    {
        if(j < avg)
        {
            triangles.at(j).setClassCat(i);
        }
        else
        {
            if(i != classes.size()-1)
            {
                ++i;
                avg = classes.at(i);
                triangles.at(j).setClassCat(i);
            }
            else
            {
                triangles.at(j).setClassCat(i);
            }
        }
    }
}
