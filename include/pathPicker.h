//
// Created by zhukan on 7/21/23.
//

#ifndef PATH_PLANNER_PATHPICKER_H
#define PATH_PLANNER_PATHPICKER_H

#include "path.h"
#include <vector>

namespace HybridAStar
{
    class PathPicker
    {

    public:
        class Point2D
        {
        public:
            double x;
            double y;

            Point2D(double x = 0.0, double y = 0.0)
                : x(x), y(y) {}

            Point2D operator+(const Point2D &other) const
            {
                return Point2D(x + other.x, y + other.y);
            }

            Point2D operator-(const Point2D &other) const
            {
                return Point2D(x - other.x, y - other.y);
            }

            Point2D operator*(double scalar) const
            {
                return Point2D(x * scalar, y * scalar);
            }

            double dotProduct(const Point2D &other) const
            {
                return x * other.x + y * other.y;
            }

            double length() const
            {
                return std::sqrt(x * x + y * y);
            }
        };
        static int pickPath(std::vector<Path> paths, Node3D nGoal);
    private:
        
    };
}

#endif // PATH_PLANNER_PATHPICKER_H
