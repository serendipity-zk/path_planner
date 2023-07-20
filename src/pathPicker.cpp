//
// Created by zhukan on 7/21/23.
//

#include "pathPicker.h"

int HybridAStar::PathPicker::pickPath(std::vector<Path> paths, Node3D nGoal)
{
    using namespace std;
    vector<double> pathCosts;
    PathPicker::Point2D goal(nGoal.getX(), nGoal.getY());
    cout<< paths.size()<<endl;
    for (const auto &path : paths)
    {
        double pathCost = 0.0;
        nav_msgs::Path pathMsg = path.getPath();
        vector<PathPicker::Point2D> points;
        for (const auto &pose : pathMsg.poses)
        {
            points.push_back(PathPicker::Point2D(pose.pose.position.x, pose.pose.position.y));
        }
        cout << "points size: " << points.size() << endl;
        if (points.size() < 2 || (points[0] - goal).length() > 3)
        {
            pathCosts.push_back(1e10);
            continue;
        }
        double totalLength = 0.0;
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            totalLength += (points[i + 1] - points[i]).length();
        }
        pathCost += totalLength;

        vector<size_t> keyIdx;
        keyIdx.push_back(0);
        // for all points in the path, if the incoming vector and outgoing vector have a negative dot product, then it is a key point
        for (size_t i = 1; i < points.size() - 1; ++i)
        {
            PathPicker::Point2D incoming = points[i] - points[i - 1];
            PathPicker::Point2D outgoing = points[i + 1] - points[i];
            if (incoming.dotProduct(outgoing) < 0)
            {
                keyIdx.push_back(i);
            }
        }
        keyIdx.push_back(points.size() - 1);
        
        int penalty = 0;
        for (size_t i = 0; i < keyIdx.size() - 1; ++i)
        {
            penalty ++;
            double distance = (points[keyIdx[i + 1]] - points[keyIdx[i]]).length();
            if (distance < 10)
            {
                penalty ++;
            }
            if (distance < 5)
            {
                penalty ++;
            }
        }
        pathCost *= penalty;
        pathCosts.push_back(pathCost);
        cout<<"pathCost: "<<pathCost<<endl;
    }
    int minIdx = 0;
    double minCost = pathCosts[0];
    for (size_t i = 1; i < pathCosts.size(); ++i)
    {
        if (pathCosts[i] < minCost)
        {
            minCost = pathCosts[i];
            minIdx = i;
        }
    }
    cout << "minIdx: " << minIdx << endl;
    cout << "minCost: " << minCost << endl;
    return minIdx;
}
