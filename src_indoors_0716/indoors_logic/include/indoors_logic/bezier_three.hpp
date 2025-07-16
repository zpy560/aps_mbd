#include <vector>
#include <iostream>
#include <cmath>
#include <cassert>
#include "byd_custom_msgs/msg/path_points.hpp"
#include "geometry_msgs/msg/point.hpp"

struct Point3D {
    double x, y,speed;

    Point3D operator-(const Point3D& rhs) const {
        return {x - rhs.x, y - rhs.y,speed};
    }
    double norm() const {
        return std::sqrt(x * x + y * y);
    }
};
inline double distanceSquared(const Point3D &a, const Point3D &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

Point3D bezier3(const std::vector<Point3D>& ctrl_pts, double t);
Point3D bezier5(const std::vector<Point3D>& control_points, double t) ;
std::vector<double> computeArcLengths(const std::vector<Point3D>& pts);
unsigned int comb(unsigned int n, unsigned int k);
double bernstein(int i, int n, double t);
// 在arc_lengths中找到对应s_target的参数t，线性插值
double findTfromArcLength(const std::vector<double>& arc_lengths, const std::vector<double>& ts, double s_target);
std::vector<Point3D> generateUniformBezier5Curve(const std::vector<Point3D>& ctrl_pts, double interval);
std::vector<Point3D> generateBezierUniformPoints(const std::vector<Point3D>& ctrl_pts, double interval);
std::vector<geometry_msgs::msg::Point> update_trajectory(const std::vector<Point3D> &points, double speed);

// 生成两点间固定间隔采样点，包含起点和终点
std::vector<Point3D> generateFixedStepPoints(const Point3D &p0,const Point3D &p1,double step);