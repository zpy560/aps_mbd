#include "bezier_three.hpp"

// // 计算两点距离
// double distance_bizer(Point2D a, const Point2D& b) {
//     double dx = b.x - a.x;
//     double dy = b.y - a.y;
//     return std::sqrt(dx*dx + dy*dy);
// }

Point3D bezier3(const std::vector<Point3D>& ctrl_pts, double t) {
    assert(ctrl_pts.size() == 4);
    double u = 1 - t;
    double b0 = u * u * u;
    double b1 = 3 * u * u * t;
    double b2 = 3 * u * t * t;
    double b3 = t * t * t;

    Point3D p;
    p.x = b0 * ctrl_pts[0].x + b1 * ctrl_pts[1].x + b2 * ctrl_pts[2].x + b3 * ctrl_pts[3].x;
    p.y = b0 * ctrl_pts[0].y + b1 * ctrl_pts[1].y + b2 * ctrl_pts[2].y + b3 * ctrl_pts[3].y;
    return p;
}

// 计算组合数C(n,k)
unsigned int comb(unsigned int n, unsigned int k) {
  if (k == 0 || k == n) return 1;
  if (k > n) return 0;
  unsigned int res = 1;
  for (size_t i = 1; i <= k; ++i) {
    res = res * (n - k + i) / i;
  }
  return res;
}

// 5阶Bernstein基函数
double bernstein(int i, int n, double t) {
    unsigned int c = comb(n, i);
    return c * std::pow(t, i) * std::pow(1 - t, n - i);
}

// 计算5阶Bezier曲线点
Point3D bezier5(const std::vector<Point3D>& control_points, double t) {
    int n = 5;
    assert(int(control_points.size()) == n + 1);
    Point3D p{0.0, 0.0,0.0};
    for (int i = 0; i <= n; ++i) {
        double b = bernstein(i, n, t);
        p.x += b * control_points[i].x;
        p.y += b * control_points[i].y;
    }
    return p;
}

std::vector<double> computeArcLengths(const std::vector<Point3D>& pts) {
    std::vector<double> arc_len(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
        arc_len[i] = arc_len[i - 1] + (pts[i] - pts[i - 1]).norm();
    }
    return arc_len;
}

// 在arc_lengths中找到对应s_target的参数t，线性插值
double findTfromArcLength(const std::vector<double>& arc_lengths, const std::vector<double>& ts, double s_target) {
    if (s_target <= arc_lengths.front()) return ts.front();
    if (s_target >= arc_lengths.back()) return ts.back();

    // 二分查找区间
    size_t low = 0, high = arc_lengths.size() - 1;
    while (low <= high) {
        size_t mid = low + (high - low) / 2;
        if (arc_lengths[mid] < s_target) {
            low = mid + 1;
        } else {
            if (mid == 0 || arc_lengths[mid - 1] < s_target) {
                size_t i = mid - 1;
                double s0 = arc_lengths[i];
                double s1 = arc_lengths[i + 1];
                double t0 = ts[i];
                double t1 = ts[i + 1];
                double t = t0 + (s_target - s0) / (s1 - s0) * (t1 - t0);
                return t;
            }
            high = mid - 1;
        }
    }
    return ts.back();
}

std::vector<Point3D> generateBezierUniformPoints(const std::vector<Point3D>& ctrl_pts, double interval) {
    assert(ctrl_pts.size() == 4); // 3次Bezier曲线

    // 高密度采样近似曲线，用于计算弧长
    const int dense_num = 1000;
    std::vector<Point3D> dense_points;
    std::vector<double> ts;
    dense_points.reserve(dense_num);
    ts.reserve(dense_num);

    for (int i = 0; i < dense_num; ++i) {
        double t = double(i) / (dense_num - 1);
        ts.push_back(t);
        dense_points.push_back(bezier3(ctrl_pts, t));
    }

    auto arc_lengths = computeArcLengths(dense_points);
    double total_len = arc_lengths.back();

    int sample_count = static_cast<int>(std::floor(total_len / interval)) + 1;
    std::vector<Point3D> uniform_points;
    uniform_points.reserve(sample_count + 1);

    for (int i = 0; i < sample_count; ++i) {
        double s = i * interval;
        if (s > total_len) s = total_len; // 防止越界
        double t = findTfromArcLength(arc_lengths, ts, s);
        uniform_points.push_back(bezier3(ctrl_pts, t));
    }

    // 确保最后一个点就是终点
    if ((uniform_points.back().x != ctrl_pts.back().x) || (uniform_points.back().y != ctrl_pts.back().y)) {
        uniform_points.push_back(ctrl_pts.back());
    }

    return uniform_points;
}

// 主函数：6个点生成均匀间隔5阶Bezier曲线采样点
std::vector<Point3D> generateUniformBezier5Curve(const std::vector<Point3D>& ctrl_pts, double interval)
{
    assert(ctrl_pts.size() == 6); // 5次Bezier曲线

    std::vector<Point3D> xs_ys;
    Point3D xy_controls;
    for (int i = 0; i < 6; ++i) {
        xy_controls.x = ctrl_pts[i].x;
        xy_controls.y = ctrl_pts[i].y;
        xy_controls.speed = ctrl_pts[i].speed;
        xs_ys.push_back(xy_controls);
    }

    const int dense_num = 2000; // 用于弧长计算的高密度采样点
    std::vector<Point3D> dense_pts;
    std::vector<double> ts;
    dense_pts.reserve(dense_num);
    ts.reserve(dense_num);

    for (int i = 0; i < dense_num; ++i) {
        double t = double(i) / (dense_num - 1);
        ts.push_back(t);
        dense_pts.push_back({bezier5(xs_ys, t)});
    }

    auto arc_lengths = computeArcLengths(dense_pts);
    double total_len = arc_lengths.back();

    int sample_num = static_cast<int>(std::floor(total_len / interval)) + 1;
    std::vector<Point3D> uniform_points;
    uniform_points.reserve(sample_num + 1);

    for (int i = 0; i < sample_num; ++i) {
        double s = i * interval;
        if (s > total_len) s = total_len;
        double t = findTfromArcLength(arc_lengths, ts, s);

        Point3D pt{bezier5(xs_ys,t)};
        uniform_points.push_back(pt);
        std::cout<<"bizer  sample_num="<<i<<"  "<< "pt.x:"<< pt.x<<"pt.y:"<< pt.y<<std::endl;
    }

    // 保证包含终点
    if ((uniform_points.back().x != ctrl_pts.back().x) ||
        (uniform_points.back().y != ctrl_pts.back().y)) {
        uniform_points.push_back(ctrl_pts.back());
    }

    return uniform_points;
}


// 生成两点间固定间隔采样点，包含起点和终点
std::vector<Point3D> generateFixedStepPoints(const Point3D &p0,
    const Point3D &p1,double step)
{
    std::vector<Point3D> points;

    double dist = std::sqrt(distanceSquared(p0, p1));
    if (dist < 1e-6)
    {
        // 起点终点重合，返回起点
        points.push_back(p0);
        // RCLCPP_INFO( "start and end points are the same");
        return points;
    }

    // 计算采样点数，向下取整保证间隔不大于step
    int num_intervals = static_cast<int>(std::floor(dist / step));
    int num_points = num_intervals + 1;

    // 方向向量归一化
    double dx = (p1.x - p0.x) / dist;
    double dy = (p1.y - p0.y) / dist;

    points.reserve(num_points + 1);
    // 采样起点和中间点
    for (int i = 0; i < num_points; ++i)
    {
        Point3D p;
        p.x = p0.x + dx * step * i;
        p.y = p0.y + dy * step * i;
        std::cout<<"num_points="<<i<<"  "<< "p.x:"<< p.x<<"p.y:"<< p.y<<std::endl;
        // RCLCPP_ERROR(get_logger(), "num_points=%d, p.x=%lf, p.y=%lf", i, p.x, p.y);
        points.push_back(p);
    }

    return points;
}

std::vector<geometry_msgs::msg::Point> update_trajectory(const std::vector<Point3D> &points, double speed)
{
    std::vector<geometry_msgs::msg::Point> traj;
    geometry_msgs::msg::Point pt;
    for (const auto& p : points)
    {
        pt.x = p.x;
        pt.y = p.y;
        pt.z = speed;
        traj.push_back(pt);
    }
    return traj;
}


