#include "control_node.hpp"

// 如果在cpp文件中，可以加上
using namespace std::chrono_literals;

// ...existing code...
/**
 * @brief 将角度误差归一化到 [-M_PI, M_PI]
 */
double ControlNode::normalizeAngle(double angle)
{
    if (angle > M_PI)
    {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI)
    {
        angle += 2 * M_PI;
    }
    return angle;
}
// ...existing code...

void ControlNode::save_log_to_file(const std::string& log_msg,bool is_save_to_file) {
    if (!is_save_to_file) {
        return; // 如果不需要保存到文件，直接返回
    }
    // 获取系统时间
    auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm tm_now;
    localtime_r(&now_c, &tm_now);

    // 获取ROS2时间
    rclcpp::Clock ros_clock(RCL_ROS_TIME);
    rclcpp::Time ros_now = ros_clock.now();

    // std::ofstream log_file("/home/byd-zpy/my_logicindoor/logs/control_node_info.log", std::ios::app);
    std::ofstream log_file("/home/forlinx/lidar_slam/colcon_ws/src/indoors_logic/control_node_info.log", std::ios::app);
    if (log_file.is_open()) {
        log_file << "[" << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S")
                 << "][ROS:" << std::fixed << std::setprecision(5) << ros_now.seconds() << "] "
                 << log_msg << std::endl;
    }
}

bool ControlNode::isPoseStampedValid(const geometry_msgs::msg::PoseStamped &pose_stamped)
{
    if (pose_stamped.header.stamp.sec == 0 && pose_stamped.header.stamp.nanosec == 0)
    {
        return false; // 时间戳无效
    }

    const auto &pos = pose_stamped.pose.position;
    const auto &ori = pose_stamped.pose.orientation;

    bool is_position_zero = (pos.x == 0.0 && pos.y == 0.0 && pos.z == 0.0);
    bool is_orientation_default = (ori.x == 0.0 && ori.y == 0.0 && ori.z == 0.0 && ori.w == 1.0);

    // 根据需求决定是否允许默认位姿
    if (is_position_zero && is_orientation_default)
    {
        return false;
    }

    return true;
}

double ControlNode::distanceSquared_node(const byd_custom_msgs::msg::PathPoints &a, const geometry_msgs::msg::Point &b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}
/**
 *
 * @brief 使用三分搜索在有序路径点序列中找到距离点p最近的点索引（假设距离是单峰函数）
 * @param p 查询点
 * @param path 有序的路径点序列
 * @return 最近点索引，-1表示空路径
 */
int ControlNode::ternarySearchClosestIndex(const byd_custom_msgs::msg::PathPoints &p, const byd_custom_msgs::msg::Trajectory msg)
{
    if (msg.path.empty())
        return -1;
    int left = 0;
    int right = static_cast<int>(msg.path.size()) - 1;

    while (right - left > 3)
    {
        int leftThird = left + (right - left) / 3;
        int rightThird = right - (right - left) / 3;

        double distLeft = distanceSquared_node(p, msg.path[leftThird]);
        double distRight = distanceSquared_node(p, msg.path[rightThird]);

        if (distLeft < distRight)
        {
            right = rightThird;
        }
        else
        {
            left = leftThird;
        }
        // std::cout << "left=" << left << "--" << "right=" << right << std::endl; // RCLCPP_INFO(get_logger(), "left=%d, right=%d", left,right);
    }

    // 最后线性遍历剩余区间，找最小距离索引
    int closest_index = left;
    double min_dist = distanceSquared_node(p, msg.path[left]);
    for (int i = left + 1; i <= right; ++i)
    {
        double dist = distanceSquared_node(p, msg.path[i]);
        if (dist < min_dist)
        {
            min_dist = dist;
            closest_index = i;
        }
    }
    return closest_index;
}

uint8_t ControlNode::checkisstart_init(const Point3D &start_point, const Point3D &end_point, const byd_custom_msgs::msg::PathPoints &current_xy)
{
    uint8_t res;
    double proj_len;
    double mid_value;
    mid_value = current_xy.speed - std::atan2((end_point.y - start_point.y), (end_point.x - start_point.x));

    mid_value = normalizeAngle(mid_value);
    double min_distance = 0.2; // 最小距离阈值
    // 构造向量
    double vx = end_point.x - start_point.x;
    double vy = end_point.y - start_point.y;
    double wx = current_xy.x - start_point.x;
    double wy = current_xy.y - start_point.y;

    // // 投影长度 = (w·v) / |v|
    // double v_len = std::sqrt(vx * vx + vy * vy);
    // if (v_len < 1e-9) proj_len = 0.0; // 防止除零

    // double dot = wx * vx + wy * vy;
    // proj_len = dot / v_len;

    // 投影长度 = (w·v) / |v|
    double hat_dot = wx * vy - wy * vx;
    double v_len = std::sqrt(vx * vx + vy * vy);
    if (v_len < 1e-9) 
    {proj_len = 0.0;} // 防止除零
    else
    {
        proj_len = hat_dot / v_len;
    }
    
    LOG_INFO("proj_len:{},dot:{},mid_value:{}",proj_len,hat_dot,mid_value);
    if (abs(proj_len) > min_distance)
    {
        res = 1; // 投影长度大于阈值，认为是起始点
    }
    else if (abs(mid_value) > 0.3)
    {
        res = 2; // 认为是旋转点
    }
    else
    {
        res = 0; // 认为是普通点
    }
    return res;
}

bool ControlNode::checkisrotation(const Point3D &start_point, const Point3D &end_point, const byd_custom_msgs::msg::PathPoints &current_xy)
{
    bool res;
    double mid_value;
    mid_value = current_xy.speed - std::atan2((end_point.y - start_point.y), (end_point.x - start_point.x));

    mid_value = normalizeAngle(mid_value);

    if (abs(mid_value) > 0.3) // 弧度制
    {
        res = true;
    }
    else
    {
        res = false;
    }
    return res;
}

/**
 * @brief 判断二维点P到线段AB的投影点是否在线段内部
 *
 * @param x  点P的x坐标
 * @param y  点P的y坐标
 * @param x1 线段端点A的x坐标
 * @param y1 线段端点A的y坐标
 * @param x2 线段端点B的x坐标
 * @param y2 线段端点B的y坐标
 * @return true  投影点在线段AB上
 * @return false 投影点不在线段AB上（在延长线上）
 */

bool ControlNode::isProjectionOnSegment(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const Eigen::Vector2d &point)
{
    Eigen::Vector2d v1 = end - start;
    Eigen::Vector2d v2 = end - point;
    double dot = v1.dot(v2);
    return dot < 0;
}

/**
 * @brief 计算下一角速度，使角度快速平滑到目标，满足最大角速度和加速度限制
 *
 * @param delta        当前角度（rad）
 * @param omega        当前角速度（rad/s）
 * @param desire_delta 期望角度（rad）
 * @param wmax         最大角速度（rad/s，正数）
 * @param wacc         最大角加速度（rad/s^2，正数）
 * @param dt           时间步长（秒）
 * @return double      下一时刻期望角速度（rad/s）
 */
double ControlNode::controlAngularVelocity(double error, double omega,double wmax, double wacc, double dt)
{
    // 计算理想停止距离（利用当前速度停止需要的距离）
    // 停止距离 d = omega^2/(2*acc)
    // double stopping_distance = (omega * omega) / (2.0 * wacc);
    double stopping_distance = 2 * (wmax * wmax) / wacc;
    // double stopping_distance = 24.0/180.0*M_PI;

    // 判断误差符号，确定加速度方向
    double direction = (error >= 0) ? 1.0 : -1.0;

    // 判断是否需要减速
    if (std::abs(error) <= stopping_distance)
    {
        // 减速阶段，逐步降低速度
        omega -= direction * wacc * dt;
        // omega = std::sqrt(2 *  wacc * std::abs(error)) * direction;
        // 防止速度反向，限制在0附近

        // if (direction * omega < 0)
        if (direction * omega < 2.0 / 180.0 * M_PI)
        {
            omega = direction * 2.0 / 180.0 * M_PI;
        }
        if (std::abs(error) < 0.01)
        {
            omega = 0;
        }
    }
    else
    {
        // 加速阶段，加速到最大角速度限制
        omega += direction * wacc * dt;
        // 限制最大角速度
        if (std::abs(omega) > wmax)
        {
            omega = direction * wmax;
        }
    }

    return omega;
}

// 限制输入值在 [min_val, max_val] 范围内
double ControlNode::clamp(double val, double min_val, double max_val)
{
    return std::max(min_val, std::min(val, max_val));
}

// vwprocess函数：平滑速度和角速度控制
// 参数:
//    car_v, car_w   : 当前速度和角速度
//    desire_v, desire_w : 期望速度和角速度
// 返回:
//    std::pair<double,double> : 平滑后的速度(goal_v)和角速度(goal_w)
std::pair<double, double> ControlNode::vwprocess(double car_v, double car_w, double desire_v, double desire_w,
                                    double max_acc_v = 0.2, // 最大线加速度 (单位 m/s^2)
                                    double max_acc_w = 0.5, // 最大角加速度 (单位 rad/s^2)
                                    double dt = 0.1)        // 控制周期 (秒)
{
    // 计算线速度差与角速度差
    std::pair<double, double> res_pair;
    double diff_v = desire_v - car_v;
    double diff_w = desire_w - car_w;

    // 限制速度变化量，防止突变
    double delta_v = clamp(diff_v, -max_acc_v * dt, max_acc_v * dt);
    double delta_w = clamp(diff_w, -max_acc_w * dt, max_acc_w * dt);

    // 更新速度和角速度
    double goal_v = car_v + delta_v;
    double goal_w = car_w + delta_w;
    res_pair.first = goal_v;
    res_pair.second = goal_w;

    return res_pair;
}

double ControlNode::liftvprocess(double lift_height_current, double lift_height_desire, double max_v, double max_a, double dt, double epsilon,double startstopdis)
{
    static double last_v = 0.0;

    double error = lift_height_desire - lift_height_current;

    // double startstopdis = 
    double delta_v = max_a * dt;
    double goal_v;
    double desired_v;

    // 死区内，停止移动
    if (std::fabs(error) < epsilon)
    {
        last_v = 0.0;
        return 0.0;
    }
    else if(std::fabs(error) < startstopdis)
    {
        if(error<0)
        {
            desired_v = last_v + delta_v;
            if (desired_v > -0.003)
            {desired_v = -0.003;}
        }
        else
        {
            desired_v = last_v - delta_v;
            if (desired_v < 0.003)
            {desired_v = 0.003;}
        }
    }
    else
    {
        if(error<0)
        {goal_v = last_v - delta_v;}
        else
        {goal_v = last_v + delta_v;}
        
        desired_v = std::clamp(goal_v, -max_v, max_v);
    }

    last_v = desired_v;

    return desired_v;
}

double ControlNode::roatvprocess(double lift_height_current, double lift_height_desire, double max_v, double max_a, double dt, double epsilon,double startstopdis)
{
    static double last_v = 0.0;

    double error = lift_height_desire - lift_height_current;

    // double startstopdis = 
    double delta_v = max_a * dt;
    double goal_v;
    double desired_v;

    // 死区内，停止移动
    if (std::fabs(error) < epsilon)
    {
        last_v = 0.0;
        return 0.0;
    }
    else if(std::fabs(error) < startstopdis)
    {
        if(error<0)
        {
            desired_v = last_v + delta_v;
            if (desired_v > -0.03)
            {desired_v = -0.03;}
        }
        else
        {
            desired_v = last_v - delta_v;
            if (desired_v < 0.03)
            {desired_v = 0.03;}
        }
    }
    else
    {
        goal_v = last_v + delta_v;
        desired_v = std::clamp(goal_v, -max_v, max_v);
    }

    last_v = desired_v;

    return desired_v;
}

// 计算两个点的向量差
Point3D ControlNode::vectorDiff(const Point3D &a, const Point3D &b)
{
    return {a.x - b.x, a.y - b.y, a.speed - b.speed};
}

// 向量长度
double ControlNode::vectorLength(const Point3D &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.speed * v.speed);
}

// 向量归一化
Point3D ControlNode::normalizeVector(const Point3D &v)
{
    double len = vectorLength(v);
    if (len < 1e-9)
    {
        return {0.0, 0.0, 0.0}; // 避免除零
    }
    return {v.x / len, v.y / len, v.speed / len};
}
// RCLCPP_INFO
// 函数：延长终点固定距离，返回新的终点
Point3D ControlNode::extendEndPoint(const Point3D &start, const Point3D &end, const double extend_distance)
{
    // const double extend_distance = 2.0; // 固定延长距离，可以改成参数

    Point3D dir = vectorDiff(end, start);    // 方向向量 end - start
    Point3D unit_dir = normalizeVector(dir); // 单位方向向量

    Point3D new_end;
    new_end.x = end.x + unit_dir.x * extend_distance;
    new_end.y = end.y + unit_dir.y * extend_distance;
    new_end.speed = end.speed + unit_dir.speed * extend_distance;

    return new_end;
}
