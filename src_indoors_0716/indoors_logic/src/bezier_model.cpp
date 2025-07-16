#include "bezier_model.hpp"
#include <iostream>
#include <cmath>

bool use_grad=false;
double composite_simpson(std::function<double(double)> f, double a, double b, int n);
void BezierModel::initializeMatrix() {
        B.resize(degree_ + 1, degree_ + 1);
        B.setZero(); // 初始化为零矩阵
        B << -1,5,-10,10,-5,1,
              5,-20,30,-20,5,0,
              -10,30,-30,10,0,0,
              10,-20,10,0,0,0,
              -5,5,0,0,0,0,
              1,0,0,0,0,0;
        this->controlPoints.resize(6,2);
        this->controlPoints.setZero(); // 初始化为零矩阵
        this->controlPoints<<1,1,2,2,3,3,4,4,5,5,6,6;
        std::cout<<"初始化成功！"<<std::endl;
        std::cout<<"B矩阵为:"<<std::endl<<B<<std::endl;
        std::cout<<"controlPoints矩阵为:"<<std::endl<<controlPoints<<std::endl;
}

// 计算组合数 C(n, k)
int BezierModel::binomialCoefficient(int n, int k) {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;
    double result = 1.0;
    for (int i = 1; i <= k; ++i) {
        result *= (n - i + 1) / static_cast<double>(i);
    }
    return static_cast<int>(result);
}

//目标函数  
double BezierModel::objective(const std::vector<double> &x, std::vector<double> &grad, void *)
{
    // std::cout<< "BezierModel::objective--x.size()="<<x.size()<<std::endl;
    if (use_grad) {
       {
         std::cout<< "cal use grad info!!!"<<std::endl;
         grad[0]=0;
       }
    }
    auto f_lengh = [this](double u) { return this->get_objective_value_lengh(u); };
    auto f_curverate = [this](double u) { return this->get_objective_value_curverate(u); };
    auto f_anglerate = [this](double u) { return this->get_objective_anglerate(u); };
    auto f_speed = [this](double u) { return this->get_objective_value_speed(u); };


    double result = 0.0;
    result = w_l* composite_simpson(f_lengh,0,1,1000)
            +w_k*composite_simpson(f_curverate,0,1,1000)
            +w_a*composite_simpson(f_anglerate,0,1,1000)
            +w_s*composite_simpson(f_speed,0,1,1000);
    return result;
}

double composite_simpson(std::function<double(double)> f, double a, double b, int n) {
    if (n % 2 != 0) n++;  // 确保n为偶数
    double h = (b - a) / n;
    double sum = f(a) + f(b);
    
    for (int i = 1; i < n; i++) {
        double x = a + i * h;
        sum += (i % 2 == 0) ? 2 * f(x) : 4 * f(x);
    }
    return sum * h / 3;
}

Eigen::Vector2d BezierModel::get_first_derivative(double u) const
{
    Eigen::RowVectorXd p(6);
    p<<5*pow(u,4),4*pow(u,3),3*pow(u,2),2*u,1,0;
    Eigen::Vector2d first_derivative;
    first_derivative=p*this->B*controlPoints;
    return first_derivative;
}

Eigen::Vector2d BezierModel::get_second_derivative(double u) const {
    Eigen::RowVectorXd p(6);
    p << 20*pow(u,3), 12*pow(u,2), 6*u, 2, 0, 0; // 修正后
    return p * this->B * controlPoints;
}

Eigen::Vector2d BezierModel::get_third_derivative(double u)const
{
    Eigen::RowVectorXd p(6);
    p << 60*pow(u,2), 24*u, 6, 0, 0, 0; // 三阶导数系数
    return p * this->B * controlPoints;
}

Eigen::Vector2d BezierModel::get_position(double u)
{
     Eigen::RowVectorXd p(6);
     p<<pow(u,5),pow(u,4),pow(u,3),pow(u,2),u,1;
     Eigen::Vector2d position;
     position=p*this->B*controlPoints;
     return position;
}

double BezierModel::get_curvature(double u) const
{
    Eigen::Vector2d first_derivative=get_first_derivative(u);
    Eigen::Vector2d second_derivative=get_second_derivative(u);
    double curvature = first_derivative.x() * second_derivative.y() - 
                   first_derivative.y() * second_derivative.x();
    curvature /= std::pow(first_derivative.norm(), 3);
    return 1000*curvature;
}

double BezierModel::get_objective_value_lengh(double u)
{
    double result=0.0;
    Eigen::Vector2d first_derivative=get_first_derivative(u);
    result=pow(first_derivative.norm(),2);
    return result;
}

// 路径平滑性评价函数
double BezierModel::get_objective_anglerate(double u)
{
    double result=0.0;
    Eigen::Vector2d second_derivative=get_second_derivative(u);
    result=pow(second_derivative.norm(),2);
    return result;
}

// 车速平滑性评价函数
double BezierModel::get_objective_value_speed(double u)
{
     double result=0.0;
    Eigen::Vector2d third_derivative=get_third_derivative(u);
    result=pow(third_derivative.norm(),2);
    return result;
}

double BezierModel::get_objective_value_curverate(double u)
{
    double result=0.0;
    if(u>0.4)
    {
        result=10*pow(get_curvature(u),2);
    }
    else
    {
        result=pow(get_curvature(u),2);
    }
    return result;
}


void BezierModel::optimizeControlPoints() {
    // 获取初始控制点（展平为一维向量）
    std::vector<double> x = controlPointsToVector();
    
    // 创建NLopt优化器
    nlopt::opt opt(nlopt::LN_COBYLA, x.size()); // 无梯度约束优化
    
    // 设置目标函数
    opt.set_min_objective(BezierModel::objectiveWrapper, this);
    
    // 设置优化参数
    opt.set_xtol_rel(1e-6); // 相对容差
    opt.set_maxeval(40);   // 最大评估次数

     // 添加起点约束（3个等式：x,y,角度）
    std::vector<double> tol_start(5, 1e-6);
    tol_start[2]=1e-8;
    tol_start[3]=1e-8;
    opt.add_equality_mconstraint(startPointConstraintWrapper, this, tol_start);
    
    // 添加终点约束（3个等式：x,y,角度）
    std::vector<double> tol_end(5, 1e-6); 
    tol_end[2]=1e-8;
    tol_end[3]=1e-8;
    opt.add_equality_mconstraint(endPointConstraintWrapper, this, tol_end);

    // 添加曲率不等式约束
    std::vector<double> curvature_tol(2 * this->curvatureSamples, 1e-6);
    opt.add_inequality_mconstraint(BezierModel::curvatureConstraintWrapper, 
                                 this, curvature_tol);

    // 添加角速度约束
    std::vector<double> angle_rate_tol(2 * this->curvatureSamples, 1e-6);
    opt.add_inequality_mconstraint(BezierModel::AngleRateConstraintWrapper, 
                                 this, angle_rate_tol);


    // 设置变量边界
    // std::vector<double> lb(x.size(), -10.0); // 下限
    // std::vector<double> ub(x.size(), 10.0);  // 上限
    // opt.set_lower_bounds(lb);
    // opt.set_upper_bounds(ub);
    
    // 执行优化
    double minf;
    try {
        nlopt::result result = opt.optimize(x, minf);
        
        if (result < 0) {
            std::cerr << "NLopt failed with code " << result << std::endl;
        } else {
            std::cout << "Optimization succeeded, minimum value = " << minf << std::endl;
            // 更新控制点
            vectorToControlPoints(x);
        }
    } catch (std::exception& e) {
        std::cerr << "NLopt exception: " << e.what() << std::endl;
    }
}


// 在bezier_model.cc中实现正确的曲率导数计算
double BezierModel::getCurvatureDerivative(double u) const {
    Eigen::Vector2d first_derivative = get_first_derivative(u);
    Eigen::Vector2d second_derivative = get_second_derivative(u);
    
    // 计算dk/du
    double curvature_derivative = 2 * first_derivative.x() * second_derivative.y() 
                               - 2 * first_derivative.y() * second_derivative.x();
    curvature_derivative /= pow(first_derivative.norm(), 3);
    
    // 计算ds/du
    double dsdu = first_derivative.norm();
    
    // 根据链式法则计算dk/ds
    return curvature_derivative / dsdu;
}

double BezierModel::calculateAngleRate(double u) const {
    // 计算在u处的角速度
    Eigen::Vector2d dr = get_first_derivative(u);  // 一阶导数 r'
    Eigen::Vector2d ddr = get_second_derivative(u); // 二阶导数 r''
    
    // 计算叉积 (r' × r'') = r'_x * r''_y - r'_y * r''_x
    double cross_product = dr.x() * ddr.y() - dr.y() * ddr.x();
    
    // 计算速度立方 ||r'||³
    double speed_cubed = std::pow(dr.norm(), 3);
    
    // 角速度 ω = (r' × r'') / ||r'||³
    return cross_product / speed_cubed;
}

void BezierModel::initializeControlPoints() {
    // 获取起点和终点
    Eigen::Vector2d p0 = this->desiredStartPoint;
    Eigen::Vector2d p5 = this->desiredEndPoint;
    
    // 计算起点和终点的单位方向向量
    Eigen::Vector2d start_dir_vec(std::cos(this->desiredStartAngle), 
                                 std::sin(this->desiredStartAngle));
    Eigen::Vector2d end_dir_vec(std::cos(this->desiredEndAngle), 
                               std::sin(this->desiredEndAngle));
    
    // 计算起点到终点的距离
    double total_dist = (p5 - p0).norm();
    
    // 设置控制点距离系数（可调整）
    const double k1 = total_dist * 0.3; // 起点附近控制点距离系数
    const double k2 = total_dist * 0.3; // 终点附近控制点距离系数
    
    // 初始化基础控制点
    controlPoints.row(0) = p0; // P0
    controlPoints.row(1) = p0 + k1 * start_dir_vec; // P1沿起点方向
    controlPoints.row(4) = p5 - k2 * end_dir_vec;   // P4沿终点反方向 
    controlPoints.row(5) = p5; // P5

    // 计算中间控制点（确保共线性）
    // P2 在 P0-P1 延长线上
    controlPoints.row(2) = controlPoints.row(1) + (controlPoints.row(1) - controlPoints.row(0));
    
    // P3 在 P4-P5 延长线上
    controlPoints.row(3) = controlPoints.row(4) + (controlPoints.row(4) - controlPoints.row(5));

    std::cout << "Control points: \n" << controlPoints << std::endl;
}
