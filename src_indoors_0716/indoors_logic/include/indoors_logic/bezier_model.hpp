#ifndef BEZIER_MODEL_HPP
#define BEZIER_MODEL_HPP

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <nlopt.hpp>

typedef double (*objective_func_t)(const std::vector<double> &, std::vector<double> &, void *);
extern bool use_grad;

class BezierModel
{
public:
    // 控制点结构体
    struct Point
    {
        double x;
        double y;
        Point(double x = 0, double y = 0) : x(x), y(y) {}
    };
    Eigen::Vector2d desiredStartPoint;
    Eigen::Vector2d desiredEndPoint;
    Eigen::MatrixXd controlPoints;
    double desiredStartAngle; // 弧度
    double desiredEndAngle;   // 弧度
    double minCurvature;
    double maxCurvature;
    double maxkesai;
    double minkesai;
    int curvatureSamples;

    // 构造函数，接受贝塞尔曲线的阶数
    BezierModel(int degree, double w_k = 10, double w_l = 0.1, double w_a = 1, double w_s = 1) : degree_(degree)
    {
        initializeMatrix();
        this->w_k = w_k;
        this->w_l = w_l;
        this->w_a = w_a;
        this->w_s = w_s;
    }

    // 公共成员函数，返回贝塞尔参数矩阵 B
    const Eigen::MatrixXd &getMatrixB() const
    {
        return B;
    }
    // 计算曲率对弧长的导数
    double getCurvatureDerivative(double u) const;

    // // 绘制曲率对弧长的导数曲线
    // void plotCurvatureDerivative(const std::string& title = "Curvature Derivative") const;

    // //绘制曲率曲线
    // void plotCurvature(const std::string& title = "Curvature") const;

    // // 绘制角速度曲线
    // void plotAngleRate(const std::string& title = " Angle Rate") const;

    // // 绘制贝塞尔曲线
    // void plotBezierCurve(const std::string& title = "Bezier Curve") const;

    double calculateAngleRate(double u) const;

    void setCurvatueRange(double max, double min)
    {
        minCurvature = min;
        maxCurvature = max;
    }

    void setSampleTimes(int times)
    {
        curvatureSamples = times;
    }

    void setKesai(double maxkesai, double minkesai)
    {
        this->maxkesai = maxkesai;
        this->minkesai = minkesai;
    }

    void vectorToControlPoints(const std::vector<double> &x)
    {
        Eigen::VectorXd vec = Eigen::VectorXd::Map(x.data(), x.size());
        controlPoints = Eigen::Map<Eigen::MatrixXd>(vec.data(), controlPoints.rows(), controlPoints.cols());
        // std::cout << "controlPoints:" << std::endl
        //           << controlPoints << std::endl;
        // std::cout << "start_curvature:" << get_curvature(0) << std::endl;
        // std::cout << "end_curvature:" << get_curvature(1) << std::endl;
    }

    std::vector<double> controlPointsToVector() const
    {
        std::vector<double> x(controlPoints.size());
        Eigen::Map<Eigen::VectorXd>(x.data(), controlPoints.size()) =
            Eigen::VectorXd::Map(controlPoints.data(), controlPoints.size());
        return x;
    }

    // 设置曲线起点
    void setStartPoint(const Eigen::Vector3d &startPoint)
    {
        desiredStartPoint = startPoint.head<2>();
        desiredStartAngle = startPoint(2);
    }

    // 设置曲线终点
    void setEndPoint(const Eigen::Vector3d &endPoint)
    {
        desiredEndPoint = endPoint.head<2>();
        desiredEndAngle = endPoint(2);
    }

    // 曲线角速率约束
    void AngleRateConstraint(const double *x, double *result, void *data,int m,int n,double *grad)
    {
        BezierModel *model = static_cast<BezierModel *>(data);
        Eigen::Map<const Eigen::MatrixXd> cp(x, 6, 2);
        // std::cout << "AngleRateConstraint:" << m << "----" << n << "----" << grad << std::endl;

        // 在多个点上检查角速率约束
        for (int i = 0; i < model->curvatureSamples; ++i)
        {
            double u = static_cast<double>(i) / (model->curvatureSamples - 1);
            double angle_rate = model->calculateAngleRate(u);

            // 角速率约束: angle_rate - maxδ ≤ 0
            result[2 * i] = angle_rate - model->maxkesai;

            // 角速率约束: δ - angle_rate ≤ 0
            result[2 * i + 1] = model->minkesai - angle_rate;
        }
    }

    // 起点约束（位置和角度）
    void startPointConstraint(const double *x, double *result, void *data,int m,int n,double *grad)
    {
        BezierModel *model = static_cast<BezierModel *>(data);
        Eigen::Map<const Eigen::MatrixXd> cp(x, 6, 2);
        double start_curvature = get_curvature(0);
        // std::cout << "startPointConstraint:" << m << "----" << n << "----" << grad << std::endl;

        // 位置约束（2个等式）
        result[0] = cp(0, 0) - model->desiredStartPoint.x(); // x坐标
        result[1] = cp(0, 1) - model->desiredStartPoint.y(); // y坐标

        // 角度约束（1个等式）
        if (cp(1, 0) == cp(0, 0) && cp(1, 1) == cp(0, 1))
        {
            result[2] = 0; // 零向量无角度定义
        }
        else
        {
            double current_angle = atan2(cp(1, 1) - cp(0, 1), cp(1, 0) - cp(0, 0));
            result[2] = angleDifference(current_angle, model->desiredStartAngle);
        }
        result[3] = start_curvature;

        // 起点三点共线约束（1个等式）
        result[4] = (cp(1, 0) - model->desiredStartPoint.x()) * (cp(2, 1) - model->desiredStartPoint.y()) - (cp(1, 1) - model->desiredStartPoint.y()) * (cp(2, 0) - model->desiredStartPoint.x());
    }

    // 终点约束（位置和角度）
    void endPointConstraint(const double *x, double *result, void *data,int m,int n,double *grad)
    {
        BezierModel *model = static_cast<BezierModel *>(data);
        Eigen::Map<const Eigen::MatrixXd> cp(x, 6, 2);
        // std::cout << "endPointConstraint:" << m << " " << n << "" << grad << std::endl;

        // 计算曲线终点位置 P(1)
        Eigen::RowVectorXd u1(6);
        u1 << 1, 1, 1, 1, 1, 1; // u=1时的多项式值
        Eigen::Vector2d end_pos = u1 * model->B * cp;

        double end_curvature = get_curvature(1);

        // 位置约束（2个等式）
        result[0] = end_pos.x() - model->desiredEndPoint.x();
        result[1] = end_pos.y() - model->desiredEndPoint.y();

        // 角度约束（1个等式）
        if (cp(5, 0) == cp(4, 0) && cp(5, 1) == cp(4, 1))
        {
            result[2] = 0; // 零向量无角度定义
        }
        else
        {
            double current_angle = atan2(cp(5, 1) - cp(4, 1), cp(5, 0) - cp(4, 0));
            result[2] = angleDifference(current_angle, model->desiredEndAngle);
            // std::cout << "endpoint_angle:" << current_angle << " " << "desired_angle:" << model->desiredEndAngle << std::endl;
        }
        result[3] = end_curvature;
        // 终点三点共线约束（1个等式）
        result[4] = (cp(5, 0) - model->desiredEndPoint.x()) * (cp(3, 1) - model->desiredEndPoint.y()) - (cp(5, 1) - model->desiredEndPoint.y()) * (cp(3, 0) - model->desiredEndPoint.x());
    }
    // 曲率约束函数
    void curvatureConstraint(unsigned n, const double *x, double *result, void *data,int m,double *grad)
    {
        BezierModel *model = static_cast<BezierModel *>(data);
        model->vectorToControlPoints(std::vector<double>(x, x + n));
        // std::cout << "curvatureConstraint:" << m << "----" << grad << std::endl;

        // 在多个点上检查曲率约束
        for (int i = 0; i < model->curvatureSamples; ++i)
        {
            double u = static_cast<double>(i) / (model->curvatureSamples - 1);
            double curvature = model->get_curvature(u);

            // 最大曲率约束: curvature - κ_max ≤ 0
            result[2 * i] = curvature - model->maxCurvature;

            // 最小曲率约束: κ_min - curvature ≤ 0
            result[2 * i + 1] = model->minCurvature - curvature;
        }
    }

    // 计算角度差（-π到π）
    static double angleDifference(double a, double b)
    {
        double diff = a - b;
        while (diff > M_PI)
            diff -= 2 * M_PI;
        while (diff < -M_PI)
            diff += 2 * M_PI;
        return diff;
    }

    // 约束包装函数
    static void startPointConstraintWrapper(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        static_cast<BezierModel *>(data)->startPointConstraint(x, result, data,m,n,grad);
    }

    static void endPointConstraintWrapper(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        static_cast<BezierModel *>(data)->endPointConstraint(x, result, data,m,n,grad);
    }
    // 曲率约束包装函数
    static void curvatureConstraintWrapper(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        static_cast<BezierModel *>(data)->curvatureConstraint(n, x, result, data,m,grad);
    }

    // 角速率约束包装函数
    static void AngleRateConstraintWrapper(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        static_cast<BezierModel *>(data)->AngleRateConstraint(x, result, data,m,n,grad);
    }

    // 静态包装函数，用于NLopt回调
    static double objectiveWrapper(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        BezierModel *model = static_cast<BezierModel *>(data);
        model->vectorToControlPoints(x);
        return model->objective(x, grad, nullptr);
    }

    // void plotBezierCurve(BezierModel &model, const std::string &title = "Optimized Bezier Curve");

    void optimizeControlPoints();

    Eigen::Vector2d get_first_derivative(double u) const;

    Eigen::Vector2d get_second_derivative(double u) const;

    Eigen::Vector2d get_third_derivative(double u) const;

    Eigen::Vector2d get_position(double u);

    double get_objective_value_lengh(double u);
    double get_objective_anglerate(double u);
    double get_objective_value_speed(double u);
    double get_objective_value_curverate(double u);

    double get_curvature(double u) const;

    void initializeControlPoints();

    // 目标函数指针
    double objective(const std::vector<double> &x, std::vector<double> &grad, void *);

private:
    int degree_; // 贝塞尔曲线的阶数
    // Eigen::MatrixXd controlPoints;
    std::vector<double> P_d;   // 控制点的距离
    Eigen::MatrixXd B;         // 贝塞尔参数矩阵
    objective_func_t obj_func; // 目标函数指针
    double w_k;                // 曲率权重
    double w_l;                // 路径长度权重
    double w_a;                // 角度权重
    double w_s;                // 速度权重

    // 初始化贝塞尔参数矩阵
    void initializeMatrix();

    // 计算组合数 C(n, k)
    int binomialCoefficient(int n, int k);
};

#endif // BEZIER_MODEL_HPP
