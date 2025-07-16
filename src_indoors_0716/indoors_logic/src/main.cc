#include <nlopt.hpp>
#include <cmath>
#include <iostream>
#include <vector>
#include "bezier_model.hpp"

int main(int argc, char** argv) {

    // 默认约束
    Eigen::Vector3d startPos(0, 0, 0);
    Eigen::Vector3d endPos(1000, 100, 0);   

    if(argc != 1 && argc != 7) {
        std::cerr << "Usage: " << argv[0] << " startX startY startAngle endX endY endAngle\n";
        return 1;
    }
    
    // 从argv中读取起点和终点，以及角度信息
    // 需要6个参数（程序名不算），所以argc应该是7
    if(argc == 7) {
        startPos(0) = std::stod(argv[1]);  // 第一个x坐标
        startPos(1) = std::stod(argv[2]);  // 第一个y坐标
        startPos(2) = std::stod(argv[3])/180*M_PI;  // 第一个角度
        endPos(0) = std::stod(argv[4]);    // 第二个x坐标
        endPos(1) = std::stod(argv[5]);    // 第二个y坐标
        endPos(2) = std::stod(argv[6])/180*M_PI;    // 第二个角度

        std::cout<<"startPOS: "<<startPos.transpose()<<std::endl;
        std::cout<<"endPOS: "<<endPos.transpose()<<std::endl;
    }

    BezierModel model(5,1,0.000004,0.000002,0.000001);
    
    
    model.setStartPoint(startPos);
    model.setEndPoint(endPos);
    model.initializeControlPoints();

    model.setCurvatueRange(1,-1);
    model.setSampleTimes(1000);
    model.setKesai(5,-5);
    // 设置起点终点
    model.setStartPoint(startPos);
    model.setEndPoint(endPos);
    model.initializeControlPoints();

    // 执行优化
    model.optimizeControlPoints();

    std::cout<<"startPOS: "<<startPos.transpose()<<std::endl;
    std::cout<<"endPOS: "<<endPos.transpose()<<std::endl;
    // model.plotCurvature();
    // model.plotCurvatureDerivative();
    // model.plotBezierCurve(model);
    // model.plotAngleRate();
    
    return 0;
}