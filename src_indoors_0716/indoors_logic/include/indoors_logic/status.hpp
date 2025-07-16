/**
 * @file status.hpp
 * @author yangfy (yang.fengyuan2@byd.com)
 * @brief 车辆状态定义
 * @version 0.1
 * @date 2025-07-04
 *
 * @copyright Copyright (c) 2025
 *
 */
#ifndef __CAR_STATUS_H__
#define __CAR_STATUS_H__

enum class CarOpStatus {
  DEFAULT = 0,        //默认初值
  CAR_SPIN = 1,       //原地旋转
  SHELF_SPIN = 2,     //托盘旋转
  CAR_SHELF_SPIN = 3, //随动
  SHELF_LIFT = 4,     //托盘举升
  SHELF_UNLIFT = 5,   //托盘下降
  NAV = 6,            //路径跟踪
  IDLE = 7            //无任务
};

#endif