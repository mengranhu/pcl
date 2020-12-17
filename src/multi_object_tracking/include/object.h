#ifndef OBJECT_H_
#define OBJECT_H_

#include "tracking.h"
#include <stdint.h>
class Object
{
private:
  /* data */
public:
  struct obj
  {
    uint64_t ID;       // 目标物ID
    uint8_t Age;       // 目标物寿命（可信度）
    unsigned int Type; // 目标物类型
    double length;     // 目标长度X
    double width;      // 目标宽度Y
    double height;     // 目标高度Z
    double x;          // 目标X方向位置（纵向）
    double y;          // 目标Y方向位置（横向）
    double z;          // 目标Z方向位置（竖直）
    double vx;         // 目标纵向速度
    double vy;         // 目标横向速度
    double ax;         // 目标纵向加速度
    double ay;         // 目标横向加速度
    int64_t timestamp; // 获取目标的时间
  };                   // 障碍物（物理数据）类型定义
  obj obj_;
  Tracking tracking_{};
  MeasurementPackage pkg_;
  void init_pkg()
  {
    pkg_.raw_measurements_ = VectorXd(7);
    pkg_.raw_measurements_ << obj_.x, obj_.y, obj_.z, atan(obj_.y / obj_.x), obj_.length, obj_.width, obj_.height;
    pkg_.timestamp_ = obj_.timestamp;
  }
  void update_obj_data()
  {
    obj_.x = tracking_.kf_.x_(0);
    obj_.y = tracking_.kf_.x_(1);
    obj_.z = tracking_.kf_.x_(2);
    obj_.vx = tracking_.kf_.x_(7);
    obj_.vy = tracking_.kf_.x_(8);
  }
};
#endif //  OBJECT_H_
