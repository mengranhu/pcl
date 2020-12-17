#ifndef BBOX_H_
#define BBOX_H_

#include "../include/hungarian_algorithm.h"
#include "../include/object.h"

#include <eigen3/Eigen/Dense>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <sys/time.h>
#include <time.h>

class Bbox
{
private:
  ros::Subscriber sub_bbox_array_;

  void point_cb(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &in_bbox);

public:
  Bbox(ros::NodeHandle &nh);
  ~Bbox();

  HungarianAlgorithm hungrian{};

  double threshold_;
  std::vector<Object> obj_result_list_;

  // void iouXupdate(std::vector<Eigen::MatrixXd> &detection_matrix, std::vector<Eigen::MatrixXd> &predict_matrix, int index,
  //                 double delta_time);
};

#endif //  BBOX_H_
