#include "../include/bbox.h"
#include "../include/hungarian_algorithm.h"
#include <algorithm>

#define max(x, y) x > y ? x : y

using Eigen::MatrixXd;
using Eigen::VectorXd;

Bbox::Bbox(ros::NodeHandle &nh)
{
    nh.getParam("threshold", threshold_);

    obj_result_list_.clear();

    /* 定于话题，初始化回调 */
    sub_bbox_array_ = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>(
        "/detected_bounding_boxs", 5, &Bbox::point_cb, this);

    ros::spin();
}

Bbox::~Bbox()
{
}

static void obj_print(std::vector<Object> obj_list)
{
    for (auto it = 0; it < obj_list.size(); it++) {

        // std::cout << " obj[" << it << "].L : " << obj_list[it].obj_.length
        //           << " obj[" << it << "].W : " << obj_list[it].obj_.width
        //           << " obj[" << it << "].H : " << obj_list[it].obj_.height
        //           << std::endl;
        // std::cout << " obj[" << it << "].x : " << obj_list[it].obj_.x
        //           << " obj[" << it << "].y : " << obj_list[it].obj_.y
        //           << " obj[" << it << "].z : " << obj_list[it].obj_.z
        //           << std::endl;
        std::cout << " obj[" << it << "].ID : " << obj_list[it].obj_.ID
                  << " , obj[" << it << "].vx : " << obj_list[it].obj_.vx
                  << " , obj[" << it << "].vy : " << obj_list[it].obj_.vy
                  << " , history : " << obj_list[it].tracking_.history_
                  << std::endl;
    }
}

static double getTime()
{
    struct timeval tvpre;
    struct timezone tz;
    int sum = 0;
    int i = 0;
    gettimeofday(&tvpre, &tz);
    return (tvpre.tv_sec) * 1000 + (tvpre.tv_usec) / 1000;
}

// void Bbox::iouXupdate(std::vector<MatrixXd> &detection_matrix,
//                       std::vector<MatrixXd> &predict_matrix, int index,
//                       double delta_time)
// {
//     if ((int)cx(index) != -1) {
//         predict_matrix[(int)cx(index)](7, 0) =
//             (detection_matrix[index](0, 0) - predict_matrix[(int)cx(index)](0, 0)) / (delta_time / 1000);
//         predict_matrix[(int)cx(index)](8, 0) =
//             (detection_matrix[index](1, 0) - predict_matrix[(int)cx(index)](1, 0)) / (delta_time / 1000);
//         predict_matrix[(int)cx(index)](9, 0) =
//             (detection_matrix[index](2, 0) - predict_matrix[(int)cx(index)](2, 0)) / (delta_time / 1000);
//     }
// }

static double _g_time = getTime();

void Bbox::point_cb(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &in_bbox)
{
    double new_time = 0;
    std::cout << " -----------I'm A Separator !!!  old size = " << obj_result_list_.size()
              << ", current size = " << in_bbox->boxes.size() << std::endl;

    int index = 0;
    std::vector<Object> current_list;

    // 第一次循环将检测bbox数据添加到current_list中
    for (auto it = in_bbox->boxes.begin(); it != in_bbox->boxes.end(); ++it) {
        Object obj;
        obj.obj_.ID = index + 1;
        obj.obj_.length = it->dimensions.x;
        obj.obj_.width = it->dimensions.y;
        obj.obj_.height = it->dimensions.z;

        obj.obj_.x = it->pose.position.x;
        obj.obj_.y = it->pose.position.y;
        obj.obj_.z = it->pose.position.z;

        obj.obj_.vx = 0;
        obj.obj_.vy = 0;

        obj.obj_.timestamp = getTime();

        obj.init_pkg();
        obj.tracking_.process_measurement(obj.pkg_);

        current_list.push_back(obj);
        index++;
    }
    // obj_result_list_用于存储检测预测更新结果
    if (obj_result_list_.empty()) {
        // 判断是否为空数组
        new_time = getTime();
        obj_result_list_ = current_list;
    }
    else {
        new_time = getTime();
        hungrian.init_para(max(obj_result_list_.size(), current_list.size()), 0.9); // 匈牙利算法的参数使用前必须初始化
        hungrian.Hungrain(obj_result_list_, current_list);

        // obj_result_list_ 非空，通过匈牙利算法匹配结果，将结果输入到跟踪器中
        std::vector<Object>::iterator it = obj_result_list_.begin();
        for (int i = 0; it != obj_result_list_.end(); it++, i++) {
            if (hungrian.cx(i) != -1) {
                // 匹配到目标，更新目标
                it->tracking_.process_measurement(current_list[(int)hungrian.cx(i)].pkg_);
                it->update_obj_data();
            }
            else {
                // 未匹配到目标，only使用线性预测更新目标，同时跟踪器存在计数 +1
                it->tracking_.history_ += 1; // 此时不可以删除，会影响iterator
                it->tracking_.process_measurement(new_time - _g_time);
                it->update_obj_data();
            }
        }
        // 当两次未匹配成功时，删除跟踪器
        auto erase_it = obj_result_list_.begin();
        while (erase_it != obj_result_list_.end()) {
            erase_it = find_if(obj_result_list_.begin(), obj_result_list_.end(), [&](const Object &obj) {
                return obj.tracking_.history_ == 2;
            });

            if (erase_it != obj_result_list_.end())
                obj_result_list_.erase(erase_it);
        }

        // obj_result_list_ 非空，将当前被匹配上的新目标加入到，检测结果中去，并赋予新的跟踪器，用于下次匹配检测
        for (int i = 0; i < current_list.size(); i++) {
            if (hungrian.cy(i) == -1)
                obj_result_list_.push_back(current_list[i]);
        }
    }
    obj_print(obj_result_list_);
    std::cout << "---------- delta time = " << new_time - _g_time << std::endl;
    _g_time = new_time;
}
