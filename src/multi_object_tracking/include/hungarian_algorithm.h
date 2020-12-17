#ifndef HUNGARIAN_ALGORITHM_H_
#define HUNGARIAN_ALGORITHM_H_

#include "object.h"
#include <iostream>
#include <vector>

class HungarianAlgorithm {
private:
    double threshold_;               // 3D-iou阈值
    int _nx, _ny;                    // x和y集合中顶点的个数
    int _ranks;                      // 行列数
    Eigen::MatrixXd connect_status_; // 邻接矩阵，g[i][j]为1表示有连接
    Eigen::VectorXd mk;              // DFS算法中记录顶点访问状态的数据mk[i]=0表示未访问过，为1表示访问过

public:
    Eigen::VectorXd cx; // cx[i],表示最终求得的最大匹配中，与x集合中元素Xi匹配的集合Y中顶点的索引
    Eigen::VectorXd cy; // cy[i],表示最终求得的最大匹配中，与y集合中元素Yi匹配的集合X中顶点的索引

    HungarianAlgorithm();
    ~HungarianAlgorithm();

    void init_para(int cl, double threshold);

    int path(int u, int _ny);
    int maxMatch(int _nx, int _ny);
    int Hungrain(const std::vector<Object> &current, const std::vector<Object> &predict);

    double bbox_volume(double x, double y, double z);
    double bbox_overlaps_3d(Object::obj Dobj, Object::obj predict_obj);
};

#endif // HUNGARIAN_ALGORITHM_H_
