#include "../include/hungarian_algorithm.h"
#define max(x, y) x > y ? x : y
#define min(x, y) x < y ? x : y
#define MAXN      100

HungarianAlgorithm::HungarianAlgorithm()
{
    _ranks = MAXN;    //行列数设置为MAXN
    connect_status_ = MatrixXd::Constant(_ranks, _ranks, 0);  //邻接矩阵设置为(100*100)的矩阵,元素均设为0,邻接矩阵，g[i][j]为1表示有连接
    cx = VectorXd::Constant(_ranks, -1);//cx[i],表示最终求得的最大匹配中，与x集合中元素Xi匹配的集合Y中顶点的索引
    cy = VectorXd::Constant(_ranks, -1);//cy[i],表示最终求得的最大匹配中，与y集合中元素Yi匹配的集合X中顶点的索引
    mk = VectorXd::Constant(_ranks, 0);//DFS算法中记录顶点访问状态的数据mk[i]=0表示未访问过，为1表示访问过
    threshold_ = 0.8;//3D-iou阈值
    _nx = 0;//x和y集合中顶点的个数
    _ny = 0;//x和y集合中顶点的个数
}
void HungarianAlgorithm::init_para(int cl, double threshold)
{
    _ranks = cl;
    connect_status_ = MatrixXd::Constant(_ranks, _ranks, 0);
    cx = VectorXd::Constant(_ranks, -1);
    cy = VectorXd::Constant(_ranks, -1);
    mk = VectorXd::Constant(_ranks, 0);
    threshold_ = threshold;
    _nx = 0;
    _ny = 0;
}

HungarianAlgorithm::~HungarianAlgorithm()
{
}

double HungarianAlgorithm::bbox_volume(double x, double y, double z)
{
    return x * y * z;
}

double HungarianAlgorithm::bbox_overlaps_3d(Object::obj Dobj, Object::obj predict_obj)
{
    // x orient
    double Lx_high =
        min(Dobj.x + 200 + Dobj.length / 2, predict_obj.x + 200 + predict_obj.length / 2);
    double Lx_low =
        max(Dobj.x + 200 - Dobj.length / 2, predict_obj.x + 200 - predict_obj.length / 2);
    // y
    double Ly_high =
        min(Dobj.y + 200 + Dobj.width / 2, predict_obj.y + 200 + predict_obj.width / 2);
    double Ly_low =
        max(Dobj.y + 200 - Dobj.width / 2, predict_obj.y + 200 - predict_obj.width / 2);
    // z
    double Lz_high =
        min(Dobj.z + 200 + Dobj.height / 2, predict_obj.z + 200 + predict_obj.height / 2);
    double Lz_low =
        max(Dobj.z + 200 - Dobj.height / 2, predict_obj.z + 200 - predict_obj.height / 2);

    double Lx = Lx_high - Lx_low;
    double Ly = Ly_high - Ly_low;
    double Lz = Lz_high - Lz_low;

    if (Lx < 0 || Ly < 0 || Lz < 0)
        return 0;

    // std::cout << " Lx = " << Lx << " Ly = " << Ly << " Lz = " << Lz << std::endl;

    double inter_volume = bbox_volume(Lx, Ly, Lz);
    double all_volume = bbox_volume(Dobj.length, Dobj.width, Dobj.height) +
                        bbox_volume(predict_obj.length, predict_obj.width, predict_obj.height);

    return inter_volume / (all_volume - inter_volume);
}

int HungarianAlgorithm::path(int u, int _ny)
{
    this->_ny = _ny;
    for (int v = 0; v < _ny; ++v) // 考虑所有Yi顶点v
    {                
        if (connect_status_(u, v) && !mk(v)) // Y中顶点v与u邻接，且没有访问过
        {     
            mk(v) = 1;                             // 访问v
                                                   // 如果v没有匹配，则直接将v匹配给u，如果v已经匹配了，
                                                   // 但是从cy[v],也就是从v之前已经匹配的x出发，找到一条增广路，但是这里记住这里v已经记录访问过了
                                                   // 如果第一个条件成立，则不会递归调用
            if (cy(v) == -1 || path(cy(v), _ny)) 
            { // cy[v] == -1为初始值
                cx(u) = v;                         // 把Y中v匹配给X中u
                cy(v) = u;                         // 把X中u匹配给Y中vß
                return 1;
            }
        }
    }
    return 0; // 如果不存在从u出发的增广路，则返回0
}

int HungarianAlgorithm::maxMatch(int _nx, int _ny)
{
    this->_nx = _nx;
    // 求二分图最大匹配的匈牙利算法
    int res = 0;
    cx = VectorXd::Constant(_ranks, -1);//cx[i],表示最终求得的最大匹配中，与x集合中元素Xi匹配的集合Y中顶点的索引
    cy = VectorXd::Constant(_ranks, -1);//cy[i],表示最终求得的最大匹配中，与y集合中元素Yi匹配的集合X中顶点的索引

    for (int i = 0; i < _nx; ++i) 
    {
        if (cx(i) == -1)          //存疑,有空搞清楚
            res += path(i, _ny); // 从X集合中每个没有匹配的点出发开始寻找增广路
    }
    return res;
}

int HungarianAlgorithm::Hungrain(const std::vector<Object> &current, const std::vector<Object> &predict)
{
    if (predict.size() == 0 || current.size() < predict.size())
        return 0;
    /* 3D IOU */
    for (int i = 0; i < current.size(); i++) 
    {
        for (int j = 0; j < predict.size(); j++) 
        {
            double Iou_3d = bbox_overlaps_3d(current[i].obj_, predict[j].obj_);

            if (Iou_3d >= threshold_)
                connect_status_(i, j) = 1;
        }
    }

    // for (int i = 0; i < connect_status_.cols(); i++) {
    //     for (int j = 0; j < connect_status_.rows(); j++) {
    //         if (connect_status_(i, j) == 1)
    //             std::cout << "connect_status_(" << i << ", " << j << ") = " << connect_status_(i, j) << std::endl;
    //     }
    // }

    int num = maxMatch(current.size(), predict.size());

    std::cout << "num = " << num << std::endl;
    // for (int i = 0; i < current.size(); ++i) {
    //     std::cout << "cx[" << i << "] -> " << cx(i) << std::endl;
    // }
    // std::cout << "!!!Hello Hungrain!!!" << std::endl;
    return num;
}
