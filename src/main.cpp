#include "kdtree.hpp"
#include <iostream>
#include <vector>
#include <algorithm>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define SAMPLE_NUM 1000000
#define INSERT_NUM 100000

using data_type = float;
inline data_type rand_mimx(data_type min, data_type max){
    auto x = rand();
    return min + x / (RAND_MAX * 1.0) * (max - min);
}
inline double get_s(clock_t start){
    return (clock() - (int)start) / (CLOCKS_PER_SEC * 1.0);
}

int main(int argc, char** argv){

    PointCloud::Ptr cloud(new PointCloud);
    cloud->height = 1;
    cloud->width = SAMPLE_NUM;
    cloud->points.resize(cloud->width * cloud->height);

    srand(time(NULL));

    for(int i = 0; i < SAMPLE_NUM; i++){
        (*cloud)[i].x = rand_mimx(0, 1);
        (*cloud)[i].y = rand_mimx(0, 1);
        (*cloud)[i].z = rand_mimx(0, 1);
    }

    // 构建KD-Tree
    auto startTime = clock();
    KDTree kdtree(cloud);
    std::cout << SAMPLE_NUM << " points' building time is: " << get_s(startTime) << "s" << std::endl;
    
    for(int i = 0; i < INSERT_NUM; i++){
        (*cloud).push_back(PointType(rand_mimx(0, 1), rand_mimx(0, 1), rand_mimx(0, 1)));
    }

    // 插入新节点
    startTime = clock();
    std::for_each(cloud->begin() + SAMPLE_NUM, cloud->end(), [&](PointType& p){
        kdtree.insert(p);
    });
    std::cout << INSERT_NUM << "  points' insert time is  : " << get_s(startTime) << "s" << std::endl;

    // 待查询点
    PointType point(rand_mimx(0, 1), rand_mimx(0, 1), rand_mimx(0, 1));
    std::cout << "input         : " << point.data[0] << " " << point.data[1] << " " << point.data[2] << std::endl;
    
    //////////// 最近邻搜索
    std::cout << "**********************************" << std::endl;
    std::cout << "********* nearest search *********" << std::endl;
    std::cout << "**********************************" << std::endl;
    std::cout << "----------- kdtree search -----------" << std::endl;
    startTime = clock();
    auto nearest = kdtree.get_nearest(point);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;
    std::cout << "nearest: " << nearest.data[0] << " " << nearest.data[1] << " " << nearest.data[2] << std::endl;

    std::cout << "----------- linear search -----------" << std::endl;
    startTime = clock();
    nearest = kdtree.get_nearest_linear_search(point);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;
    std::cout << "nearest: " << nearest.data[0] << " " << nearest.data[1] << " " << nearest.data[2] << std::endl;

    //////////// 邻域搜索
    std::cout << "*********************************" << std::endl;
    std::cout << "********* radius search *********" << std::endl;
    std::cout << "*********************************" << std::endl;
    auto r = 0.2; // 搜索半径
    std::cout << "----------- kdtree search -----------" << std::endl;
    startTime = clock();
    auto nears = kdtree.get_range(point, r);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;
    std::cout << "total num: " << nears.size() << std::endl;

    std::cout << "----------- linear search -----------" << std::endl;
    startTime = clock();
    nears = kdtree.get_range_linear(point, r);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;    
    std::cout << "total num: " << nears.size() << std::endl;

    // kdtree.show_tree();
    return 0;
}