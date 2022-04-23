#include "kdtree.hpp"
#include <iostream>
#include <vector>
#include <algorithm>


template<typename T>
inline T rand_mimx(T min, T max){
    auto x = rand();
    return min + x / (RAND_MAX * 1.0) * (max - min);
}
inline double get_s(clock_t start){
    return (clock() - (int)start) / (CLOCKS_PER_SEC * 1.0);
}
#define SAMPLE_NUM 1000000
#define INSERT_NUM 100000
int main(int argc, char** argv){

    std::vector<point_t> points;
    for(int i = 0; i < SAMPLE_NUM; i++)
        points.push_back(point_t{rand_mimx<double>(0, 1), rand_mimx<double>(0, 1), rand_mimx<double>(0, 1)});

    // 构建KD-Tree
    auto startTime = clock();
    KDTree kdtree(points);
    std::cout << SAMPLE_NUM << " points' building time is: " << get_s(startTime) << "s" << std::endl;
    
    for(int i = 0; i < INSERT_NUM; i++)
        points.push_back(point_t{rand_mimx<double>(0, 1), rand_mimx<double>(0, 1), rand_mimx<double>(0, 1)});
    // 插入新节点
    startTime = clock();
    std::for_each(points.begin() + SAMPLE_NUM, points.end(), [&](point_t& p){
        kdtree.insert(p);
    });
    std::cout << INSERT_NUM << "  points' insert time is  : " << get_s(startTime) << "s" << std::endl;

    // 待查询点
    point_t point({rand_mimx<double>(0, 1), rand_mimx<double>(0, 1), rand_mimx<double>(0, 1)});
    std::cout << "input         : " << point[0] << " " << point[1] << " " << point[2] << std::endl;
    
    // 最近邻搜索
    std::cout << "----------- kdtree search -----------" << std::endl;
    startTime = clock();
    auto nearest = kdtree.get_nearest(point);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;
    std::cout << "nearest: " << nearest[0] << " " << nearest[1] << " " << nearest[2] << std::endl;

    std::cout << "----------- linear search -----------" << std::endl;
    startTime = clock();
    nearest = kdtree.get_nearest_linear_search(point, points);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;
    std::cout << "nearest: " << nearest[0] << " " << nearest[1] << " " << nearest[2] << std::endl;
    
    // 邻域搜索
    auto r = 0.2; // 搜索半径
    std::cout << "----------- kdtree search -----------" << std::endl;
    startTime = clock();
    auto nears = kdtree.get_range(point, r);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;
    std::cout << "total num: " << nears.size() << std::endl;

    std::cout << "----------- linear search -----------" << std::endl;
    startTime = clock();
    nears = kdtree.get_range_linear(point, r, points);
    std::cout << "The run time is: " << get_s(startTime) << " s" << std::endl;    
    std::cout << "total num: " << nears.size() << std::endl;



    // kdtree.show_tree();
    return 0;
}