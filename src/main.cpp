#include "kdtree.hpp"
#include <iostream>
#include <vector>


inline double rand_mimx(double min, double max){
    auto x = rand();
    return min + x / (RAND_MAX * 1.0) * (max - min);
}
inline double get_s(clock_t start){
    return (clock() - (int)start) / (CLOCKS_PER_SEC * 1.0);
}

int main(int argc, char** argv){

    std::vector<point_t> points;
    for(int i = 0; i < 1000000; i++)
        points.push_back(point_t{rand_mimx(0, 1), rand_mimx(0, 1), rand_mimx(0, 1)});

    KDTree kdtree(points);

    point_t point({rand_mimx(0, 1), rand_mimx(0, 1), rand_mimx(0, 1)});

    
    std::cout << "total points: " << points.size() << std::endl;
    std::cout << "input         : " << point[0] << " " << point[1] << " " << point[2] << std::endl;
    std::cout << "----------- kdtree search -----------" << std::endl;
    auto startTime = clock();
    auto nearest = kdtree.get_nearest(point);
    std::cout << "The run time is: " << get_s(startTime) << "s" << std::endl;
    std::cout << "nearest: " << nearest[0] << " " << nearest[1] << " " << nearest[2] << std::endl;

    std::cout << "----------- linear search -----------" << std::endl;
    startTime = clock();
    nearest = kdtree.get_nearest_linear_search(point);
    std::cout << "The run time is: " << get_s(startTime) << "s" << std::endl;
    std::cout << "nearest: " << nearest[0] << " " << nearest[1] << " " << nearest[2] << std::endl;

    return 0;
}