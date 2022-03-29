#include "kdtree.hpp"
#include <iostream>
#include <vector>


inline double rand_mimx(double min, double max){
    auto x = rand();

    return min + x / (RAND_MAX * 1.0) * (max - min);
}

int main(int argc, char** argv){

    std::vector<point_t> points;

    for(int i = 0; i < 10; i++)
        points.push_back(point_t{rand_mimx(0, 1), rand_mimx(0, 1), rand_mimx(0, 1)});

    KDTree kdtree(points);

    point_t point({rand_mimx(0, 1), rand_mimx(0, 1), rand_mimx(0, 1)});

    auto nearest = kdtree.get_nearest(point);

    std::cout << "input         : " << point[0] << " " << point[1] << " " << point[2] << std::endl;
    std::cout << "kdtree nearest: " << nearest[0] << " " << nearest[1] << " " << nearest[2] << std::endl;
    nearest = kdtree.get_nearest_linear_search(point);
    std::cout << "kdtree nearest: " << nearest[0] << " " << nearest[1] << " " << nearest[2] << std::endl;

    return 0;
}