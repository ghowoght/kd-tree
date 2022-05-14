/**
 * @file kdtree.hpp
 * @brief 基于pcl容器实现的kdtree
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 2.0
 * @date 2022-05-14
 * 
 * @copyright Copyright (c) 2022  WHU-I2NAV
 * 
 */

#ifndef _KDTREE_HPP
#define _KDTREE_HPP
#include <vector>
#include <algorithm>
#include <memory>
#include <cmath>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define DIM 3 // KD-Tree维数，PCL容器维数为3维

using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

class KDNode{
using KDNodePtr = std::shared_ptr<KDNode>;
public:
    int dim;            // 维数
    int split;
    int count;          // 子节点总数
    PointType point;
    KDNodePtr left;     // 左子树
    KDNodePtr right;    // 右子树
public:
    KDNode(PointType& point, int split, KDNodePtr left, KDNodePtr right)
        : point(point), split(split), left(left), right(right), count(0){ }

};

class KDTree{
using KDNodePtr = std::shared_ptr<KDNode>;
private:
    KDNodePtr root = nullptr;
    double alpha = 0.7; // 不平衡度阈值，超过这个阈值就拍平重建

    PointCloud::Ptr cloud_;

public:
    KDTree()  = default;
    ~KDTree() = default;

    KDTree(PointCloud::Ptr& cloud) : cloud_(cloud){
        if(cloud->empty()){
            root = nullptr;
            return;
        }
        root = make_tree(cloud->begin(), cloud->end(), cloud->size(), 0);
    }

    PointType get_nearest_linear_search(const PointType& point){
        auto best = (*cloud_)[0];
        auto best_dist = get_distance(point, best);
        for(auto& p : (*cloud_)){
            auto curr_dist = get_distance(point, p);
            if(curr_dist < best_dist){
                best = p;
                best_dist = curr_dist;
            }
        }
        return best;
    }

    PointType get_nearest(const PointType& point){
        auto best = root;
        auto next = root;
        auto best_dist = get_distance(point, best->point);
        std::vector<KDNodePtr> near_nodes;
        
        // 搜索函数：以二分搜索的方式从当前节点搜索到叶子节点
        auto search_to_leaf = [&](KDNodePtr& next){
            while(next != nullptr){
                near_nodes.push_back(next);
                auto next_dist = get_distance(point, next->point);
                if(next_dist < best_dist){
                    best = next;
                    best_dist = next_dist;
                }
                if(point.data[next->split] <= next->point.data[next->split])
                    next = next->left;
                else next = next->right;
            }
        };

        // 从根节点往下搜寻，直到找到叶子节点
        search_to_leaf(next);
        
        // 回溯
        while(near_nodes.size() != 0){
            next = *(near_nodes.end() - 1);
            near_nodes.pop_back();
            
            // 判断以查询点为球心的超球面与分割面是否相交
            if(fabs(point.data[next->split] - next->point.data[next->split]) < best_dist){
                // 相交则需要进入另一分支
                if(point.data[next->split] <= next->point.data[next->split])
                    next = next->right;
                else next = next->left;
                // 对另一子树进行搜寻，直到找到叶子节点
                search_to_leaf(next);
            }
        }

        return best->point;
    }

    PointCloud get_range(PointType& point, double r){
        PointCloud bests;
        auto best = root;
        auto next = root;
        
        auto best_dist = get_distance(point, best->point);
        std::vector<KDNodePtr> near_nodes;

        // 搜索函数：以二分搜索的方式从当前节点搜索到叶子节点
        auto search_to_leaf = [&](KDNodePtr& next){
            while(next != nullptr){
                near_nodes.push_back(next);
                auto next_dist = get_distance(point, next->point);
                if(next_dist < r){
                    if(next_dist < best_dist){
                        best = next;
                        best_dist = next_dist;
                    }
                    bests.push_back(next->point);
                }
                if(point.data[next->split] <= next->point.data[next->split])
                    next = next->left;
                else next = next->right;
            }
        };

        // 从根节点往下搜寻，直到找到叶子节点
        search_to_leaf(next);
        
        // 回溯
        while(near_nodes.size() != 0){
            next = near_nodes.back();
            near_nodes.pop_back();
            
            // 判断以查询点为球心的超球面与分割面是否相交
            if(fabs(point.data[next->split] - next->point.data[next->split]) < r){
                // 相交则需要进入另一分支
                if(point.data[next->split] <= next->point.data[next->split])
                    next = next->right;
                else next = next->left;
                // 对另一子树进行搜寻，直到找到叶子节点
                search_to_leaf(next);
            }
        }

        return bests;
    }

    PointCloud get_range_linear(PointType& point, double r){
        PointCloud bests;
        for(auto& p : (*cloud_)){
            auto curr_dist = get_distance(point, p);
            if(curr_dist < r){
                bests.push_back(p);
            }
        }
        return bests;

    }
    void insert(PointType& point){ // 使用替罪羊树的方法动态插入节点
        auto node_new = std::make_shared<KDNode>(point, 0, nullptr, nullptr);
        if(root == nullptr)
            root = node_new;
        else
            insert(root, node_new);
    }

private:
    void insert(KDNodePtr& node, KDNodePtr& node_new){
        node->count++;
        if(node_new->point.data[node->split] <= node->point.data[node->split]){
            if(node->left == nullptr){
                node->left = node_new;
            }
            else{
                insert(node->left, node_new);
            }
            if(node->left->count > alpha * node->count){ // 不平衡，拍平重建
                rebuild(node);
            }
        }
        else{
            if(node->right == nullptr){
                node->right = node_new;
            }
            else{
                insert(node->right, node_new);
            }
            if(node->right->count > alpha * node->count){ // 不平衡，拍平重建
                rebuild(node);
            }
        }

    }

    void rebuild(KDNodePtr& node){
        PointCloud points;
        pre_order_traversal(node, points);

        node = make_tree(points.begin(), points.end(), points.size(), node->split);
    }
    void pre_order_traversal(KDNodePtr& curr_node, PointCloud& points){
        if(curr_node == nullptr) 
            return;
        points.push_back(curr_node->point);

        pre_order_traversal(curr_node->left, points);
        pre_order_traversal(curr_node->right, points);
    }

private:
    KDNodePtr make_tree(const typename PointCloud::iterator& begin,
                        const typename PointCloud::iterator& end,
                        int length,
                        int split)
    {
        if(begin == end) return nullptr;

        // 按照指定split进行排序(升序)
        if(length < 100000) // 测试发现数量较少时用stl库会比较快
            std::sort(begin, end, [&split](PointType& a, PointType& b){ return a.data[split] < b.data[split]; });
        else
            quick_sort(begin, end, length, split);

        int mid = length / 2; // 中位数索引

        auto node = std::make_shared<KDNode>(
                        *(begin + mid),
                        split,
                        make_tree(begin, begin + mid, mid - 1, (split + 1) % DIM),
                        make_tree(begin + mid + 1, end, length - mid - 1, (split + 1) % DIM)
                        );
        node->count = length - 1;
        return node;
    }

    inline double get_distance(const PointType& a, const PointType& b){
        return sqrt((a.x - b.x) * (a.x - b.x)
                    + (a.y - b.y) * (a.y - b.y)
                    + (a.z - b.z) * (a.z - b.z));
    }

    void quick_sort(const PointCloud::iterator& begin,
                    const PointCloud::iterator& end, 
                    int low, 
                    int high, 
                    int split, 
                    int midIdx, 
                    bool& flag)
    {
        // 分治lambda表达式
        auto paritition = [](const PointCloud::iterator& begin,
                            const PointCloud::iterator& end, 
                            int low, 
                            int high, 
                            int split
        ){    
            auto pivot = *(begin + low);
            while(low < high){
                while(low < high && (*(begin + high)).data[split] >= pivot.data[split])
                    high--;
                *(begin + low) = *(begin + high);
                while(low < high && (*(begin + low)).data[split] < pivot.data[split])
                    low++;
                *(begin + high) = *(begin + low);
            }
            *(begin + low) = pivot;
            return low;
        };

        if(low < high && !flag){
            auto pivot = paritition(begin, end, low, high, split);
            if(pivot == midIdx){
                flag = true;
                return;
            }
            quick_sort(begin, end, low, pivot - 1, split, midIdx, flag);
            quick_sort(begin, end, pivot + 1, high, split, midIdx, flag);
        }
    }

    // 使用快速排序将原始数据分成左右两组，并返回中点
    PointType quick_sort(const PointCloud::iterator& begin,
                        const PointCloud::iterator& end, 
                        int length,
                        int split)
    {
        bool flag = false;
        quick_sort(begin, end, 0, length - 1, split, length / 2, flag);
        return *(begin + length / 2);
    }

public:
    // 用mermaid可视化节点树
    void show_tree(){
        PointCloud points;
        pre_order_traversal(root, points);
        for(auto& p : points)
            std::cout << "(" << p.data[0] << "," << p.data[1] << "),";
        std::cout << std::endl;
        
        std::cout << "graph TB" << std::endl;
        show_tree(root);
    }
private:
    void show_tree(KDNodePtr& node){
        if(node == nullptr) 
            return;
        // 计算索引key
        auto calc_key = [](KDNodePtr& node){
            int key = 0;
            for(int i = 0; i < DIM; i++)
                key += ((uint8_t)node->point.data[i] << (8 * i));
            return key + node->count;
        };
        
        std::cout << calc_key(node) << "((" << node->point.data[0];
        for(int i = 1; i < DIM; i++)
            std::cout << "," << node->point.data[i];
        std::cout << "))" << std::endl;
        if(node->left != nullptr){
            std::cout << calc_key(node) << "-->|split=" << node->split << "|"<< calc_key(node->left) << std::endl;
            show_tree(node->left);
        }
        if(node->right != nullptr){
            std::cout << calc_key(node) << "-->|split=" << node->split << "|"<< calc_key(node->right) << std::endl;
            show_tree(node->right);
        }
    }

};

#endif

