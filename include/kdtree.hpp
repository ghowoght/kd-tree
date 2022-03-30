/**
 * @file kdtree.hpp
 * @brief 
 * @author Linfu Wei (ghowoght@qq.com)
 * @version 1.0
 * @date 2022-03-29
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

using point_t = std::vector<double>;

class KDNode{
using KDNodePtr = std::shared_ptr<KDNode>;
public:
    int dim;            // 维数
    int split;
    point_t point;      
    KDNodePtr left;     // 左子树
    KDNodePtr right;    // 右子树
    KDNodePtr front;    // 父节点
public:
    KDNode(point_t& point, int split, KDNodePtr left, KDNodePtr right)
        : point(point), split(split), left(left), right(right){ }

};

using KDNodePtr = std::shared_ptr<KDNode>;

class KDTree{
private:
    int dim;
    KDNodePtr root = nullptr;
    std::vector<KDNodePtr> nodes;

public:
    KDTree()  = default;
    ~KDTree() = default;

    KDTree(std::vector<point_t> points){
        if(points.size() == 0){
            root = nullptr;
            return;
        }

        dim = points[0].size();
        root = make_tree(points.begin(), points.end(), points.size(), 0);
    }

    point_t get_nearest_linear_search(const point_t& point){
        auto best = nodes[0];
        auto best_dist = get_distance(point, best->point);
        for(auto& node : nodes){
            auto curr_dist = get_distance(point, node->point);
            if(curr_dist < best_dist){
                best = node;
                best_dist = curr_dist;
            }
        }
        return best->point;
    }

    point_t get_nearest(const point_t& point){
        auto best = root;
        auto next = root;
        auto best_dist = get_distance(point, best->point);
        std::vector<KDNodePtr> near_nodes;
        
        // 从根节点往下搜寻，直到找到叶子节点
        while(next != nullptr){
            near_nodes.push_back(next);
            auto next_dist = get_distance(point, next->point);
            if(next_dist < best_dist){
                best = next;
                best_dist = next_dist;
            }

            if(point[next->split] <= next->point[next->split])
                next = next->left;
            else next = next->right;
        }
        // 回溯
        while(near_nodes.size() != 0){
            next = *(near_nodes.end() - 1);
            near_nodes.pop_back();
            
            // 判断以查询点为球心的超球面与分割面是否相交
            if(fabs(point[next->split] - next->point[next->split]) < best_dist){
                // 相交则需要进入另一分支
                if(point[next->split] <= next->point[next->split])
                    next = next->right;
                else next = next->left;
                // 对另一子树进行搜寻，直到找到叶子节点
                while(next != nullptr){
                    near_nodes.push_back(next);
                    auto next_dist = get_distance(point, next->point);
                    if(next_dist < best_dist){
                        best = next;
                        best_dist = next_dist;
                    }

                    if(point[next->split] <= next->point[next->split])
                        next = next->left;
                    else next = next->right;   
                }
            }
        }

        return best->point;
    }
    void insert(point_t& point){ // 使用替罪羊树的方法动态插入节点
        dim = point.size();
        auto node_new = std::make_shared<KDNode>(point, 0, nullptr, nullptr);
        nodes.push_back(node_new);
        if(root == nullptr)
            root = node_new;
        else
            insert(root, node_new);
    }
    
private:
    void insert(KDNodePtr& node, KDNodePtr& node_new){ // 使用替罪羊树的方法动态插入节点
        if(node_new->point[node->split] <= node->point[node->split]){
            if(node->left == nullptr){
                node->left = node_new;
            }
            else{
                insert(node->left, node_new);
            }
        }
        else{
            if(node->right == nullptr){
                node->right = node_new;
            }
            else{
                insert(node->right, node_new);
            }
        }

    }
private:
    KDNodePtr make_tree(const std::vector<point_t>::iterator& begin,
                        const std::vector<point_t>::iterator& end,
                        int length,
                        int split)
    {
        if(begin == end) return nullptr;

        // 按照指定split进行排序(升序)
        std::sort(begin, end, [split](point_t& a, point_t& b){ return a[split] < b[split]; });

        int mid = length / 2; // 中位数

        auto node = std::make_shared<KDNode>(
                        *(begin + mid),
                        split,
                        make_tree(begin, begin + mid, mid - 1, (split + 1) % dim),
                        make_tree(begin + mid + 1, end, length - mid - 1, (split + 1) % dim)
                        );
        nodes.push_back(node);
        return node;
    }

    inline double get_distance(const point_t& a, const point_t& b){
        double sum = 0;
        for(int i = 0; i < dim; i++){
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        }
        return sqrt(sum);
    }

};

#endif

