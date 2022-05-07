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
    int count;          // 子节点总数
    point_t point;      
    KDNodePtr left;     // 左子树
    KDNodePtr right;    // 右子树
public:
    KDNode(point_t& point, int split, KDNodePtr left, KDNodePtr right)
        : point(point), split(split), left(left), right(right), count(0){ }

};

using KDNodePtr = std::shared_ptr<KDNode>;

class KDTree{
private:
    int dim;
    KDNodePtr root = nullptr;
    double alpha = 0.6; // 不平衡度阈值，超过这个阈值就拍平重建

public:
    KDTree()  = default;
    ~KDTree() = default;

    KDTree(std::vector<point_t>& points){
        if(points.size() == 0){
            root = nullptr;
            return;
        }

        dim = points[0].size();
        root = make_tree(points.begin(), points.end(), points.size(), 0);
    }

    point_t get_nearest_linear_search(const point_t& point, std::vector<point_t>& points){
        auto best = points[0];
        auto best_dist = get_distance(point, best);
        for(auto& p : points){
            auto curr_dist = get_distance(point, p);
            if(curr_dist < best_dist){
                best = p;
                best_dist = curr_dist;
            }
        }
        return best;
    }

    point_t get_nearest(const point_t& point){
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
                if(point[next->split] <= next->point[next->split])
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
            if(fabs(point[next->split] - next->point[next->split]) < best_dist){
                // 相交则需要进入另一分支
                if(point[next->split] <= next->point[next->split])
                    next = next->right;
                else next = next->left;
                // 对另一子树进行搜寻，直到找到叶子节点
                search_to_leaf(next);
            }
        }

        return best->point;
    }

    std::vector<point_t> get_range(point_t& point, double r){
        std::vector<point_t> bests;
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
                if(point[next->split] <= next->point[next->split])
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
            if(fabs(point[next->split] - next->point[next->split]) < r){
                // 相交则需要进入另一分支
                if(point[next->split] <= next->point[next->split])
                    next = next->right;
                else next = next->left;
                // 对另一子树进行搜寻，直到找到叶子节点
                search_to_leaf(next);
            }
        }

        return bests;
    }

    std::vector<point_t> get_range_linear(point_t& point, double r, std::vector<point_t>& points){
        std::vector<point_t> bests;
        for(auto& p : points){
            auto curr_dist = get_distance(point, p);
            if(curr_dist < r){
                bests.push_back(p);
            }
        }
        return bests;

    }
    void insert(point_t& point){ // 使用替罪羊树的方法动态插入节点
        dim = point.size();
        auto node_new = std::make_shared<KDNode>(point, 0, nullptr, nullptr);
        if(root == nullptr)
            root = node_new;
        else
            insert(root, node_new);
    }
private:
    void insert(KDNodePtr& node, KDNodePtr& node_new){
        node->count++;
        if(node_new->point[node->split] <= node->point[node->split]){
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
        std::vector<point_t> points;
        pre_order_traversal(node, points);

        node = make_tree(points.begin(), points.end(), points.size(), node->split);
    }
    void pre_order_traversal(KDNodePtr& curr_node, std::vector<point_t>& points){
        if(curr_node == nullptr) 
            return;
        points.push_back(curr_node->point);

        pre_order_traversal(curr_node->left, points);
        pre_order_traversal(curr_node->right, points);
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
        node->count = length;
        return node;
    }

    inline double get_distance(const point_t& a, const point_t& b){
        double sum = 0;
        for(int i = 0; i < dim; i++){
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        }
        return sqrt(sum);
    }

public:
    // 用mermaid可视化节点树
    void show_tree(){
        std::vector<point_t> points;
        pre_order_traversal(root, points);
        for(auto& p : points)
            std::cout << "(" << p[0] << "," << p[1] << "),";
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
            for(int i = 0; i < node->point.size(); i++)
                key += ((uint8_t)node->point[i] << (8 * i));
            return key + node->count;
        };
        
        std::cout << calc_key(node) << "((" << node->point[0];
        for(int i = 1; i < node->point.size(); i++)
            std::cout << "," << node->point[i];
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

