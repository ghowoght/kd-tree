### KD-Tree的C++实现

- [x] 构建KD-Tree
- [x] 最近邻搜索
- [x] 增量式构建KD-Tree: 再平衡
- [x] 半径搜索
- [x] 简化排序算法(1M数据构建耗时: 1.25s->0.95s)

### 测试结果
```
1000000 points' building time is: 0.935642s
100000  points' insert time is  : 0.092115s
input         : 0.229831 0.126447 0.777731
**********************************
********* nearest search *********
**********************************
----------- kdtree search -----------
The run time is: 8e-06 s
nearest: 0.222685 0.12289 0.777183
----------- linear search -----------
The run time is: 0.004096 s
nearest: 0.222685 0.12289 0.777183
*********************************
********* radius search *********
*********************************
----------- kdtree search -----------
The run time is: 0.003747 s
total num: 33231
----------- linear search -----------
The run time is: 0.005412 s
total num: 33231
```
