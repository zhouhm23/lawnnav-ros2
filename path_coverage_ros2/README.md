# 原址：https://github.com/nirmalka94/path_coverage_ros2/tree/main
# ROS2 区域覆盖（Path Coverage）

路径覆盖常用于需要机器人完全覆盖环境的应用，例如清扫或割草。此 ROS 包会对给定区域执行覆盖路径。覆盖区域由多边形给出，多边形的点可在 RViz 中通过 “Publish Point” 设置。当检测到最后一个点与第一个点相同时，算法会将多边形划分为若干单元（类似 Boustrophedon 细胞分解的输出），每个单元可以用简单的往返运动覆盖。生成的目标点随后会交给导航栈：

![path coverage demonstration](./images/path_coverage.gif)

## 更新
* 覆盖区域（多边形）需更智能地排序（旅行商问题）。
    解决：多边形列表已重新排序。共享边/点或在某一阈值距离内的多边形被视为邻居。
    后续：可以采用其他计算多边形间距离的方法，我当前使用的是欧几里得距离，其他更适合多边形的距离度量也可尝试。
* 存在非常小的多边形会被规划（几乎只有两点）。
    解决：根据多边形面积进行阈值裁剪/删除，这样也能缩短整体覆盖时间。
* 本地规划器在航点之间倾向于重新规划：机器人实际行驶的路径可能不是直线，不利于清扫类机器人。
    解决：在生成的航点之间可自动插入一定数量的中间航点，以形成更平滑、更接近直线的路径段。

## 依赖
- ROS2 Humble 或 Galactic（已在 Humble 上测试）
- python-shapely
- python-numpy
- 用于 Boustrophedon 分解的 ruby：sudo apt-get install ruby-full

## 使用方法
1. 启动 path coverage：  
   终端（ROS2）：source /opt/ros/humble/setup.bash; source ros2_ws/install/setup.bash; ros2 launch path_coverage path_coverage.launch.py
2. 打开 RViz，添加一个 Marker 插件，并将 topic 设置为 "path_coverage_marker"
3. 在地图上想好机器人需要覆盖的区域
4. 在 RViz 顶部点击 *Publish Point*
5. 在区域上点击每个角点（1 次）
6. 重复步骤 5 直到边界点全部选完，之后将看到一个多边形
7. 最后闭合点的位置应接近第一个点
8. 检测到闭合点后，机器人开始覆盖该区域

## ROS 节点
### path_coverage_node.py
该节点执行 Boustrophedon 分解，计算往返覆盖路线，并将航点写入 .yaml 文件。

#### 输入：订阅的话题
* "/clicked_point" - RViz 中点击的点
* "/global_costmap/costmap" - 用于检测路径中的障碍
* "/local_costmap/costmap" - 用于检测路径中的障碍

#### 输出：
* "pose_output.yaml" - 位于用户主目录，包含生成的航点。

### 参数
* boustrophedon_decomposition (bool, 默认: true)  
  是否执行 Boustrophedon 细胞分解，或仅生成往返运动路径。

* border_drive (bool, 默认: false)  
  是否先沿单元边界行驶一圈再进行往返覆盖。

* robot_width (float, 默认: 0.3)  
  每条路径间隔宽度（机器人宽度）。

* costmap_max_non_lethal (float, 默认: 70)  
  认为空闲的 costmap 最大阈值。

* base_frame (string, 默认: "base_link")  
  机器人的基坐标系名称。

* global_frame (string, 默认: "map")  
  全局坐标系名称。

## 作者
ROS1: Erik Andresen - erik@vontaene.de  
ROS2: Azeez Adebayo - hazeezadebayo@gmail.com



