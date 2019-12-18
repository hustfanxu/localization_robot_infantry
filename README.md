## 实时定位系统

2. ###### 依赖安装

   - openMP
   - eigen3
   - `sudo apt-get install -y libomp-dev libeigen3-dev`以安装上述依赖
   
3. ###### 离线运行

   1. `source ./devel/setup.bash`
   2. `roslaunch prm_localization wuhan_offline.launch`离线启动定位系统
   3. `rosbag play /PATH_TO_TESTBAG/test.bag --clock` 以clock模式播放数据包
   4. 初始化2~5s后可以看到rviz中定位结果

4. ###### 在线运行

   1. `source ./devel/setup.bash`

   2. （在确保激光雷达驱动已安装且测试可用后）启动激光雷达

      `roslaunch velodyne_pointcloud VLP16_points.launch`

   3. 检测`topic: /velodyne_points`是否有输出

      `rostopic echo /velodyne_points`

   4. 根据当前agv所在可能位置在`yuyao_factory.launch`中给出大致初始定位

      ```xml
      以右上坐标系（地图参考系）为参考系
      	<arg name="init_x" default="-4.1" />    初始x位置（单位：米）
          <arg name="init_y" default="6.0" />     初始y位置（单位：米）
          <arg name="init_yaw" default="-1.57" /> 初始yaw朝向（单位：弧度）
      红x轴 绿y轴 蓝z轴
      ```
      
   5. 启动实时定位系统`roslaunch prm_localization wuhan.launch`
   
   6. 可以在rviz中查看到当前定位，并看到红色（激光）里程计信息
   
5. ###### 进阶设置

   激光定位计设置`prm_localization wuhan.launch`

   ```xml
   <?xml version="1.0"?>
   <launch>
   <!--argument-->
       <arg name="points_topic" default="/velodyne_points" />激光点云订阅话题
       <arg name="map_tf" default="odom" />地图坐标系名称
       <arg name="base_lidar_tf" default="velodyne" />激光坐标系名称
       <arg name="kf_odometry" default="/karman_filter_odom"/>滤波器里程计发布话题
       <arg name="base_foot_tf" default="base_footprint"/>agv控制中心坐标系名称（由于未标定激光雷达与控制中心相对位置，故暂不可用）
       <arg name="init_x" default="-4.1" />初始x
       <arg name="init_y" default="6.0" />初始y
       <arg name="init_yaw" default="-1.57" />初始yaw
       <arg name="lidar_height" default="0.0"/>
       <arg name="trim_low" default="-0.4" />以雷达中心为高度0，裁剪其trim_low下点云
       <arg name="trim_high" default="7" />以雷达中心为高度0，裁剪其trim_high上点云
       <arg name="radius" default="20.0" />局部地图半径
       <arg name="mapUpdateTime" default="3" />局部地图更新时间
      <arg name="global_map_pcd_path" default="$(find prm_localization)/data/shunyu_factory_half.pcd" />全局地图路径
       <!--regis para-->
       <arg name="use_GPU_ICP" default="false"/>是否使用GPU
       <arg name="downsample_resolution" default="0.10" />地图降采样体素大小，单位m
       <arg name="TransformationEpsilon" default="0.01" />电云匹配收敛判断阈值
       <!--ndt-->
       <arg name="ndt_resolution" default="1.0" />匹配算法分辨率
       <!--filter-->
       <arg name="farPointThreshold" default="20" />去除当前远点信息
       <arg name="nearPointThreshold" default="0.98" />去除当前车体信息
       <arg name="manager_name" default="localization_nodelet_manager"/>nodelet_manager命名
       <!-- lidar_predict -->
       <arg name="lp_odom_rate" default="0"/>激光预测结果输出频率
   
   ```

   