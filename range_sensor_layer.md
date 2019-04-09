rospack plugins --attrib=plugin costmap_2d  

pedestrian_layer-pedestrian-layer /data/nucsim/docker_share/catkin_ws/src/pedestrian_layer-pedestrian-layer/costmap_plugins.xml

virtual_obstacles /data/nucsim/docker_share/catkin_ws/src/virtual_obstacles-master/costmap_plugins.xml

range_sensor_layer /data/nucsim/docker_share/catkin_ws/src/navigation_layers-indigo/range_sensor_layer/costmap_plugins.xml

social_navigation_layers /data/nucsim/docker_share/catkin_ws/src/navigation_layers-indigo/social_navigation_layers/costmap_plugins.xml

costmap_2d /data/nucsim/docker_share/catkin_ws/src/navigation-indigo-devel/costmap_2d/costmap_plugins.xml

此方法仅适用于采用源码安装的navigation包。

一、下载sonar_layer的代码：https://github.com/DLu/navigation_layers

        实际只需要其中的range_sensor_layer

        放到工作空间catkin_make需要的地方。

二、将plugin插入ros系统。可以参考：http://www.cnblogs.com/W-chocolate/p/4328725.html

       实际上下载的源码配置已完善，无需做任何改动，直接编译。编译完成时range_sensor_layer已插入ros系统。

     在此只需要source一下cartkin_ws，验证 一下即可：rospack plugins --attrib=plugin costmap_2d。如果能检测到range_sensor_layer则说明安装成功。

三、配置costmap参数文件

       1、costmap_common_params.yaml

            加入sonar层：

    sonar_layer:
      ns : ( string , default : ”” ) 命名空间，用作所有topic的前缀；
      topics : (Array of strings , default : [‘/sonar’] ) 列举可以订阅的距离topic；
      no_readings_timeout : ( double , default : 0.0 ) 如果是0，该参数不起作用，否则如果层在该参数指定时间内没有收到传感器任何数据，层会给出warning并被标记为没有数据流；
      clear_threshold : ( double, default : .2 ) 概率比clear_threshold低的cell在master costmap中被标记为free空间；
      mark_threshold : ( double , default : .8 ) 概率比mark_threshold高的cell在master costmap中被标记为lethal obstacles；
      clear_on_max_reading : ( bool , default : false ) 是否将超出sonar最大距离清除。

            参数根据实际情况修改。

       2、global_costmap_params.yaml

            加入sonar层：

       plugins: 
          - {name: sonar_layer,   type: "range_sensor_layer::RangeSensorLayer"}

四、编译运行即可。可以在rviz中将sonar的local_costmap数据调出来观察机器人周围障碍情况。

       实际在navigation测试时，遇到sonar生成的costmap中，障碍物无法消除的现象。当机器人面向障碍物旋转一圈或行人绕机器人旋转一圈时，costmap中机器人周围的网格均为occupied，机器人将无法规划路径。在阅读了源码后（range_sensor_layer.cpp），发现应该对sonar数据进行处理。

    void RangeSensorLayer::processVariableRangeMsg(sensor_msgs::Range& range_message)
    {
      if (range_message.range < range_message.min_range || range_message.range > range_message.max_range)
        return;
     
      bool clear_sensor_cone = false;
     
      if (range_message.range == range_message.max_range && clear_on_max_reading_)
        clear_sensor_cone = true;
    ...
    }

        根据以上代码，程序会将超出sonar量程的数据滤除，然后当sonar检测到障碍物的距离等于max_range时，clear开关打开，costmap更新clear。

        根据以上逻辑，sonar在前方无障碍物时，其range数据应设置为与max_range一致。而一般来说在前方无障碍时，sonar的数据习惯设为无穷大或大于max_range的值。因此clear开关将永远不会打开，costmap上occupied状态的网格将无法更新。
--------------------- 
作者：cabinx 
来源：CSDN 
原文：https://blog.csdn.net/xiekaikaibing/article/details/80096033 
版权声明：本文为博主原创文章，转载请附上博文链接！

3 配置导航包参数文件，添加range_data_layer
ROS导航包中有关于cost-map的配置文件有三个, 由于只是作为测试，我之配置了local_costmap_params.yaml文件，即只是让超声波作用于局部避障规划，配置如下;

    local_costmap:
       global_frame: /odom
       robot_base_frame: /base_link
       update_frequency: 5.0
       publish_frequency: 5.0
       static_map: false
       rolling_window: true
       width: 6.0
       height: 6.0
       resolution: 0.05
       transform_tolerance: 5.0 
     
       # 需要添加的配置：
       plugins:
       # - {name: sonar,   type: "range_sensor_layer::RangeSensorLayer"}
     

4 观察新加层是否起作用

在roslaunch rbx1_nav fake_move_base_blank_map.launch中添加一个静态转换<node pkg="tf" type="static_transform_publisher" name="base_link_ultrasound_broadcaster" args="0 0 0 0 0 0 /base_link /ultrasound 100" />（根据自己的实际坐标系定/base_link /ultrasound ）

    1.roslaunch rbx1_bringup fake_turtlebot.launch
    2.roslaunch rbx1_nav fake_move_base_blank_map.launch

    在启动之前超声波发布的cpp对应的可执行文件

在rviz中打开map和Range,RobotModel后可以得到下图所示的结果：
--------------------- 
作者：qq_32115101 
来源：CSDN 
原文：https://blog.csdn.net/qq_32115101/article/details/81123212 
版权声明：本文为博主原创文章，转载请附上博文链接！
