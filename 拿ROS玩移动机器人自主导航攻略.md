

这个攻略是为一些有自己的硬件平台，想快速上手ROS navigaion的同学（老师 etc。。）准备的，并且假设大家已经对ROS的基本概念（进程间通讯topic service 数据类型 msg）有了基本的了解，有基本的C++/python编程技术，有基本的移动机器人技术概念。我的攻略与yuanbo she前辈的exbot_xi平台介绍有些不同，我将着眼与更深一层的讲解，期望大家能针对不同的硬件平台都能得心应手。

    首先是你需要的硬件平台：

一个可以与你上位机（运行ROS linux）通过某种硬件总线通信的移动平台。（移动平台需要能接受上位机的速度 指令，并且向上位机返回里程计数据，一般是编码器的累计值）

一个RGBD-camera （kinect xition etc。。。）或者一个激光雷达（hokuyo rplidar etc。。。）（这两个对以后的很多部分都有影响，会在后面仔细讲解）

以上是基本要求，一般移动平台类似与turtlebot或者exbot_xi都已经有非常成熟的ROS接口，大家问得比较多的也是ROS接口这部分的问题，我一直用的都是我们嵌入式同学做的底盘，我就着重讲一下这个接口怎么写。

首先是速度接口：

一般来说，导航规划层（不管是用什么自主移动的package），直接输出都是一个topic “cmd_vel”, 里面的数据类型为 geometry_msgs/Twist 这个数据类型表示的是3d空间中的速度，2d的移动机器人只会用到三个值 linear.x linear.y 与 angular.z 分别表示水平分速度，垂直分速度，与角速度。而对于移动平台来说，下位机大多接受到的是几个轮子分别的角速度（如果是封装完全的移动平台，可以跳过这一部分），我们需要将一个描述一个平面刚体的三个分速度映射成驱动部分的速度，这里涉及到一个底盘运动学解算的问题，一般来说分为差动底盘（turtlebot那种通过两个轮子转速不同来实现转弯）和全向底盘这两种，不同的底盘对应的解算公式也是不同的，详见自主移动机器人导论（Introduction to Autonomous Mobile Robots）第二章的讲解，具体公式可以搜索网上论文。

做完解算后一般情况就可以发送给下位机了，与下位机通讯的手段多种多样，则使用的编程接口也相应的不同。我用过串口与CAN总线，推荐使用c++ boost 串口与 python-can。

但是很多情况下针对不同的移动平台的下位机，有些细节需要注意：

1. 发送的频率：这个要看嵌入式部分的要求。

2. 关于速度的平滑： 对于规划层而言，即使有加速度等一些参数的限制，它输出的速度值可能还是对于下位机过于不友好（比如过大的加速减速，不定的发送频率等等），那么就在速度接口这边就要执行一个平滑的过程，turtlebot中给出了一个非常好的速度插值的包（http://wiki.ros.org/yocs_velocity_smoother）大家看情况使用。

总体来说，流程就是订阅 “cmd_vel”topic 然后将速度做处理，发送给下位机执行。

再者是里程计接口：

一般导航都会要求一个里程计数据的输入，这个可以解释为“通过编码器的转动推测轮子在时间片中的位移，进一步算出机器人整体的位移与速度”。和速度接口类似，一般移动平台返回的是轮子转角，需要做逆运动学解算，结果为机器人中心相对与计算开始的“原点”。

基本的编程教程如下（http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom）

这个部分需要注意的地方主要就是发布频率了。这个频率涉及到之后的costmap更新与坐标系的访问超时问题，之后会仔细讲解。

都准备好了之后，那就要对这几个部分进行调试，就我的经验而言，问题最多的部分是里程计部分，介绍大家一个小技巧，在rviz中显示 odometry ，然后移动机器人，观察其是否能回到原点。

硬件部分非常重要，我认为一定要在结合ROS的可视化工具进行充分的调试，要不然在之后的开发中会非常麻烦，硬件的问题比软件的问题隐蔽得多，而且难以定位。

还有就是传感器接口：

一般rgbd-camera与激光雷达都有相应的sensor driver 提供，roslaunch相应文件就可以了。

传感器一般在消息的 header中都需要相应的传感器采集坐标系，对于固定的传感器使用 http://wiki.ros.org/tf#static_transform_publisher tf中的static_transform_publisher 给定传感器与机器人中心相对位姿就可以了。

    SLAM 与 navigaion ROS 工具综述：

很多应用ROS做 自主导航的新手都不清楚ROS提供的这些工具和工具之间的关系，接下来我将总体阐述下这些。

ROS 中的重要相关部分部分：

tf ： 坐标转换库，提供实时的坐标转换查询与更改服务。 它将坐标转换表示为树形结构，而坐标转换数据的更新速度直接影响 其他节点的实时性，进而导致整个系统的运行出错，出问题大部分也是在这部分。

actionlib：提供动作（action ROS中表示一些长时间，连续性的行为）的编程接口，提供动作执行时的查看状态，终止， 抢占等一系列功能。ROS中的自主移动规划层向上的编程接口一般都是通过这个库。

（可选了解）

pluginlib ： 提供可配置的组件（配置文件就可以规定组件的添加与更换，提供了运行时改变组件的可能性）navigaion 中提供了对于多种planner的支持。

dynamic_reconfigure : 提供运行时改变算法（节点）参数的功能。

SLAM：

提到SLAM， 在社区中应用最多的应该是 Gmapping 与 hector_slam这两种2d slam方法了，其实，SLAM算法的实现不得不说的就是 这个（http://openslam.org/），这上面有几乎所有主流slam方法的C++/C实现，ROS的 gmapping就是调用的这个上面的代码。不管是视觉还是激光雷达算法都非常丰富。

机器人的导航规划部分：

我了解最深的是ROS navigation metapackage ， 一般它输入为激光雷达（使用rgbd-camera 效果不佳）与里程计数据，输出为cmd_vel 机器人在2d平面上的速度。与之类似的是hector_navigation。这一部分主要解决安全路径的规划问题。

    ROS navigation 软件框架介绍

如果要让 navigation 跑起来，有些知识我们是必须知道的。

（1） navigaion总体介绍：

navigaion总体介绍

这个图绝对是最好的。

我们可以清楚得看见之前说的navigation的输入：里程计odometry， 激光雷达或者rgbd-camera的信息sensor_topics，还有已知的先验地图（可选），坐标系变换信息，输出就是cmd_vel速度。

从软件架构角度来讲move_base类 作为navigation的逻辑核心存在。

从移动机器人体系结构来说，move_base规定了整个规划层的行为流程。

而如果要配置ROS navigation，重点就是move_base 与它组件的配置。

定位与navigation meta package的关系：

这一部分是论坛里面问得最多的。也是最不好搞清楚的部分。

首先我想先补充一些ROS tf tree的知识。

ROS 中对于多坐标系的处理是使用树型表示，在机器人自主导航中，ROS会构建这几个很重要的坐标系：

base_link: 一般位于tf tree的最根部，物理语义原点一般为表示机器人中心，为相对机器人的本体的坐标系。

odom：一般直接与base_link 相链接，语义为一个对于机器人全局位姿的粗略估计。取名来源于odometry（里程计），一般这个坐标系的数据也是来源于里程计。对于全局位姿的估计方法很多，比如在hector SLAM与导航体系中，就采用了imu数据估计全局位姿，还有很多视觉里程计的算法（visual odometry）也能提供位姿估计。原点为开始计算位姿那个时刻的机器人的位置。之

odom_combined 这个tf一般为好几种位姿估计方法的信息融合后的数据。在navigation metapackage中有 robot_pose_ekf 这个包是用扩展卡尔曼滤波算法（EKF）融合不同传感器的数据。

map: 一般与odom（或者odom_combined）相连，语义为一个经过先验（或者SLAM）地图数据矫正过的，在地图中的位姿信息。与odom同为全局坐标系。原点为地图原点（地图原点在地图相应的yaml文件中有规定）。

一个完整的ROS navigation 运行时的tf tree 如下：

navigation 运行时的tf tree

（base_footprint 坐标系不是必须的）

而通过在tf， 你就可以在程序里询问机器人在全局坐标系中的信息了。这对于路径规划是非常重要的。

在机器人定位与导航体系中，定位模块作为规划层的输入与参考数据所存在。而对于ROS navigation 体系而言，因为它先天的模块间通讯方式实现了模块间的完全解耦，所以对于导航规划层而言（具体就是move_base 这个node），什么定位方法，静态还是动态的地图，对于导航层内部几乎没有区别。在这种思想指导下，navigation metapackage 中 就有了为仿真器环境下的定位工具包fake_localization： 用于把仿真器中的位姿（就是直接吧odom 变换成map）估计直接变换成关于全局地图的定位，简化定位部分；对于动态创建的地图slam， gmapping 在ROS中提供从 odom -》map的坐标转换，也可以作为navigation 中 move_base 的输入。

从定位这部分扩展开来， 对于导航规划层来说，仿真器还是实物传回来的数据这些都无所谓。只要有相应的数据就可以执行相应功能。所以我们配置navigation的时候思路就是先把数据流接对，然后再根据自己机器人硬件与执行任务的不同修改相应的参数。

（2）navigation 各个部分的解析：

具体每个部分的参数列表在wiki上都有，我把一些比较重要的概念挑出来了。

move_base: 在之前说过了， move_base 部分作为navigation的逻辑核心。它实现了一个供上层节点调用的action 接口（通过actionlib 实现），即给定坐标信息与相应坐标位置，执行相应的安全导航动作。对于下层控制器，之前说过了输出为cmd_vel 2d速度。 它规定了这个navigation的行为。

ROS navigation 导航规划层提供一个在良好的定位条件下，安全导航到指定目标坐标的功能。

总体可以视为一个 慎思-反应 混合范式。

行为层： move_base 综合机器人状态与上层指令，给出机器人当前行为：正常导航，执行恢复动作，给上层节点返回失败，终止导航。其中恢复动作可以自己定义。

全局规划层：global_planner

局部规划层： local_planner

控制器层（一般就是之前自己写的速度发送部分）

costmap_2d ： 这一部分可看作为navigation的输入处理器。不同的传感器输入的数据差异很大（激光雷达 & RGBD-camera）通过costmap_2d ， 不同的数据被处理成统一的格式：栅格地图，权值用经过概率方法处理过的，表示空间中障碍物，未知与安全区域。生成出来的costmap则是planner的输入。

global_planner ： 为navigation的全局规划器，接受costmap生成的 global costmap 规划出从起始点到目标点的路径，为local_planner 作出参考。

local_planner ： 为navigation 的局部规划器，接受costmap 生成的local costmap 规划出速度。

recovery_behavior : 规定move_base 行为集合中处理异常情况的行为

这是主体部分。 要理解ROS navigation 最重要的部分是nav_core: 这个包里面就包含了global_planner ， local_planner 与 recovery_behavior的基类的头文件。但是极其重要。

我之前提到过的pluginlib , 而最重要的就是： 在ROS navigation中， move_base 提供的是框架，在move_base 中是通过nav_core 中规定的planner 与 recovery_behavior 的基类的接口进行调用。与具体的实现方法隔离开来。而具体采用的方法由pluginlib 根据不同参数导入。这样的实现方法使得navigation的可定制性大大增加。像base_local_planner 中就实现了两种局部路径规划方法，global_planner 实现了A* 与Dijkstra 两种方法，在navigation_experimental 中还有更多这样的实现。这赋予了这个框架很大的灵活性。通过不同的配置方法可以让navigation适应很多不同的任务。

（3）navigation 的配置

完成与嵌入式层的交互后，就可以着手配置navigation了。基本就是按照官网上这个教程把launch文件和相应的yaml文件配起来就好了（http://wiki.ros.org/navigation/Tutorials/RobotSetup）。我这里就不赘述了，我就讲一下最容易出问题的几个地方。

对于导航规划层来说，整个系统的表现与实时性息息相关，就我个人对于ROS navigation的实践来说，制约表现好坏的最重要一部分就是costmap的生成。costmap会分别生成两份，local_costmap 与 global_costmap 。这两份的参数是完全不同的。先是local_costmap ，local_planner 要求的实时性还是挺高的（特别是你把速度调高的时候），而local_costmap 所依赖的全局坐标系一般是odom，绘制costmap的时候会反复询问odom-》base_link 坐标系的信息，tf数据延迟要是大了会影响costmap，进而导致机器人planner实时性降低，机器人移动迟缓或者撞上障碍物。所以有个参数transform_tolerance一定要慎重。如果是使用静态先验地图做导航，那么全局的costmap可以选择使用static_map选项，这样的话在move_base 创建之初就会根据先验地图生成一次，以后不会再更新了。这样会节省一些计算量。

而如果采用动态地图（实时slam出来的）或者根本不使用先验地图，那可以将全局的costmap所依赖的全局坐标系也改为odom， rolling_window选项代替static选项，这样costmap就会实时更新，要注意的是这样的话你上层程序给出的目标点就不能超过rolling_window的范围。

基本调试好这些navigation就可以初步的运行起来了，而之后就是针对不同环境与不同任务对规划器进行选择与调试。

附录： 一些ROS中的视觉里程计算法：

http://wiki.ros.org/libviso2

http://wiki.ros.org/fovis_ros

amcl 包中的定位算法（adaptive monte carlo localization）与 gmapping中的基本概率方法：

《Probabilistic Robotics》



拿ROS玩移动机器人自主导航攻略(二) --by 西工大一小学生
2015年10月31日 ROS 评论 7 条 阅读 19,600 次	

(拖了巨长时间了, 现在学了点东西, 有些东西应该也讲得清楚了. )

 

上一期主要是将如何将navigation 在不同的硬件平台上跑起来,主要讨论的是软硬件接口与配置, 这一期主要着眼点在于通过主要算法原理讲解与参数解析,希望能彻底把一些应用上的问题讲清楚.

 

从移动机器人学的角度来看, ROS navigation package:

两层规划模型global && local planner , costmap_2d 统一 环境表示模型, move_base 组织导航过程中的规划与执行逻辑, 定位数据通过 与tf tree的交互得到, 最大程度上做到了与定位算法细节的解耦. 定位算法在本篇不细讲, 以后开专门的slam 的专题吧. 那就开始了.

 

    Navigation 之前要做的事情:

上期说过, navigation 部分 主要输入有 两部分,tf tree, 传感器的数据, 而tf tree 除了sensor在机器人本体上的偏移量以外主要就是定位的数据, 所以, navigation上机器人之前所要做的事情, 就是把输入彻底调试正确, 以免出现无法定位问题的情况.

    TF tree: 定位是一个很大的话题, 但是对于导航来说, 定位问题就完全用tf tree 做接口进行了封装. 首先是odom,由广义的里程计数据生成, 确保这个坐标系转换在局部是准确的对于navigation的正常运转来说是非常重要. 而不同的传感器不同算法调试又多种多样, 隔离细节来说, 最好的测试方法就是定量测试, 比如计算1米, 5米, 10米机器人移动观测量与groud truth之间的或者是做loop closure的检测, 走一个环形,看回到原点的error. 在有地图或者slam的情况下, 用于生成map-> odom 的算法也需要良好的调试, 方法和上面类似.
    Sensor data: 这个用ROS传感器驱动和通讯, 基本不会出问题, 主要传感器坐标系到本体坐标系别转错了. 关于不同的传感器对于navigation的适配这个问题放在costmap部分详细解答.

 

    Navigation 中组件:

1>costmap

首先是costmap, costmap_2d package中实现了 带权栅格地图 的环境表示方式. 作为导航架构中环境表示模块,承担了与传感器数据流交互,暴露接口给planner的任务.这一部分可以说是navigation 中最容易出问题的部分.

Costmap_2d主要流程为输入激光雷达laserscan 或者点云 pointcloud 数据(当然可以自己定义别的), 从tf tree上获取定位数据, 进行已知定位数据的建图 处理. 因为是根据已知定位数据的建图算法,假设定位数据精确. 所以如何选取全局定位坐标系,很大程度上决定了建图质量, 很多问题也源于此.

Costmap 除去参数设置和多线程调度, 已知定位数据的建图 核心步骤是raytrace过程.算法过程如下:

<1 得到当前时刻的机器人当前的全局位置pos, 传感器相对机器人中心坐标系偏移offset, 将不同的传感器数据统一处理成点云, 将传感器中心以及点云数据转换到全局坐标系.

<2 根据传感器模型,从传感器中心到点云的连线, 这一部分空间为没有障碍物的安全空间free, 点云的位置为障碍物所在位置occupied, 根据我们对于costmap中, 栅格cost的定义, 对连线上赋权值. 一般使用bresenham算法将直线离散化到栅格中.

<3 确定了free 和 occupied的栅格, 根据costmap 中对栅格权值的分类,将unkown 和inflation 等部分的权值填上, inflation 使用广度优先的方式进行扩展栅格, 将occupied 的栅格入队, 然后层层扩展,得到膨胀出去的栅格,并赋值.

costmapspec

对于 costmap 栅格cost设置, 没有考虑传感器概率建模,应该也用不着, 因为costmap设计之初是给用激光雷达的机器人使用, 激光雷达在室内使用的误差相对栅格地图的分辨率来说基本可以忽略. 而255 -0 的cost设置并没有限制我们给它新的含义, 我们定制costmap着手点也是这里.

从软件架构来讲, David.Lu 在13 – 14 年接手navigation 重新做的一些工作的很重要的一部分就是重新整理了costmap的软件框架, 通过使用costmap_layer 抽象出costmap 给栅格赋权值的部分,使costmap变得更加灵活.具体做法是将不同类型的栅格赋值过程通过pluginlib进行解耦和隔离, 每一种不同的栅格类型抽象成一层,layer,每一层可以使用不同的传感器数据输入, 不同的栅格赋值策略.David.Lu 写了一个range_sensor_layer接受超声波测距数据sensor_msgs/Range 构建costmap,social_layer, 结合people package 找到人之后将2d激光检测到障碍物中人的部分的cost值修改, 以达到机器人行为的改变. 大家可以参考一下.通过定制这一部分,可以实现costmap对不同应用情况, 不同传感器数据的适配与定制.

Costmap 中比较重要的几个参数:

<1 global_frame & transform_tolerance : costmap获取定位数据, 在tf tree上选择哪个坐标系作为建图的全局坐标系, 由 global_frame 规定. 而定位数据更新,查询tf 数据的tolerance 由 transform_tolerance规定, 如果超出了这个阈值,costmap会有warning. Navigation中, 两层planner 使用的 global frame 一般不同, 看需求来制定, 而transform_tolerance 一般要看tf tree 的完整性和计算机的性能.

<2 observation_source : 规定了costmap的输入, 一般来说,costmap主要数据来源来自于激光雷达,点云数据也可以输入.之前提到过,costmap 会统一将数据处理成点云. 而voxel_grid 参数规定了raytrace 过程是在二维平面上还是三维平面, 虽然最后结果都会被映射到二维.

<3 map_update_rate & publish_frequency : map_update_rate 规定了传感器数据到grid map的更新速率, 跟硬件性能有关, 而publish_frequency 为rviz 可视化costmap数据时, costmap的发布速率, 当使用先验地图生成static costmap, 请将这个参数调低, 大地图发布出去数据传输受不了.

<4 static_map or rolling_window : static_map 参数规定costmap是否通过先验地图直接生成, 而rolling_window 规定数据由实时传感器数据生成, 以robot_base_frame坐标系原点为滑动窗口中心, 需要规定窗口大小, 分辨率等参数. 过大的窗口与过小的分辨率对计算性能都是考验, 根据需求选择吧.

 

2>Local Planner

Navigation中规划分了两层,较为底层的是local planner,负责做局部避障的规划. 具体navigation中实现的算法是Dynamic Windows Approach, 因为速度的采样空间不同, ROS中的两种实现DWA和 trajectory rollout 有细微的差别. 这个方法最早96年提出. 具体流程如下:

<1 由移动底盘的运动学模型得到速度的采样空间.

在给定的速度加速度限制下, 在给定时间间隔下, 没有碰撞的速度为admission velocity.

(这些给定的限制都是我们需要调试的参数)

<2 在采样空间中, 我们计算每个样本的目标函数:

NF =α ⋅vel + β ⋅nf +γ ⋅Δnf +δ ⋅ goal

Vel 当前速度值

Nf  到当前目标点的相关的cost 值

Δnf 与全局路径的贴合程度的cost 值

Goal 到全局目标点的距离值.

还有一些cost可以自己定义, navigation实现中还有对最大最小障碍物距离的cost 与倾向于向前走的cost.

然后α,β,γ,δ 都是权重参数, 调节这些参数可以极大影响机器人避障行为

<3 得到期望速度, 插值成轨迹输出给move_base

2015103102421520151031024250

由以上算法流程,costmap作为检验速度采样值是否合法, 计算障碍物距离的依据存在, 所以local_planner 出了问题首先检查local_costmap 是否正常. Local planner 需要订阅里程计信息获得当前机器人移动速度, 所以保证里程计odom topic中twist部分的正确性也是非常重要的.

local planner 部分是navigation中参数最多, 最能直接影响机器人行为的部分. 调节local planner 中的 acceleration & velocity limit 可以直接控制机器人速度/加速度上限. 而设置holonomic_robot 与 y_vels 参数, 是否采样y 方向速度, 全向移动机器人可以通过调整这些参数实现平移避障等动作.

Local_planner的调试在于观察机器人行为, 而有具体的应用场景时, 适当增加与删除cost function 中的项可以取得不错的效果, 比如在机器人跟随人的动作中, 期望能控制机器人使得人一直在传感器视角之内, 适当将人的信息加入cost function 就可以达到相应效果,而不用自己重新实现一遍避障算法, 最大程度上复用navigation.

对于local planner, local costmap global frame 的选取一般是odom, odom坐标系在局部准确性满足要求而且实时性比map 更好(少经过一层处理). 一般要把rolling window选上, 因为局部避障要考虑先验地图中没有的障碍物.

3>Global Planner

全局路径规划navigation中实现的方法是基于栅格地图的cost 搜索找最优, 而我们所喜闻乐见得A*与dijkstra 只是扩展栅格路径的不同方式而已, dijstra 一般消耗时间和空间都较A*要多,但是A*找到的一般不是全局最优解,而这两种经典方法任何一本讲移动机器人路径规划的书上都有很详细的说明,一下不在赘述.值得说明的是.navigation中的这两层规划之间联系纽带就是local_planner中的cost function 有将全局路径考虑在内, 也就是说 local_planner 输出的local path会有靠近global path 的趋势, 而具体情况得看当时的速度以及障碍物情况(cost function 去最大值而global path 只是一个因素), 当然这个行为我们也是可以通过更改权值参数调整的.

对于global costmap, global frame的选取一般要看机器人需求, 如果是在已知地图中, 使用static map预生成costmap是又快又好的方法, 使用map坐标系做全局坐标系, 但是如果要执行一些探索任务, 没有先验地图, 定位数据选取就看你手上有什么类型的数据了, rolling_window也是看需求选取.

4>Move base

recovery_behaviors

这个模块负责整个navigation 行为的调度, 包括初始化costmap与planner, 监视导航状态适时更换导航策略等. 涉及到行为的控制, move_base 具体实现就是有限状态机, 定义了若干recovery_behavior , 指定机器人导航过程中出问题后的行为. 出问题有这几种情况:

<1 控制失败, 找不到合法的速度控制量输出, 一般就是local_planner 出问题了.

<2 规划失败, 找不到合法的路径, 一般就是global planner 出问题了.

<3 局部规划震荡, 局部规划器在某些特定情况下, cost function 出现局部最小, 具体表现为以一个较小的速度来回移动,这个时候机器人光靠local planner无法给出合适的速度控制量.

前两种情况, 一般就是planner 在当前的cost map中无法找到合法路径, 所以recovery_behavior一般是清除costmap 中的occupied grid,重新通过传感器数据生成costmap, 这样可以消除一些costmap 建图不准产生的杂点所导致的失败.

后一种情况, move_base 会给底盘发送一小段时间的很小的速度.通过主动移动给一个扰动逃离local planner cost function的局部最小.

在前两轮的recovery 失败后,move_base默认会采用一种更加激进的方式,原地旋转来企图清除costmap中的障碍物.所以很多navigation 失败情况之前都会原地转上好几圈, 这个行为完全可以通过move_base的参数进行更改,recovery_behavior也是通过pluginlib构建,自己定制也是完全可行.

    一些经验:

使用默认参数,只要把navigation的输入搞好, 表现一般是不会差的.

逐模块调试, 步步为营, 基于原理理解navigation才是解决问题之道.

看准需求定制navigation可以完成绝大多数室内导航任务.

LongHua 2016年05月05日 上午 11:07  -49楼  回复

local costmap和global costmap在obstacle layer并非重复，因为他们影响的planner不同！
887400 2015年12月01日 下午 8:39  -48楼  回复

更正下，上面说的某些激光雷达对于超过范围的障碍标为0，那么raytrace就无法将该障碍前面已经标记的障碍清除。这个问题在ros论坛里问得最多。典型现象就是人从机器人边上走过，但是障碍标记没有擦除。可以这样复现该bug： 在一个大厅的中央，激光范围到不了大厅的墙壁，这时从机器人身边经过，标记为障碍。下一次激光扫描数据没有人了，但是由于扫到的是墙壁，太远导致没有数据。这时raytrace就无法擦除上次扫到的人。
887400 2015年12月01日 下午 8:24  -47楼  回复

总结的很完整。ros整个导航框架规划得不错，但也有很多地方有过度设计之嫌，例如 local costmap和global costmap在obstacle layer是完全重复的，有些浪费cpu。 move_base里 global planner的线程也是完全不必要的。还有recovery设计我认为也是失败的，通过调整local planner算法，我把recovery完全去掉，一直很稳定没有任何问题。另外就是costmap在细节之处有很多小bug需要改正。例如obstacle layer层偶尔不能移除障碍，是因为某些激光雷达对于超过范围的障碍标为0，那么raytrace就无法将该障碍清除。还有inflation层的一个类似bug，会导致膨胀区域一直存在于costmap中，这时会影响global planner。总共大约有10来处这种小bug，完全调整好整个导航系统稳定运行，还是需要对源码做不少修改。

    西工大一小学生 2015年12月02日 下午 5:30  地下1层  回复

    谢谢您的回复，costmap这部分的bug我还没注意过，之前一直用hokuyo的激光，谢谢您的提醒啊，costmap部分因为换人写了，所以冗余错误难免，recovery设计我认为是有必要的，但是具体行为设计我也觉得已经实现的那个转圈问题很大。。。。我很想知道您对local planner做了哪些修改，谢谢交流哈
        887400 2015年12月02日 下午 6:06  地下2层  回复

        如您文中所写，recovery所解决的3个问题，基本上都是在给local_planner,global planner打补丁：控制失败，路径规划失败，局部震荡也可以算作是控制失败。local_planner里本身都加了震荡检测的cost，完全可以做到消除局部极值点。最后消除costmap的杂点也可以在costmap层面解决。

