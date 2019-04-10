https://blog.csdn.net/Chenming_Hnu/article/details/60469410
1. 准备

（1）两台电脑AB，最好在同一网段；
（2）电脑A-安装了Docker，并下载了ros镜像，电脑B下载并安装了与A中相同distribution的ROS（非镜像）；
2. 配置3个环境

（1）获取主机A和B的ip地址，我的主机A的ip是 192.168.1.102，主机B的ip是192.168.1.103；

（2）在主机A中运行ros镜像，得到ros容器（关于容器-container的理解，借用官网的解释-“A running instance of image“），在容器中运行ifconfig 得到容器在主机A中的相对ip地址，我的是“172.17.0.3 “；
这里写图片描述

f8289b197a45 代表容器ID，其实这个ID  很长，这里也只是截取了前几位便于操作

    1

（3）在主机A中的 .bashrc （home目录下的隐藏文件，crtl+H显化）文件中添加：

export ROS_HOSTNAME=192.168.1.102
export ROS_MASTER_URI=http://192.168.1.103:11311

    1
    2

其中，11311端口是下面试验的turtlesim的通讯端口。
（4）在主机B中的 .bashrc 文件中添加：

export ROS_HOSTNAME=192.168.1.103
export ROS_MASTER_URI=http://192.168.1.103:11311

    1
    2

（5）在主机A的ROS容器中（上述查看了ip）添加变量
在目录/etc/profile中添加环境变量，容器中可提供的编辑器vi, 而vim nano gedit等需要重新在容器中下载。关于vi的用法，主要记住命令模式 (command mode)、插入模式(insert mode)、底行模式(command mode)这三种模式下的操作的不同，搞混了的话，会感觉输入很奇怪。
在profile的输入内容为：

export ROS_HOSTNAME=172.17.0.3
export ROS_MASTER_URI=http://192.168.1.103:11311

    1
    2

保存 退出。
运行命令使其生效：

$ source /etc/profile

    1
    2

$ source /opt/ros/indigo/setup.bash

    1

3. 测试

（1）在主机B（装有ROS系统）中运行以下命令：
开启ros服务：

$ roscore

    1

新开一个窗口，运行turtlesim：

$ rosrun turtlesim turtlesim_ndoe

    1

（2）在主机A中测试rostopic, rosnode是否显示：
分别输入命令：

$ rostopic list

    1

$ rosnode list

    1

这里写图片描述

能显示，说明主机A中可以与ROS主机B通讯，接下来测试容器中的连接情况。

（3）在主机A中的docker的ros容器，测试rostopic, rosnode是否显示：
类似于上述命令
这里写图片描述
可以显示，说明ROS容器内部可以与远端的ROS主机通讯。
4. 关于数据传输的测试

    初步设想：传输rosbag数据，在ROS容器里面通过运行rosrun turtlesim turtle_teleop_key命令，操控键盘，能够控制远端的ROS系统中的turtlesim的移动。

尝试1：A主机-》B主机，在主机A中，不考虑容器的话，可以控制远端ROS主机B中 turtlesim中的小乌龟的移动。

尝试2：A主机中容器ROS-》A主机，（可以理解A中容器与主机A系统隔离，类似于虚拟机），在A中的容器中运行控制命令，可以控制A中主机的turtlesim的小乌龟的移动。

尝试3：A主机中的容器ROS-》（A主机）-》B主机
类似上面的控制，希望在A主机中的容器ROS中运行控制命令，控制主机B中运行的turtleism的小乌龟的移动。
在这种情况的时候，虽然在容器ROS中运行rostopic list rosnode list 都能够捕捉到B主机运行的turtlesim的相关的topic和list，但是无法控制小乌龟的移动。在输出速度的值的时候也是无反应，

$ rostopic echo /turtle1/cmd_vel

    1

在窗口中的速度并没有显示。暂时不知道原因，还在探究中，若有新发现，将会在后续的博客中写出来。

控制turtlesim中的小乌龟的移动效果应该如下：

这里写图片描述
5. 注意情况

（1）开启同一个容器的多个窗口：
通常，我们进行命令行操作的时候，要打开多个窗口，对于docker容器的操作也是如此。
开启docker容器的另外的终端框，有两种命令：

docker attach <Container's ID>

    1

docker exec -ti <Container's ID> /bin/bash

    1
    2

这里，适用于我们的操作的是第二种docker exec
对于而者的区别，我google到的解释，
这里写图片描述

从stackoverflow大概总结为下表。
命令 	描述
docker attach 容器ID 	attaching to the running process
docker exec -ti 容器ID /bin/bash 	running new things, it would be a shell

前者附在进程上，后者开新的交互shell.

（2）打开新shell后，输入rostopic, rosnode 报错 cannot find this command
在容器中，输入如下命令：

$ source /opt/ros/indigo/setup.bash 

    1

可参考：Let’s install turtlesim packages

（3）遇到 turtlesim这个package没有找到的问题。
这是因为，不同的ROS镜像中自带的package不一定是完整的，有些只是包含了核心的一些package, 缺少什么package就安装什么就可以了。

$ apt-get install ros-indigo-turtlesim 
--------------------- 
作者：忧愁莫扎特 
来源：CSDN 
原文：https://blog.csdn.net/Chenming_Hnu/article/details/60469410 
版权声明：本文为博主原创文章，转载请附上博文链接！
