

ROS2驱动程序
==============

机械臂的ROS 2驱动程序。已在ROS 2 Iron（Ubuntu 22.04）上进行测试。

概述
--------
[](about:blank#overview)
*   **ar\_description**
    *   臂和伺服夹爪的硬件描述urdf。
*   **ar\_hardware\_interface**
    *   基于ros2_control框架的臂和伺服夹爪驱动程序的ROS接口。
    *   管理关节偏移、限制以及关节和执行器消息之间的转换。
    *   处理与微控制器的通信。
*   **ar\_moveit\_config**
    *   运动规划的MoveIt模块。
    *   通过Rviz控制臂和伺服夹爪。
*   **ar\_gazebo**
    *   Gazebo中的模拟。

安装
------------
[](about:blank#installation)
*   安装适用于Ubuntu 22.04的ROS 2 Iron
*   安装ros开发环境、安装rosdepc工具：
```
 wget http://fishros.com/install -O fishros && . fishros
```
   *   创建 ROS 2工作区：
```
mkdir -p ~/ar4_ros2/src && cd "$_"
```
*   将文件拷贝到到工作区 `src` 中：

   工作区目录应该是这样的：
```
  ar4_ros2
  +-- src
  |   +-- ar_hardware
  |   +-- ar_description
  |   +-- ...
```
*   安装工作空间依赖项：
```
    rosdepc install --from-paths . --ignore-src -r -y
```
   
*   构建工作空间：
```
    colcon build
```
   
*   Source 源工作空间：
   
```
    source install/setup.bash
```

*  可以将其添加到 .bashrc 中，以便在每次打开新终端时自动运行：
```
  echo "source ~/ar4_ros2/install/setup.bash" >> ~/.bashrc
```
*   如果尚未执行，请启用串行端口访问：
```shell
    sudo addgroup $USER dialout
 ```
   
    需要注销并重新登录才能使更改生效。
### 固件烧录
[](about:blank#firmware-flashing)

ar_microcontrollers中提供的Teensy和Arduino Nano草图与默认硬件兼容。要将其刷新，请按照AR4机器人设置中指定的相同过程进行操作。
### [可选]在Docker容器中运行
[](about:blank#optional-running-in-docker-container)
提供了一个docker容器和运行脚本，可用于运行机器人和任何GUI程序。需要NVIDIA GPU以及NVIDIA容器工具包。然后，可以使用以下命令启动docker容器：
```
docker build -t ar4_ros_driver .
./run_in_docker.sh
```
使用
-----
[](about:blank#usage)
需要运行两个模块：
1.  臂模块 - 可用于真实世界或模拟臂的模块
   
    *   要控制真实世界的臂，需要运行ar_hardware_interface模块
    *   对于模拟臂，需要运行ar_gazebo模块
    *   两个模块中的任意一个都会加载MoveIt所需的硬件描述
2.  MoveIt模块 - ar_moveit_config模块提供了MoveIt接口和RViz GUI。
   
各个模块的多种使用案例和运行指令如下所述：
* * *
### 在RViz中进行MoveIt演示
[](about:blank#moveit-demo-in-rviz)
如果不熟悉MoveIt，建议从这个开始，以便在RViz中探索MoveIt的规划。这里既没有真实世界的也没有模拟的臂，只有一个模型在RViz中进行可视化。

演示启动文件将加载机器人描述、MoveIt接口和RViz。
```shell
ros2 launch ar_moveit_config demo.launch.py
```
* * *
### 使用MoveIt在RViz中控制真实世界的臂
[](about:blank#control-real-world-arm-with-moveit-in-rviz)
使用MoveIt在RViz中控制真实世界的臂
启动ar_hardware_interface模块，该模块将加载配置和机器人描述：
```shell
ros2 launch ar_hardware_interface ar_hardware.launch.py \
  calibrate:=True
```
可用启动参数：
*   `calibrate`: 是否对机器人臂进行校准（确定每个关节的绝对位置）。
*   `include_gripper`: 是否包含伺服夹爪。默认值为：`include_gripper:=True`.
*   `serial_port`: Teensy板的串行端口。默认值为：`serial_port:=/dev/ttyACM0`.
*   `arduino_serial_port`: Arduino Nano板的串行端口。默认值为 `arduino_serial_port:=/dev/ttyUSB0`.
⚠️📏  注意：在向Teensy板刷新固件以及为机器人臂和/或Teensy板断电并重新上电后，需要进行校准。校准可以在后续运行中跳过，方法是使用 `calibrate:=False`.

启动MoveIt和RViz：
```shell
ros2 launch ar_moveit_config ar_moveit.launch.py
```
可用启动参数：
*   `include_gripper`: 是否包含伺服夹爪。默认值为：`include_gripper:=True`.
*   `use_sim_time`: 使MoveIt使用模拟时间。仅在与Gazebo一起运行时应启用。默认值为： `use_sim_time:=False`.

现在，可以在RViz中进行规划并控制真实世界的臂。关节命令和关节状态将通过硬件接口进行更新。
* * *
### 使用MoveIt在Gazebo模拟器中用RViz控制模拟臂
[](about:blank#control-simulated-arm-in-gazebo-with-moveit-in-rviz)
启动 `ar_gazebo` 模块，该模块将启动Gazebo模拟器并加载机器人描述：
```shell
ros2 launch ar_gazebo ar_gazebo.launch.py
```
启动MoveIt和RViz：
```shell
ros2 launch ar_moveit_config ar_moveit.launch.py use_sim_time:=true include_gripper:=True
```
现在，可以在RViz中进行规划并控制模拟臂。
 
