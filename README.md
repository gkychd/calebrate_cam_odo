# calebrate_cam_odo
### camera has angle
~~~
    roslaunch cam_odo_cal S800.launch
    rosbag play xxx.bag
    
    rosbag发布的消息类型分别为相机的位姿信息和轮速计的里程计信息，
    其消息类型一个是自定义消息类型cam_odo_cal::cam_data、nav_msgs::Odometry，
    其中前者的消息类型是我自定义的，其在ros_tools中的txt_to_rosmsg中可以通过txt文件中的数据生成，
    后者是ros自带的消息类型，其也是通过txt_to_rosmsg读取txt文件中的里程计数据生成。
    这两个数据需要时间戳对齐。两个txt文件是通过simulation文件生成，
    用来模拟相机与轮式计的标定，该程序修改了标定的接口，使得可以直接传入rostopic运行。
