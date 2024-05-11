# HaIBot-Lab-Qlearning-with-Mobile-Robot

__Inspired by: https://github.com/lukovicaleksa/autonomous-driving-turtlebot-with-reinforcement-learning__

<Son_Nguyen: son94227@gmail.com> 

>> Vietnamese Version (English update later - Translate into English if nessesary)

---------------------------------------------------------
* Hệ thống file trong thư mục differential_haibotlab/src: 
---------------------------------------------------------
	- diffbot_landing: chứa file launch robot vi sai tích hợp cảm biến hokuyo (lidar2d), imu, kinect_v2 (tof camera) mô phỏng trong gazebo, tích hợp chương trình chạy teleop từ bàn phím;
	- differential_robot: chứa file urdf của robot vi sai;
	- Q_learning_Diffbot: gói chứa các thuật toán deep reinforcement learning đào tạo robot vi sai;
	- ekf: gói bộ lọc kalman mở rộng giúp trộn dữ liệu cảm biến;
	- mybot_description: chứa file urdf và meshs của cảm biến Hokuyo;
	- navigation: chứa các gói tiện ích cho việc điều hướng robot;
	- robot_description: chứa các file urdf và meshes của cảm biến imu, kinect_v2, các thuộc tính của robot trên gazebo dựa vào pkg turtlebot3;
	- robot_simulations: chứa các không gian, môi trường xây dựng để robot hoạt động trên trường mô phỏng gazebo dựa vào cách thiết lập của pkg turtlebot3;
	- File rgbd_cfg.rviz: lưu trữ dữ liệu visualize trên môi trường Rviz.

---------------------------------------------------------------
<1> Cách chạy file .launch khởi tạo môi trường và robot vi sai: 
---------------------------------------------------------------
	+ b1: catkin build hoặc catkin_make trên terminal để xây dựng workspace;
	+ b2: cấp source cho workspace: 
		$ source devel/setup.bash
	+ b3: chạy file .launch robot với môi trường tùy chọn: 
    	$ roslaunch diffbot_landing diff_bot.launch --> môi trường cơ bản 6x6 với đa biên dạng vật cản, robot vi sai với dải đo lidar_scan 20 mẫu/chu kỳ.
    	$ roslaunch diffbot_landing diffbot_for_Qlearn.launch --> môi trường cơ bản 6x6 với đa biên dạng vật cản, robot vi sai với dải đo lidar_scan 180 mẫu/chu kỳ.
     	$ roslaunch diffbot_landing difbot_with_less_obstacles.launch --> môi trường 5x5 với ít vật cản, phụ hợp cho việc training thuật toán và đánh giá nhanh, robot đặt vào với dải đo lidar_scan 20 mẫu/chu kỳ.
	(với thuộc tính slam_methods được gán cho phương thức slam muốn sử dụng)
	+ b4: sau khi màn hình Rviz và Gazebo hiện ra, để visualize dữ liệu, ta chọn file -> Open Config | Ctrl+O -> không gian lưu trữ differential -> src -> rgbd_cfg.rviz
	+ b5: kích vào termial đang chạy file .launch sử dụng cách phím A, S, D, W, X để điều khiển robot di chuyển.
