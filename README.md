# franka_ros2
Franka ROS 2 with Unity integration

encender contenedor (1a vez): docker compose up -d
parar: docker compose stop
encender el existente: docker compose start
apagar contenedor y elminar: docker compose down
xhost +
docker exec -it franka_ros2_humble bash
colcon build --packages-select ros_tcp_endpoint

ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0

Se ha cambiado la configuracion del docker-compose.yml.
En lugar de # network_mode: "host". 	ports:
											- "10000:10000"
											- "5005:5005"


ros2 topic echo /pos_rot

ros2 topic echo /franka_robot_state --field o_t_ee.pose.position


colcon build --packages-select unity_franka_bridge

ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

ros2 run unity_franka_bridge planner_node

ros2 topic pub --once /unity/target_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'fr3_link0'}, pose: {position: {x: 0.4, y: 0.0, z: 0.6}, orientation: {x: 1.0, y: 0.0, z: 0.0, w: 0.0}}}"

Cuando conectemos el robot real, suscribir unity directamente al real, no al moveit. De esta forma real y unity iran a la vez 

				
