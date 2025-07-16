
演示感知端请求，生成样条路径
1、监测启动主流程， ros2 run indoors_logic control_node
2、模拟mcu接收到ros消息， ros2 run indoors_logic bizer_to_mcu_server_test
3、模拟感知client请求，ros2 run indoors_logic perception_test 0 0 0 400 0 0 800 0 0
