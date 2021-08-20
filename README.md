# rokae_robot

camera在 arm 的gazebo启动方式
arm_world.launch

camera在 arm 的rviz启动方式
rokae_camera_moveit_config    moveit_planning_execution.launch

单独添加电池包，battery:rokae_robot/rokae/src/rokae_control/scripts
robot_sorting.py


添加电池包并套接，battery:rokae_robot/rokae/src/rokae_control/scripts
motion_planning.py   添加电池包时，输入字符“add”，过后按照提出 输入其它字符。若是不加载电池，空一行后回车就可以啦

深度相机，我的问题是 py2  不支持touch.
src/rokae_control/scripts/bolt_position_detector.py
拆解原语
src/rokae_control/scripts/testmotion.py



机器人运动规划数据样例集合，
https://python.hotexamples.com/zh/examples/moveit_commander/PlanningSceneInterface/-/python-planningsceneinterface-class-examples.html
