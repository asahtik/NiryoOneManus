Dependencies:
- wiringpi (on arm)

<param name="goal_joint_tolerance"       type="double" value="0.0001" />
<param name="goal_position_tolerance"    type="double" value="0.0001" />
<param name="goal_orientation_tolerance" type="double" value="0.001" />

###### Robot
SSH

Username: ubuntu
Password: niryoone

cd manus_launch

./launch_manus.sh
./launch_webshell.sh
