
\mainpage
\htmlinclude manifest.html

[TOC]

This component can be used for grasp selection. The component should be initialized with the 
available grasp databases and workcell descriptions of the gripper that is to be used and
the objects. 

The selection of a feasible grasp will be determined from quality measures and from 
collision information in the relevant online scenario of grasping.

In order to use the grasp selection component the system should be modelled in robwork
and the models should continously be updated with state variables from the real world, such
as robot joint states.




-- Example of use --

First startup a node that provides the robwork workcell models to other nodes and connect the 
joint state output of you robot to the input of the robot in robwork virtual workcell. The
robworkcell is currently defined in the caros_grasp_selection component (will be moved eventually)
and the following launch script will start a dummy service for publishing joint states to the robworkcell
which is also started and mapped in the launch script. Notice that the global parameter "/caros/workcell" 
must be set to the robwork workcell model that is used.

```
<launch>
  <param name="/caros/workcell" value="/mnt/hgfs/DATA/Data/ACAT/scene-1/ACAT-IASSES/iasses_gantry_robotiq.wc.xml"/>

  <group >
  	<!-- start the dummy joint state emitter -->
  	<node name="caros_dummy_jointstate" pkg="caros_grasp_selection" type="dummyRobotState.py" respawn="false" output="screen">
	</node>
  	
  
	<!-- start the robworkcell nodes -->
	<node name="robworkcell" pkg="caros_grasp_selection" type="robworkcell" respawn="false" output="screen">
		<param name="UR_6_85_5_A" value="/joint_states"/>
	</node>

	
  </group>
</launch>
```

Having setup the robworkcell, now we can start the grasp selection component. The arguments are which gripper
to use and the location of the grasp database.

```
./grasp_selection _gripper:=Robotiq-3 _arm:=UR-6-85-5-A _graspdb:=./db
```