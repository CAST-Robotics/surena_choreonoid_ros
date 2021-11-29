# Surena Humanoid Robot Simulation

<p font-size:40px>&#129302</p><b> Description</b>
<p>This repository contains the simulation files of the Surena Humanoid Robot in <a href="https://github.com/choreonoid">Choreonoid</a>. A ROS node has been implmented in trajectoy_planner package that can be used for the real robot as well. <br>Surena is an ongoing project at the Center of Advanced Systems and Technology (<a href="http://www.castech.ir/">CAST</a>). <br> Follow us in Instagram: <a href="https://www.instagram.com/surena_humanoid/?hl=en">surena_humanoid </a> </p>

---
<p font-size:40px>&#10067</p><b> How To Run?</b>
<p>&#9888 Make sure you have already installed <a href="https://ros.org">ROS</a> and <a href="https://choreonoid.org/ja/manuals/latest/ros/index.html">Choreonoid</a></p>

  * clone the repository in the workspace of Choreonoid
  * Build all packages `catkin build`
  * Run Ros master `roscore`
  * Run Trajectory Planner Node `rosrun trajectory_planner trajectory_planner`
  * Open Choreonoid `rosrun choreonoid_ros choreonoid`
  * Load project file surenaiv.wrl
  * Start AISTSimulator
---
<br>
<img class="center" src="https://github.com/CAST-Robotics/surena_choreonoid_ros/blob/main/SurenaIV_Online.gif" align="center"/>
