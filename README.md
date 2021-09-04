# Surena Humanoid Robot Simulation

<b>&#2705 Description</b>
<p>This repository contains the simulation files of the Surena Humanoid Robot in <a href="https://github.com/choreonoid">Choreonoid</a>. A ROS node has been implmented in trajectoy_planner package that can be used for the real robot as well. <br>Surena is an ongoing project at the Center of Advanced Systems and Technology (<a href="http://www.castech.ir/">CAST</a>). <br> Follow us in Instagram: <a href="https://www.instagram.com/surena_humanoid/?hl=en">surena_humanoid </a> &#1F60A</p>
<br>
<b>&#2753 How To Run?</b>
<p>&#8505 Make sure you have already installed <a href="https://ros.org">ROS</a> and <a href="https://choreonoid.org/ja/manuals/latest/ros/index.html">Choreonoid</a></p>
<ul>
  <li>clone the repository in the workspace of Choreonoid</li>
  <li>Build all packages```catkin build```</li>
  <li>Run Ros master ```roscore```</li>
  <li>Run Trajectory Planner Node ```rosrun trajectory_planner trajectory_planner```</li>
  <li>Open Choreonoid ```rosrun choreonoid choreonoid_ros choreonoid```</li>
  <li>Load project file surenaiv.wrl</li>
  <li>Start AISTSimulator</li>
</ul>

<br>
<img class="center" src="https://github.com/CAST-Robotics/surena_choreonoid_ros/blob/main/SurenaIV_Online.gif" align="center"/>



