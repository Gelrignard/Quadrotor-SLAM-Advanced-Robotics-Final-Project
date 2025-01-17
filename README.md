# Quadrotor-SLAM-Advanced-Robotics-Final-Project

This is the repository for MEAM 6200 Final Project: An Autonomous VIO-based Quadcopter.

## Project Overview

The figure below shows the overview of the replanning framework:

![Overview of the replanning framework](ovv.jpg)

The specific steps for the Project is:

* Use an Error State Kalman Filter to combine the IMU and vision data together.

* Use Dijkstra and A* algorithm to find a passable trajectory in local map.

* Use Ramer–Douglas–Peucker algorithm to make the point set sparse.

* Use Min-Jerk Algorithm to generate a spline of flight.

* Use a non-linear geometric PD controller for the quadroter.

##  Flight Demostration

The two figures below demostrates the performance of this project. The left one shows the quadrotor finding path and flying in a maze, and the right one shows how the quadrotor flies in a map with several changes in path direction.

<table>
  <tr>
    <td><img src="maze.gif" width="100%" /></td>
    <td><img src="overunder.gif" width="100%" /></td>
  </tr>
</table>
