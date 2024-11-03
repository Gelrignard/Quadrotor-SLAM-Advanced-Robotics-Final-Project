# MEAM-6200-Advanced-Robotics

This is the repository for MEAM 6200 Final Project: An Autonomous VIO-based Quadcopter.

## Project Overview

The specific steps for the Project is:

* Use an Error State Kalman Filter to combine the IMU and vision data together.

* Use Dijkstra and A* algorithm to find a passable trajectory in local map.

* Use Ramer–Douglas–Peucker algorithm to make the point set sparse.

* Use Min-Jerk Algorithm to generate a spline of flight.

* Use a non-linear geometric PD controller for the quadroter.

##  Flight Demostration

<table>
  <tr>
    <td><img src="maze.gif" width="100%" /></td>
    <td><img src="overunder.gif" width="100%" /></td>
  </tr>
</table>