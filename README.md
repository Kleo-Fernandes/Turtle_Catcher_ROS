# Turtle Catcher ROS Project

This is a ROS 2-based multi-package project where a master turtle catches randomly spawned turtles in the Turtlesim simulation. It demonstrates custom nodes, message/service interfaces, parameterization, and launch configuration.
![Turtle_sim_project](https://github.com/user-attachments/assets/8e744519-0f98-4491-812c-95c454b80d3a)
---

## Packages Overview

- **turtle_sim**: Contains the `turtle_controller` and `turtle_spawner` nodes.
- **my_robot_interfaces**: Defines custom message and service types.
- **my_robot_bringup**: Launch and configuration package to start the full project with parameters.

---

## Project Description

This project simulates a turtle catching game using the `turtlesim_node`. The main turtle ("turtle1") automatically moves toward and "catches" randomly spawned turtles.

### Goal

Control the main turtle (`turtle1`) to autonomously navigate to other turtles, catch them, and remove them from the simulation.

---

## Nodes and Functionality

### 1. `turtle_sim/turtle_spawner` (Custom Node)

Handles turtle spawning, killing, and tracking of alive turtles.

- Calls `/spawn` to create new turtles at random `(x, y)` positions in `[0.0, 11.0]`
- Calls `/kill` to remove turtles
- Publishes list of alive turtles on `/alive_turtles` (`TurtleArray.msg`)
- Offers service `/catch_turtle` (`CatchTurtle.srv`) to remove a turtle by name

### 2. `turtle_sim/turtle_controller` (Custom Node)

Controls `turtle1` to catch other turtles.

- Subscribes to `/turtle1/pose` and `/alive_turtles`
- Publishes velocity commands to `/turtle1/cmd_vel`
- Implements a simple P controller to reach target turtles
- Calls `/catch_turtle` to "catch" a turtle when reached

---

## Custom Interfaces (in `my_robot_interfaces`)

### Messages

- `Turtle.msg`:
  ```msg
  string name
  float64 x
  float64 y
  ```
  




  
