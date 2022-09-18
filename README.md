# comprobo-warmup
Olin Fall 2022 Computational Robotics Course Warmup Project  
## Introduction
TODO
## Robot Teleop
### Description
Robot teleoperation is intended to allow a user to manually control a robot. We used the keyboard as our manual input controller.

### Diagram
Below are the keybindings we created for controlling the robot:
| Key      | Direction |
| ----------- | ----------- |
| i      | forward       |
| k   | stop        |
| j   | left and forward        |
| l   | right and forward        |
| u   | rotate left        |
| o   | rotate right        |
| m   | backward        |

### Strategy

A node was created to handle reading keyboard input and transforming it into robot commands. Key presses are read and are used to reference a dictionary of robot velocities. These velocities are then sent to the robot. We chose to let the robot continue executing the last pressed key to remove the need for holding down commands. This strategy was more simple to implement and removed the need to send continuous commands to the robot. However, if connection is lost the robot will continue driving rather than stopping which could be potentially dangerous in certain applications.

## Drive Square
### Description
Drive square is a intended to direct the robot to drive in a 1m by 1m square. 

### Diagram
![Drive Square Diagram](diagrams/drive_square.jpg)
The desired behavior of the robot is shown above. 

### Strategy
There are two main ways to approach this problem: odometry and timing. We chose to use the simpler of the two and used timing the approximate distances and rotations for the robot. Instead of using the ros built-in time, we used Python's time.sleep() function. Based on the velocity commands given to the robot we set a delay that would result in the desired final position. Using time.sleep is not optimal as it does not always match up with ros time, especially in simulations when time can be manipulated. However, for this simple implementation we were able to consistently drive a square.

## Wall Follower

### Description

### Diagram

### Strategy

## Person Follower
### Description
The goal of the person follower is to direct a robot to follow a person as they walk and move away from the robot. This behavior should keep the robot at a specified distance from the person and stop when there is not a person detected. Lidar scan data is used to 

### Diagram

![Person Follower Diagram](diagrams/person_follower.jpg)
The diagram above shows our implementation of the person follower algorithm.

### Strategy
Our algorithm at a high level:
- Retrieve laser scan data within a specific field of view
- Find the centroid (shown in green) of the laser scan data
- Find a vector that will move the robot from its current position to the centroid.
- Subtract a set follow distance from the vector
- Instruct the robot to drive along the vector

Field of View  
In order to prevent the robot from following walls and other large objects rather than a person, a field of view and maximum distance was used. To do this, only specific indexes of the laser scan data was used, and any values above the maximum distance were removed.

Finding the centroid  
The laser scan data is returned in polar coordinates with the robot at the origin. In order to easily find the centroid, we converted the data to cartesian coordinates using sine and cosine. We could then take the mean of the x and y values to find the centroid. 

Instructing the robot  
After finding the centroid in cartesian it was converted back to polar coordinates. Then, a specific follow distance was removed from the magnitude of the vector. The direction and magnitude were then multiplied by corresponding p values and sent directly to the robot's velocity command. This is a proportional controller because the farther the robot is from the person centroid the larger the velocity commands will be sent. 

One improvement in the future could be to find the centroid of the laser scan without converting to cartesian coordinates. This would reduce the code complexity and potentially allow us to directly send the centroid to the robot.

## Obstacle Avoidance

### Description

### Diagram

### Strategy