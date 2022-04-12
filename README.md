# warmup_project
Evan Wu

## Drive in square 
The task was to program the turtlebot to drive in a square path. I achieved this
using regular cycles of alternating driving forward a fixed amount of time and
turning 90 degrees. This was achieved using a single function with a while loop
that continuously changed the robot's twist at set intervals to switch between
forward movement and turning. The timing of these switches was then calibrated
to make sure the robot drives in essentially a square path.
\n
![Drive in square GIF](./gifs/drive_square.gif)  

## Person follower
The task was to program the turtlebot to be able to follow around a person within 
a 3m radius of the robot. This includes dynamic following behavior if the person 
is also moving at the same time. To do this, I utilized the turtlebot's LiDAR 
scan measurements in the process_scan function to find the closest object within 3m. 
Based on the nearest object, I then proportionally control the linear and angular 
velocity based on how far away and how much angle there is, respectively so that the 
robot turns and moves towards the closest object and follows faster at greater 
distances or angles. This allows it to more effectively follow if the person is 
changing their position quickly, or is standing directly behind the robot as well. 
\n
![Person follower GIF](./gifs/person_follower.gif)

## Wall follower
The task was to program the turtlebot to be able to follow at a fixed distance 
parallel to a wall, and be able to round corners. To do this, I similarly utilized
the LiDAR measurements in process_scan to find the closest object. To make the robot
follow parallel, the minimum distance should be at 90 degrees so I set the robot to
turn proportionally based on the angular difference between the minimum direction
and 90 degrees. I also added another term to the proportional control that turned 
the robot based on the minimum distance, to allow it to keep an approximately fixed 
distance from the wall. Corner rounding behavior arises out of the parallel part, as 
the minimum distance will change and cause the robot to start turning towards the 
next stretch fo wall. 
\n
![Wall follower GIF](./gifs/wall_follower.gif)

## Challenges
The biggest challenge for me in this project was determining whether failures were
due to problems with my code or untuned parameters. If the parameters are 
unrealistic, the robot begins to behave very strangely and with more and more 
parameters it becomes difficult to discern what exactly the problem is. This came
up especially for the wall follower as I had a more complex proportional error, and 
the constant scaling parameter for each part needed to be very different. One way I
got around this was to ensure that each part was working correctly independently
(robot following parallel/turning corners, robot adjusting distance to wall) before
seeing whether they worked together. 

## Future work
I would have liked to understand how to drive in a square with odometry given more 
time, as relative control probably allows for more elegant solutions in other cases.
I also would have liked to optimize parameters and proportional control more for the 
two folowers, as the robot showed some difficulties turning 180 degrees in the person 
follower, and the wall following behavior was consistent but rather stuttery. 

## Takeaways
* How to process and manipulate robot LiDAR readings to navigate an environment. 
The ability to get consistent data of distance and angle of nearby objects over time
was much of the basis of these behaviors, and can definitely be used to produce even
more complex behaviors related to navigation. 

* How to utilize proportional control to get sensitive and realistic behaviors. 
The robot's ability to respond to the dynamically changing LiDAR readings 
necessitated a dynamic response, which was very well addressed by using 
proportional control. 
