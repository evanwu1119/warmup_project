# warmup_project
## Drive in square 
The task was to program the turtlebot to drive in a square path. I achieved this using regular cycles of alternating driving forward a fixed amount of time and turning 90 degrees. This was achieved using a single fucntion using a while loop that continuously changed the robot's twist at set intervals to switch between forward movement and turning. The timing of these switches was then calibrated to make sure the robot drives in essentially a square path. GIF of the resulting behavior:
![Drive in square GIF](./gifs/drive_square.gif)  
