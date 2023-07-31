# Juice Box Robot

This was my senior capstone project. In a team of five people, we designed, built, and programmed a robot to autonomously pick up a juice box, navigate through an obstacle course, and then drop it off. One other teammate and I did all of the coding for the robot in Arduino (based in C++). This robot then competed against the other teams in our class for who could complete the course the fastest and we finished first!  

### Example Video ([YouTube Video](https://youtu.be/qs6W7hU35tc))

https://github.com/CoryChilton/juice-box-robot/assets/16692102/68c55b20-7a0e-4ef4-98ba-96805e498906

https://github.com/CoryChilton/juice-box-robot/assets/16692102/4b31a41b-66ac-4713-a3f9-142d705ea888

### More Information

There were six possible pick up and drop off locations for the juice box, so the correct locations were determined immediately before the run by a dice roll. This information could only be passed to the robot via remote control, and no further user input was allowed from that point forward.  

The coding included line tracking and branch detection using four IR sensors, object and wall detection using the two front mounted ultrasonic sensors, and controlling the lift and gripper mechanisms. The most difficult part of the programming segment of this project was getting the robot to consistently make 90<sup>o</sup> turns onto the branches using the IR sensors.

### Final Design Report
Check out the full report [here](./final-design-report.pdf)
