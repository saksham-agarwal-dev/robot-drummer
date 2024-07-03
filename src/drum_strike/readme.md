# Control Node

This package, drum_strike, handles the physical actuation / Sawyer aspect of our project.

Using EECS106A Lab 5 as a base, the .py files written in this folder are expected to be able to receive note messages from our CV software and strike the Taiko no Tatsujin drum controller based on the note messages. This aspect of our project will include path planning, by having the arm move between two different fixed positions to strike the drum. 

rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true


## Plan of Action:

### Develop the software that allows for the Sawyer robot arm to strike the drum controller

1. Receive inspiration from EECS106A Lab 5 as a starting position

2. TO BE DONE BEFORE EACH INTERACTION WITH A SAWYER ROBOT ARM: record the x, y, z positions for a strike on the center of the drum controller, and for a strike on the rim of the drum controller

    * This needs to be done before each interaction, as we are not implementing CV on the robot arm to detect the position of the drum. Instead, we are having the drum move between two fixed positions, which vary depending on where the drum is placed with respect to the Sawyer robot arm during each interaction.

3. Implement three movement functions -- one for movement to a default position, one for a red "DON" or center strike, and one for a blue "KA" or rim strike. 

    * TO BE DONE BEFORE EACH INTERACTION WITH A SAWYER ROBOT ARM: Within each of the striking position functions, update the position values to be that of which was recorded in part 2
    * TO BE DONE BEFORE EACH INTERACTION WITH A SAWYER ROBOT ARM: Determine a default position for the robot to have based on the position values recorded in part 2

    * ** This default position is necessary to ensure that the robot strikes the drum correctly, and needs to be a fair distance away from each of the striking positions.
    * ** The Sawyer robot arm should be moved to the default position after each strike
    
4. Implement an overarching function to determine the appropriate movement and call the corresponding function

    * ** This function will later be used to accept inputs from our CV software, and is necessary for calling the correct corresponding movement function.
    * ** For now, this function will accept a string to determine movement types for testing purposes.
    
5. Verify that the drum striking software moves appropriately to each position

    * Test center strikes, verifying that the Taiko no Tatsujin software is receiving these strikes as inputs
    * Test rim strikes, verifying that the Taiko no Tatsujin software is receiving these strikes as inputs
    * Ensure that the robot is moving to the default position and between strikes smoothly

    * ** If the robot is not striking with enough force, we may have to make adjustments to the default position accordingly or change the velocity of the arm (to be done with caution)

### Establish communication between the CV node and Sawyer

6. Create a package in ROS (maybe called "drum_strike", based on this folder) with the appropriate dependencies

7. Set up a subscriber node that subscribes to the same topic that our CV software is publishing to

8. Verify that the robot is receiving messages from the CV software

    * Refer to part 4. For testing purposes, disable the call of movement functions, and replace them with print statements for verification purposes. These print statements should print out the notes received from our CV software. Upon successful testing, reenable the call of movement function and disable print statements.
    
9. Verify that the robot can play Taiko no Tatsujin at a basic level

    * Test using "The Alphabet Song". This song has a 1 star difficulty (the lowest possible difficulty level) and a lower BPM, which makes it a good song to test on, as the Sawyer robot arm is rather slow and would allow us to more easily visually debug.

    * Song bpm's can be found [here](https://taikotime.blogspot.com/2012/12/songid-listing-console.html)

    * ** This song will likely be the song we use for our final presentation.