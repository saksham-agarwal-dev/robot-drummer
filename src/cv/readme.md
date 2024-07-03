# Computer Vision Node

This package handles the computer vision aspect of our project.

Using the Python files from "perception" package from EECS106A Lab 8 as a base (namely, object_detector.py), the .py files written in this folder are expected to be able to receive image input (from either a camera or a video feed) and process these inputs to detect red "DON" notes and blue "KA" notes.

After detection, a message should be sent to the Sawyer arm such that it knows the next action it should take. 
** This should be done AFTER all of the CV software is complete, as this step is a migration to ROS.

To run:
```py
rosrun cv note-detection.py
```

## Plan of Action:

### Develop CV software for an arbitrary input stream (camera or video feed)
1. Receive inspiration from object_detector.py from EECS106A Lab 8 as a starting point

2. Implement a function (maybe called "retrieve_and_translate_input") that receives input from an arbitrary source, and "translates" it to be passed into main logic (part 3) 

    * ** We are splitting input retrieval and parsing into two different functions for robustness of the main logic; this function would be able to accept various types of image inputs and convert them to be of uniform type ("uniform type" meaning the output of this function will always be of the same type), allowing the output of this function to be safely used in the image parser (which will expect only ONE type of input, [cv2 image])

    * Retrieve image input from an arbitrary source
    * Convert or "translate this image input into a consistent, suitable type (namely, cv2 image)
    * Call the image parser on the converted input

    * ** Consider trimming the image such that only one note is only ever in frame before "translating" the entire image
    * ** Alternate approach: take the left most note if there is more than one note in our frame
    * ** NOTE: this function may not be entirely necessary

3. Implement a function (maybe called "detect_note") that accepts a "translated" input and parses for red "DON" notes and blue "KA" notes
        
    * ** Before coding begins, it would be beneficial to find the HSV values / windows for the red "DON" notes and blue "KA" notes respectively to be hard-coded as lower and upper bounds

    * Define upper and lower bounds for both the red and blue colors
    * Create a color mask using cv2.inRange for both red and blue using the defined upper and lower bounds
    * Determine "correlation value" (by default we're defining this to be sum, as the mask is somewhat of a cross-correlation between image and HSV bounds) for each mask
    * Return the note type based on greatest "correlation value"

    * ** Consider possibly adding support for "drumroll" and "balloon" notes. By default, we are choosing to ignore them (but they may be treated as red or blue notes depending on our bounds)
        
4. Verify CV software for an arbitrary input stream
    * Test on camera inputs. Point the camera onto the Nintendo Switch screen and verify that our software is retrieving the data stream. Verify that our software is parsing the data stream, and that the correct note is being returned (by printing the detected note to the terminal)

    * Test on video steam input. Hook up the Nintendo Switch to send a video stream to the lab computer, and then send the video stream to our CV software. Verify that our software is retrieving the data stream. Verify that our software is then parsing the data stream, and that the correct note is being returned (by printing the detected note to the terminal)


### Upon completion of the CV software, we will create a package in ROS to house our .py file(s) and modify our .py file(s) to be consistent with ROS.

5. Create a package in ROS (maybe called "note detection") with the appropriate dependencies

6. Define a new message type (maybe called "note message" or "note type")

    * ** Currently, we would be returning something like a boolean flag or a string for "red" or "blue", which is not optimal. It would be a better idea to have a new message type with two boolean variables "red" or "blue" which are true and false according to detected note. While this may seem redundant as our detection is binary (technically there are 3 possible outputs if we include "empty" or "no note"), this will come to be a more future-proof system if we decide to include support for "drumroll" and "balloon" notes. Alternatively, our new message type could just be a string, but we may run into the edge case of conflicting detected notes, but this is highly unlikely.

    * ** Consider how long a note should be detected before being sent to ROS; we don't want to overload the arm with too many messages for only one note, otherwise Sawyer will attempt to hit the drum multiple times for only one note.

7. Modify the function in part 2 to be consistent with ROS (e.g. accepting message types based on input device)

8. Modify the function in part 3 to be consistent with ROS

    * Add support for the new message type defined in part 6
    * Set up a publisher node to be used to send the parsed input to the robot / drum strike
