## Project: Search and Sample Return - WriteUp
---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Record data from the simulator in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./my_dataset/IMG/robocam_2017_06_03_22_40_34_589.jpg
[image2]: ./misc/source_points.png
[image3]: ./misc/transform_.png
[image4]: ./calibration_images/example_rock1.jpg 
[image5]: ./misc/binary_track.png
[image6]: ./misc/rock_binary.png
[image7]: ./misc/tracks.png

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
---


#### 1. Training Data
We use the simulator in training mode and record a sequence of images (stored in `my_dataset\IMG\`) with the corresponding telemetry (`.csv` file).

![alt text][image1]

Telemetry

|Path | SteerAngle | Throttle | Brake | X_Position | Y_Position | Pitch | Yaw | Roll |
|-----|------------|----------|-------|------------|------------|-------|-----|------|
|my_dataset/IMG/robocam_2017_06_03_22_40_34_589.jpg| 0 | 0| 1 | 99.66999 | 85.58897 | 0.0002373002 | 56.70551 | -1.002861E-6 |

### Notebook Analysis
#### 1. `Process_image()` function

**1. Perspective transform **

The original image are converted to a top down view using the function `perspective transform()`

![alt text][image2]
![alt text][image3]

**2. Object detection**

The starting image is an RGB image.

**2.1. Detection of navigable terrain (track)**

We generate a binary image where all the pixels intensity for the 3 channels of the RGB are thresholded: in particular all pixels intensities above **160** are set to 1 whereas intensity < 160 are set to 0.

![alt text][image5]

This step is done in the function `color_thresh(img, rgb_thresh=(160, 160, 160))`

**2.2. Detection of obstacles**

We use the same function as for the detection of the navigable terrain. However, the returned image is inverted, so that the navigable terrain has pixel intensity of 0.

**2.3. Detection of the samples**

We transform the image to HSV color format. Then use a low/high band pass filtering scheme to keep only yellow-ish pixels.

```
def color_thresh_rock(img, lo_yellow = np.array([20,100,150]), up_yellow = np.array([100,255,250])):
    '''
    we use HSV color format and apply lo-hi pass band filter
    '''
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    #convert img to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lo_yellow, up_yellow)
    # Bitwise-AND mask and original image
    range_thresh = cv2.bitwise_and(img,img, mask= mask)
    # Index the array of zeros with the boolean array and set to 1
    color_select[np.where(range_thresh[:,:,0]!=0)] = 1
    # Return the binary image
    return color_select
```

![alt text][image4]
![alt text][image6]



2.4. Gneerate perception image

The 3 binary images, for the detection of the track, obstacles and samples, are stacked into a single image array with 3 channels.

```
def stacked_color_thresh(img, sel={'track': True, 'rocks': True, 'obstacles':True}):
    '''
    stack obstacle/rocks/track detections [ch0, ch1, ch2]

    '''
    stacked_detector = np.zeros_like(img[:,:,:])
    if sel['track']:
        stacked_detector[:,:,2] = color_thresh(img, rgb_thresh=(160,160, 160))
    if sel['rocks']:
           stacked_detector[:,:,1] = color_thresh_rock(img,lo_yellow = np.array([10,200,100]), up_yellow = np.array([50,255,190]))
    if sel['obstacles']:
           stacked_detector[:,:,0] = np.absolute( stacked_detector[:,:,2] -1)
    return stacked_detector

```

**2.5. Estimate the heading direction**

Once the binary image of the navigable terrain is extracted, we convert the pixels values to rover-centric coords `(xpix, ypix)`. 
Then, we convert into polar coordinate (distance, angle). The average of the angle value gives the heading direction for the Rover.

In order to know where the Rover is on the map, the `(xpix, ypix)` (Rover-centric) are converted to world coordinate `(x_pix_world, y_pix_world)`.

Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

![alt text][image7]

#### 3. Results:

The video of the processed image can be found here: `./output/test_mapping.mp4` 


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script)


```
def perception_step(Rover):

    image = Rover.img
    ...
    ...
    ...
    ...
    # Run for each channels
    for ch in range(3):
        threshed_ch = warped[:,:,ch]

        #if np.max(threshed_ch) != 0:
        print('channel ', ch)
        #if presence of hot pixels in this channel
        # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        if  (Rover.pitch >= Rover.thresh_pitch) & (Rover.pitch <= 360 - Rover.thresh_pitch):
            Rover.vision_image[:,:,ch] = np.zeros_like(image[:,:,ch])
        else:
            Rover.vision_image[:,:,ch] = np.multiply(threshed_ch, image[:,:,ch])

        if  (Rover.roll >= Rover.thresh_roll) & (Rover.roll <= 360 - Rover.thresh_roll):
            Rover.vision_image[:,:,ch] = np.zeros_like(image[:,:,ch])
        else:
            Rover.vision_image[:,:,ch] = np.multiply(threshed_ch, image[:,:,ch])
        # 5) Convert map image pixel values to rover-centric coords

        xpix, ypix = rover_coords(threshed_ch)
        # 6) Convert rover-centric pixel values to world coords
        x_pix_world, y_pix_world = pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale)
        # 7) Update Rover worldmap (to be displayed on right side of screen)
        Rover.worldmap[y_pix_world, x_pix_world, ch] += 255
        # 8) Convert rover-centric pixel positions to polar coordinates
        # Update Rover pixel distances and angles
        # Calculate pixel values in rover-centric coords and distance/angle to all pixels
        dists, angles = to_polar_coords(xpix, ypix)
        if ch == 1:
            Rover.sample_dists = dists
            Rover.sample_angles = angles
        elif ch == 2: #navigation track
            Rover.nav_dists = dists
            Rover.nav_angles = angles
            print('Navigation distance ', np.mean(Rover.nav_dists) )
        

    return Rover

    ```

In addition of the perception steps used in the notebook, we have added 2 additional steps:

**1.1. direction of the sample**
When using the binary image that shows only the sample, we calculate the position of the sample versus the Rover

```
if ch == 1:
            Rover.sample_dists = dists
            Rover.sample_angles = angles
```

**1.2. Mapping fidelity**

To improve the fidelity of the mapping, we set a threhold for the `roll` and the `pitch`. If the current `roll` and `pitch` of the Rover is above the threshold values (1.2), the frame is dismissed.


#### 2. The `decision_step()` function

The only part of the decision_step that was modified is for picking up the rocks:

```
    # If in a state where want to pickup a rock send pickup command
    if Rover.sample_angles is not None:
        print(Rover.sample_angles)
        if len(Rover.sample_angles) > 0:
            Rover.steer =  np.clip(np.mean(Rover.sample_angles * 180/np.pi), -15, 15)
            Rover.vel = 0.1
            if Rover.near_sample:
                Rover.vel = 0
                if Rover.vel == 0:
                    if not Rover.picking_up:
                        Rover.send_pickup = True
                        Rover.near_sample = False
                        Rover.mode = 'stop'
    print('Rover s velocity', Rover.vel)

```

What this means is that if there are `hot pixels` related to a **sample rock** in the field of vision of the Rover, the Rover would set a lower speed (0.1). Furthermore the Rover would steer in the direction of the **sampel rock**.

Once the Rover is close to the sample (`Rover.near_sample= True`), we set the velocity of the Rover to 0 and then pick the sample. Once the sample is picked, we need to make sure to update `Rover.near_sample` to `False` as there is no sample in that location now. We set the `Rover.mode='stop'`, so that if facing an obstacle or a wall, the Rover can steer to get 'unstucked'.


#### 3. Navigate and map autonomously 

The Rover was run in autonomous mode using the following specs:

- screen resolution 480*640 
- graphics quality : Good

In average FPS was 20.


Here is the link to the video.https://youtu.be/BXPac9mZE6c

The Rover was able to cover 47% of the map with a fidelity of 65%.


1. The fidelity of the mapping could be optimized by using a lower thresholding value for the *pitch* and *roll*. 

2. Mapping area

In some instances, the Rover could be stucked in a region of the map: i.e it would navigate only between point A and point B and would not explore any other area.

One possible strategy would be to have the Rover crawling along the walls rather than driving in the middle of the track. In that case, the Rover is practically certain to yield 100% mapping. 

3. Multi-modal track.

If there is an obstacle in the middle of the navigable terrain, the hot pixels (pixels with intensity 1) might be equally distributed on each side of the obstacle. Taking the average would lead the Rover directly toward the obstacle. One option is to reduce the view of the Rover to pixels that are closer.

