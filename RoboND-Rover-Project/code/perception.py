import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


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


# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    yaw_rad = yaw * np.pi/180.0
    # Apply a rotation
    x_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    y_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return x_rotated, y_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):

    image = Rover.img
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst_size = 5
    bottom_offset = 6
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
    # 2) Apply color threshold to identify navigable terrain/obstacles/rock samples
        # 3 channels image: [channel0=track threshold, channel1=rock threshold, channel3=obstacle threshold]
    stacked_threshed =  stacked_color_thresh(image, sel={'track': True, 'rocks': True, 'obstacles':True})
    # 3) Apply perspective transform
    warped_orig = perspect_transform(stacked_threshed, source, destination)
    warped = warped_orig
    #Binarize all channels
    warped[np.where(warped > 0)] = 1
    #########
    # Telemetry data and constants
    ################
    pos = Rover.pos
    xpos = float(pos[0])
    ypos = float(pos[1])
    yaw = float(Rover.yaw)
    world_size = 200
    scale = 10

    print('start image process')
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