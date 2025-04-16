import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

#TODO: Complete the function for rotating the input `vector` around `rot_axis` by `angle_degrees`
#      Construct a 3x3 rotation matrix (no need Homogeneous) and multiply it with the input vector
#      rot_axis: "X", "Y", or "Z"
#      Return the rotated vector.
def rotate_vector(vector, angle_degrees, rot_axis = "Y"):
    rotated_vector = np.array([0.0, 0.0, 0.0])
    
    # TODO: convert angle degrees to radians


    # TODO: construct a 3x3 totation matrix using np.array based on angle and rotation axis


    # TODO: rotate the input vector by multiplying with the matrix using np.dot()

    return rotated_vector

class Camera:
    def __init__(self, view_mode = "front"):
        self.view_mode = view_mode
        # camera parameters
        self.eye_pos = np.array([0.0, 10.0, 50.0]) # initial setting for the front view
        self.look_at = np.array([0.0, 0.0, -1.0])
        self.view_up = np.array([0.0, 1.0, 0.0])

        # viewing parameters adjustable by keyboard input
        self.tilt_angle_horizontal = 0.0 # the angle (degrees) to rotate the gaze vector to the left or right
        self.tilt_angle_vertical = 0.0 # the angle (degrees) to rotate the gaze vector upward or downward
        self.zoom_distance = 0.0 # camera forward/backward distance along the gaze vector, positive or negative

    # TODO: Task 7: Switch between 3 standard view modes: front, side, and back
    #               For each view mode, pre-define camera parameters here 
    def switch_view(self):
        # TODO: Switch the current view_mode to the next in the cycle: 
        #   front -> side -> back -> (first_person) -> front -> side -> ...

        
        # Front view
        if self.view_mode == "front":
            self.eye_pos = np.array([0.0, 10.0, 50.0]) 
            self.look_at = np.array([0.0, 0.0, -1.0])
            self.view_up = np.array([0.0, 1.0, 0.0])
            

        # TODO: Side view
        elif self.view_mode == "side":

            pass # placeholder, you may delete this


        # TODO: Back view
        elif self.view_mode == "back":

            pass # placeholder, you may delete this
        
    # TODO: Task 8: Update camera parameters (eye_pos and look_at) based on the new 
    #               tilt_angle_horizontal, tilt_angle_vertical, and zoom_distance updated by key input (A, D, W, S, Q, E)
    def update_view(self):
        new_eye_pos = self.eye_pos # THIS IS A PLACEHOLDER - PLEASE DELETE THIS LINE
        new_lookat = self.look_at  # THIS IS A PLACEHOLDER - PLEASE DELETE THIS LINE

        # TODO: calculate the current gaze vector
        

        ## calculate new look-at point
        # TODO: tilt horizontally: rotate the gaze vector around SOME axis by tilt_angle_horizontal


        # TODO: tilt vertically: rotate the gaze vector around SOME axis by title-angle_vertical


        # TODO: calculate the current look-at point


        ## calculate new eye position by moving the camera along the gaze vector by zoom_distance
        # TODO: calculate the unit vector of the current gaze vector


        # TODO: calculate the new eye_position


        # return new eye position and look-at point
        return new_eye_pos, new_lookat


class Scarecrow:
    def __init__(self, version = "basic"):
        self.version = version 
        # Scarecrow body part dimensions, for both "basic" and "upgraded"
        self.head_sphere = 2.5 # radius
        self.nose_cylinder = [0.3, 0.0, 1.8] # base radius, top radius, height
        self.torso_cylinder = [2.5, 2.5, 10.0] # base radius, top radius, height
        # basic scarecrow
        self.leg_cylinder = [1.0, 1.0, 12.0] # base radius, top radius, height
        self.arm_cylinder = [1.0, 1.0, 10.0] # base radius, top radius, height
        # upgraded scarecrow
        self.upper_lower_leg_cylinder = [1.0, 1.0, 6.0] # base radius, top radius, height
        self.upper_lower_arm_cylinder = [1.0, 1.0, 5.0] # base radius, top radius, height
        self.joint_leg_sphere = 1.0 # radius
        self.joint_arm_sphere = 1.0 # radius
        self.hand_sphere = 1.1 # radius
        self.foot_cube = 1.5 # cube's side length
        
        # head/limb motion parameters (Task 4: Walk-in-place)
        self.head_angle = 0.0 # head rotation angle used in Task 2 (TODO)
        self.arm_angle = 0.0 # arm rotation angle, with a range [-30,30], used for walk-in-place and freeform walking animations (TODO)
        self.arm_direction = 1 # 1 for swinging backward (CCW), -1 for forward (CW)
        self.leg_angle = 0.0 # leg rotation angle, with a range [-30,30], used for walk-in-place and freeform walking animations (TODO)
        self.leg_direction = 1 # 1 for swinging backward (CCW), -1 for forward (CW)
        self.swing_speed = 1.0 # arm and leg swinging speed, delta angle to increase/decrease arm_angle and leg_angle each iteration (TODO)
        
        # walk motion parameters (Task 5: Straightline and Task 6: Freeform)
        self.walk_direction = np.array([0.0, 0.0, 1.0]) # unit vector; initially aligned with z-axis (TODO)
        self.walk_angle = 0.0 # the angle (degrees) between the walk_direction and the z-axis [0, 0, 1] (TODO)
        self.walk_speed = 0.1 # straigntline and freeform walking speed (TODO)
        self.walk_vector = np.array([0.0, 0.0, 0.0]) # = walk_speed * walk_direction; updated for every iteration to translate the scarecrow during walking (TODO)

    # TODO: Task 6 Use: Update Scarecrow's walk_direction and walk_vector based on walk_angle changed by key input
    def update_walk_vector(self):



        pass # YOU MAY DELTE THIS

    # TODO: Task 1 and Task 2
    # 1. Create a Basic Scarecrow
    # 2. Rotate its head and nose based on transformation parameters updated by key input
    # NOTE: Body parts needed for the basic scarcrow have been created already
    #       you will need to transform them to approporate positions
    def draw_Scarecrow(self): 
        glClearColor(0, 0, 0, 1)                                                # set background RGBA color 
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)                        # clear the buffers initialized in the display mode

        # configure quatratic drawing
        quadratic = gluNewQuadric()
        gluQuadricDrawStyle(quadratic, GLU_FILL)  

        glPushMatrix() # DO NOT DELETE THIS

        #--------------Write your code below -------------------
 
        # TODO: Head (sphere: radius=2.5)

        glColor3f(0.0, 1.0, 0.0)
        gluSphere(quadratic, self.head_sphere, 32, 32)

        # TODO: Nose (cylinder: base-radius=0.3, top-radius=0, length=1.8)

        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.nose_cylinder[0], self.nose_cylinder[1], self.nose_cylinder[2], 32, 32)

        # TODO: Torso (cylinder: radius=2.5, length=10)

        glColor3f(1.0, 1.0, 0.0)
        gluCylinder(quadratic, self.torso_cylinder[0], self.torso_cylinder[1], self.torso_cylinder[2], 32, 32)
    
        # TODO: Right Leg (cylinders: radius=1.0, length=12)

        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.leg_cylinder[0], self.leg_cylinder[1], self.leg_cylinder[2], 32, 32)

        # TODO: Left Leg (cylinders: radius=1.0, length=12)

        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.leg_cylinder[0], self.leg_cylinder[1], self.leg_cylinder[2], 32, 32)


        # TODO: right Arm (cylinders: radius=1.0, length=10)

        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.arm_cylinder[0], self.arm_cylinder[1], self.arm_cylinder[2], 32, 32)

        # TODO: left Arm (cylinders: radius=1.0, length=10)

        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.arm_cylinder[0], self.arm_cylinder[1], self.arm_cylinder[2], 32, 32)

        #--------------Write your code above -------------------
        glPopMatrix() # DO NOT DELETE THIS


    # TODO: Task 3: Upgrade the Scarecrow with more joints 
    #       Task 4: Walk-in-place animation
    #       Task 5: Walk-in-straightline animation
    #       Task 6: Freeform walk animation with keyboard input
    # NOTE: Create a new Scarecrow with more joints, hands, and feet according to scene graph
    #       Use the given body part dimensions defined within __init__()
    #       Transform them in a cumulative manner 
    def draw_Scarecrow_Upgrade(self): 
        glClearColor(0, 0, 0, 1)                                                # set background RGBA color 
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)                        # clear the buffers initialized in the display mode

        # configure quatratic drawing
        quadratic = gluNewQuadric()
        gluQuadricDrawStyle(quadratic, GLU_FILL)  

        glPushMatrix() # DO NOT DELETE THIS

        #--------------Write your code below -------------------


        #--------------Write your code above -------------------
        glPopMatrix() # DO NOT DELETE THIS
