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
# def rotate_vector(vector, angle_degrees, rot_axis = "Y"): # Re-add if needed for Task 6/8
#     rotated_vector = np.array([0.0, 0.0, 0.0])
#     # TODO: convert angle degrees to radians
#     # TODO: construct a 3x3 totation matrix using np.array based on angle and rotation axis
#     # TODO: rotate the input vector by multiplying with the matrix using np.dot()
#     return rotated_vector

class Camera:
    def __init__(self, view_mode = "front"):
        self.view_mode = view_mode
        # camera parameters
        self.eye_pos = np.array([0.0, 10.0, 50.0]) # initial setting for the front view
        self.look_at = np.array([0.0, 0.0, -1.0]) # Initial look_at, will be modified
        self.view_up = np.array([0.0, 1.0, 0.0]) # Initial world up

        # Viewing parameters adjustable by keyboard input (Task 8, keys disabled in main for now)
        self.tilt_angle_horizontal = 0.0 # the angle (degrees) to rotate the gaze vector to the left or right
        self.tilt_angle_vertical = 0.0 # the angle (degrees) to rotate the gaze vector upward or downward
        self.zoom_distance = 0.0 # camera forward/backward distance along the gaze vector, positive or negative

    # TODO: Task 7: Switch between 3 standard view modes: front, side, and back
    #               For each view mode, pre-define camera parameters here 
    def switch_view(self):
        self.tilt_angle_horizontal = 0.0
        self.tilt_angle_vertical = 0.0
        self.zoom_distance = 0.0
        
        if self.view_mode == "front":
            self.eye_pos = np.array([50.0, 10.0, 0.0])
            self.look_at = np.array([-1.0, 0.0, 0.0]) 
            self.view_up = np.array([0.0, 1.0, 0.0]) 
            self.view_mode = "side"
        elif self.view_mode == "side":
            self.eye_pos = np.array([0.0, 10.0, -50.0])
            self.look_at = np.array([0.0, 0.0, 1.0]) 
            self.view_up = np.array([0.0, 1.0, 0.0])
            self.view_mode = "back"
        elif self.view_mode == "back":
            self.eye_pos = np.array([0.0, 10.0, 50.0])
            self.look_at = np.array([0.0, 0.0, -1.0]) 
            self.view_up = np.array([0.0, 1.0, 0.0])
            self.view_mode = "front"
        
    # TODO: Task 8: Update camera parameters (eye_pos and look_at) based on the new 
    #               tilt_angle_horizontal, tilt_angle_vertical, and zoom_distance updated by key input (A, D, W, S, Q, E)
    #               (Key input is currently disabled in main for this)
    def update_view(self):
        base_look_at = np.array([0.0, 0.0, 0.0]) 
        if self.view_mode == "front":
             base_look_at = self.eye_pos + np.array([0.0, -10.0, -50.0]) 
        elif self.view_mode == "side":
             base_look_at = self.eye_pos + np.array([-50.0, -10.0, 0.0])
        elif self.view_mode == "back":
             base_look_at = self.eye_pos + np.array([0.0, -10.0, 50.0])

        gaze_vector = base_look_at - self.eye_pos
        
        # TODO: calculate the current gaze vector (Done above)
        
        ## calculate new look-at point
        # TODO: tilt horizontally: rotate the gaze vector around SOME axis by tilt_angle_horizontal
        #       (Requires rotate_vector function and key handling)
        # TODO: tilt vertically: rotate the gaze vector around SOME axis by title-angle_vertical
        #       (Requires rotate_vector function and key handling)
        
        # TODO: calculate the current look-at point (Simplified below for zoom only)
        
        ## calculate new eye position by moving the camera along the gaze vector by zoom_distance
        # TODO: calculate the unit vector of the current gaze vector
        unit_gaze = gaze_vector / np.linalg.norm(gaze_vector) if np.linalg.norm(gaze_vector) != 0 else np.array([0,0,-1])

        # TODO: calculate the new eye_position
        new_eye_pos = self.eye_pos + (unit_gaze * self.zoom_distance)
        new_lookat = new_eye_pos + unit_gaze 

        self.view_up = np.array([0.0, 1.0, 0.0]) 

        return new_eye_pos, new_lookat


class Scarecrow:
    def __init__(self, version = "basic"):
        self.version = version 
        # Scarecrow body part dimensions, for both "basic" and "upgraded"
        self.head_sphere = 2.5 # radius
        self.nose_cylinder = [0.3, 0.0, 1.8] # base radius, top radius, height
        self.torso_cylinder = [2.5, 2.5, 10.0] # base radius, top radius, height
        self.head_offset = [0.0, 12.5, 0.0]
        self.head_rotation = [0.0, 0.0, 0.0] # Using this instead of head_angle for Task 2
        self.nose_offset = [0.0, 12.5, 2.5] # Relative offset might be simpler, but absolute used here
        self.torso_offset = [0.0, 0.0, 0.0]
        self.torso_rotation = [-90.0, 0.0, 0.0]
        # basic scarecrow
        self.leg_cylinder = [1.0, 1.0, 12.0] # base radius, top radius, height
        self.arm_cylinder = [1.0, 1.0, 10.0] # base radius, top radius, height
        self.leg_offset = [1.2, 0.0, 0.0]
        self.arm_offset = [2.5, 9.0, 0.0]
        self.leg_rotation = [90.0, 0.0, 0.0]
        self.arm_rotation = [90.0, 0.0, 0.0]
        
        # upgraded scarecrow
        self.upper_lower_leg_cylinder = [1.0, 1.0, 6.0] # base radius, top radius, height
        self.upper_lower_arm_cylinder = [1.0, 1.0, 5.0] # base radius, top radius, height
        self.joint_leg_sphere = 1.0 # radius
        self.joint_arm_sphere = 1.0 # radius
        self.hand_sphere = 1.1 # radius
        self.foot_cube = 1.5 # cube's side length
        
        # head/limb motion parameters (Task 4: Walk-in-place)
        self.head_angle = 0.0 # head rotation angle used in Task 2 (TODO) - Note: Using self.head_rotation now
        self.arm_angle = 0.0 # arm rotation angle, with a range [-30,30], used for walk-in-place and freeform walking animations (TODO)
        self.arm_direction = 1 # 1 for swinging backward (CCW), -1 for forward (CW)
        self.leg_angle = 0.0 # leg rotation angle, with a range [-30,30], used for walk-in-place and freeform walking animations (TODO)
        self.leg_direction = 1 # 1 for swinging backward (CCW), -1 for forward (CW)
        self.swing_speed = 1.0 # arm and leg swinging speed, delta angle to increase/decrease arm_angle and leg_angle each iteration (TODO)
        
        # walk motion parameters (Task 5: Straightline and Task 6: Freeform)
        self.walk_vector = np.array([0.0, 0.0, 0.0]) # the vector to translate the scarecrow along the walking direction (TODO)
        self.walk_direction = np.array([0.0, 0.0, 1.0]) # the direction vector that the scarecrow walks along (TODO)
        self.walk_speed = 0.5 # scarecrow walking speed, step size to increase walk_vector each iteration (TODO)
        self.walk_angle = 0.0 # scarecrow rotation angle along the y-axis, used for turning left/right (TODO)

    # TODO: Task 6 Use: Update Scarecrow's walk_direction and walk_vector based on walk_angle changed by key input
    # def update_walk_vector(self):
    #     pass # YOU MAY DELTE THIS

    # TODO: Task 1 and Task 2
    # 1. Create a Basic Scarecrow
    # 2. Rotate its head and nose based on transformation parameters updated by key input
    # NOTE: Body parts needed for the basic scarcrow have been created already
    #       you will need to transform them to approporate positions
    def draw_Scarecrow(self): 
        quadratic = gluNewQuadric()
        gluQuadricDrawStyle(quadratic, GLU_FILL)  

        glPushMatrix() # DO NOT DELETE THIS

        #--------------Write your code below -------------------
 
        # TODO: Head (sphere: radius=2.5)
        glPushMatrix()
        glTranslatef(self.head_offset[0], self.head_offset[1], self.head_offset[2])
        glRotatef(self.head_rotation[0], 0, 1, 0) # Use head_rotation for Task 2
        glColor3f(0.0, 1.0, 0.0)
        gluSphere(quadratic, self.head_sphere, 32, 32)

        # TODO: Nose (cylinder: base-radius=0.3, top-radius=0, length=1.8)
        glPushMatrix()
        glTranslatef(0.0, 0.0, self.head_sphere) 
        glColor3f(1.0, 0.5, 0.0) 
        gluCylinder(quadratic, self.nose_cylinder[0], self.nose_cylinder[1], self.nose_cylinder[2], 32, 32)
        glPopMatrix()
        glPopMatrix()

        # TODO: Torso (cylinder: radius=2.5, length=10)
        glPushMatrix()
        glTranslatef(self.torso_offset[0], self.torso_offset[1], self.torso_offset[2])
        glRotatef(self.torso_rotation[0], 1, 0, 0) 
        glColor3f(1.0, 1.0, 0.0)
        gluCylinder(quadratic, self.torso_cylinder[0], self.torso_cylinder[1], self.torso_cylinder[2], 32, 32)
        glPopMatrix()
    
        # TODO: Right Leg (cylinders: radius=1.0, length=12)
        glPushMatrix()
        glTranslatef(self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        glRotatef(self.leg_rotation[0], 1, 0, 0)
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.leg_cylinder[0], self.leg_cylinder[1], self.leg_cylinder[2], 32, 32)
        glPopMatrix()

        # TODO: Left Leg (cylinders: radius=1.0, length=12)
        glPushMatrix()
        glTranslatef(-self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        glRotatef(self.leg_rotation[0], 1, 0, 0)
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.leg_cylinder[0], self.leg_cylinder[1], self.leg_cylinder[2], 32, 32)
        glPopMatrix()

        # TODO: right Arm (cylinders: radius=1.0, length=10)
        glPushMatrix()
        glTranslatef(self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(self.arm_rotation[0], 0, 1, 0)
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.arm_cylinder[0], self.arm_cylinder[1], self.arm_cylinder[2], 32, 32)
        glPopMatrix()

        # TODO: left Arm (cylinders: radius=1.0, length=10)
        glPushMatrix()
        glTranslatef(-self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(-self.arm_rotation[0], 0, 1, 0)
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.arm_cylinder[0], self.arm_cylinder[1], self.arm_cylinder[2], 32, 32)
        glPopMatrix()
        
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
        quadratic = gluNewQuadric()
        gluQuadricDrawStyle(quadratic, GLU_FILL)  
    
        glPushMatrix()  # DO NOT DELETE THIS
        # TODO: Task 5 & 6: Translate and rotate the entire Scarecrow based on walk_vector and walk_angle
        glTranslatef(self.walk_vector[0], self.walk_vector[1], self.walk_vector[2])
        # glRotatef(self.walk_angle, 0, 1, 0) # Rotation for Task 6
    
        #--------------Write your code below -------------------
    
        # Head
        glPushMatrix()
        glTranslatef(*self.head_offset)
        glRotatef(self.head_rotation[0], 0, 1, 0) # Use head_rotation for Task 2
        glColor3f(0.0, 1.0, 0.0)  
        gluSphere(quadratic, self.head_sphere, 32, 32)
    
        # Nose
        glPushMatrix()
        glTranslatef(0.0, 0.0, self.head_sphere) 
        glColor3f(1.0, 0.5, 0.0)  
        gluCylinder(quadratic, self.nose_cylinder[0], self.nose_cylinder[1], self.nose_cylinder[2], 32, 32)
        glPopMatrix()  
        glPopMatrix()  
    
        # Torso
        glPushMatrix()
        glTranslatef(*self.torso_offset)
        glRotatef(self.torso_rotation[0], 1, 0, 0) 
        glColor3f(1.0, 1.0, 0.0)  
        gluCylinder(quadratic, self.torso_cylinder[0], self.torso_cylinder[1], self.torso_cylinder[2], 32, 32)
        glPopMatrix()  
    
        # Right Arm
        glPushMatrix()
        glTranslatef(self.arm_offset[0], self.arm_offset[1], self.arm_offset[2]) 
        # TODO: Task 4: Apply arm swing rotation (self.arm_angle)
        glRotatef(self.arm_angle, 1, 0, 0) 
        glRotatef(15, 0, 0, 1)  
        glRotatef(90, 1, 0, 0) 
    
        # Upper Arm
        glPushMatrix()
        glColor3f(0.0, 0.0, 1.0)  
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        glPopMatrix()
    
        # Joint Sphere (Elbow)
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2]) 
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5) 
        gluSphere(quadratic, self.joint_arm_sphere, 32, 32)
        glPopMatrix()
    
        # Lower Arm and Hand (right arm)
        glPushMatrix() 
        # TODO: Task 4: Apply elbow bend rotation based on arm_angle
        if self.arm_angle < 0: 
            bend_angle = -60 * (abs(self.arm_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        
        glColor3f(0.0, 0.0, 1.0)  
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2]) 
        
        glPushMatrix()
        glColor3f(0.0, 1.0, 0.0)  
        gluSphere(quadratic, self.hand_sphere, 32, 32) 
        glPopMatrix() 
        
        glPopMatrix() 
    
        glPopMatrix()  
    
        # Left Arm
        glPushMatrix()
        glTranslatef(-self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        # TODO: Task 4: Apply arm swing rotation (mirrored)
        glRotatef(-self.arm_angle, 1, 0, 0) 
        glRotatef(-15, 0, 0, 1)  
        glRotatef(90, 1, 0, 0) 
    
        # Upper Arm
        glPushMatrix()
        glColor3f(0.0, 0.0, 1.0)  
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        glPopMatrix()
    
        # Joint Sphere (Elbow)
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2]) 
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5) 
        gluSphere(quadratic, self.joint_arm_sphere, 32, 32)
        glPopMatrix()
    
        # Lower Arm and Hand (left arm)
        glPushMatrix() 
        # TODO: Task 4: Apply elbow bend rotation based on arm_angle (mirrored)
        if self.arm_angle > 0: 
            bend_angle = -60 * (abs(self.arm_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        
        glColor3f(0.0, 0.0, 1.0)  
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2]) 
        
        glPushMatrix()
        glColor3f(0.0, 1.0, 0.0)  
        gluSphere(quadratic, self.hand_sphere, 32, 32)
        glPopMatrix() 
        
        glPopMatrix() 
    
        glPopMatrix()  
    
        # Right Leg
        glPushMatrix()
        glTranslatef(self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        # TODO: Task 4: Apply leg swing rotation (self.leg_angle)
        glRotatef(self.leg_angle, 1, 0, 0) 
        glRotatef(90, 1, 0, 0) 
    
        # Upper Leg
        glPushMatrix()
        glColor3f(1.0, 0.0, 0.0)  
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        glPopMatrix()
    
        # Joint Sphere (Knee)
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2]) 
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5) 
        gluSphere(quadratic, self.joint_leg_sphere, 32, 32)
        glPopMatrix()
    
        # Lower Leg and Foot
        glPushMatrix() 
        # TODO: Task 4: Apply knee bend rotation based on leg_angle
        if self.leg_angle > 0: 
            bend_angle = -30 * (abs(self.leg_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        
        glColor3f(1.0, 0.0, 0.0)  
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2]) 

        # Foot (Scaled Cube) - Applying Scale FIRST, then Translate
        glPushMatrix() # Start Foot block
        # Apply scaling FIRST, centered at the ankle
        glScalef(1.0 * self.foot_cube, 0.8 * self.foot_cube, 1.8 * self.foot_cube) 
        # Translate AFTER scaling. Move forward along the scaled Z-axis by 0.5 
        # (half the unit cube's depth) so the ankle is at the back face (Z=0) of the scaled cube.
        glTranslatef(0.0, 0.0, 0.5) 
        glColor3f(0.8, 0.4, 0.0)  
        
        # TODO: Task 3: Draw the foot using glutSolidCube or GL_QUADS
        glPushMatrix() # Isolate glutSolidCube
        glutSolidCube(1.0) # Draw a unit cube; scaling/translation already applied
        glPopMatrix() # End isolation
        
        glPopMatrix() # End Foot Draw

        glPopMatrix() # End Lower Leg + Foot block

        glPopMatrix()  # Close Right Leg
        
        # Left Leg
        glPushMatrix()
        glTranslatef(-self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        # TODO: Task 4: Apply leg swing rotation (mirrored)
        glRotatef(-self.leg_angle, 1, 0, 0) 
        glRotatef(90, 1, 0, 0) 

        # Upper Leg
        glPushMatrix()
        glColor3f(1.0, 0.0, 0.0)  
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        glPopMatrix()
    
        # Joint Sphere (Knee)
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2]) 
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5) 
        gluSphere(quadratic, self.joint_leg_sphere, 32, 32)
        glPopMatrix()
    
        # Lower Leg and Foot
        glPushMatrix() 
        # TODO: Task 4: Apply knee bend rotation based on leg_angle (mirrored)
        if self.leg_angle < 0: 
            bend_angle = -30 * (abs(self.leg_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        
        glColor3f(1.0, 0.0, 0.0)  
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2]) 

        # Foot (Scaled Cube) - Applying Scale FIRST, then Translate
        glPushMatrix() # Start Foot block
        # Apply scaling FIRST, centered at the ankle
        glScalef(1.0 * self.foot_cube, 0.8 * self.foot_cube, 1.8 * self.foot_cube) 
        # Translate AFTER scaling. Move forward along the scaled Z-axis by 0.5 
        # (half the unit cube's depth) so the ankle is at the back face (Z=0) of the scaled cube.
        glTranslatef(0.0, 0.0, 0.5) 
        glColor3f(0.8, 0.4, 0.0)  

        # TODO: Task 3: Draw the foot using glutSolidCube or GL_QUADS
        glPushMatrix() # Isolate glutSolidCube
        glutSolidCube(1.0) # Draw a unit cube; scaling/translation already applied
        glPopMatrix() # End isolation

        glPopMatrix() # End Foot Draw

        glPopMatrix() # End Lower Leg + Foot block
        
        glPopMatrix()  # Close Left Leg
    
        #--------------Write your code above -------------------
        glPopMatrix()  # DO NOT DELETE THIS
