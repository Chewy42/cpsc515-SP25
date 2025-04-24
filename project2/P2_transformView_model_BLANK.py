import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

# Function to rotate a vector around a specified axis
def rotate_vector(vector, angle_degrees, rot_axis = "Y"):
    # Initialize the rotated vector
    rotated_vector = np.array([0.0, 0.0, 0.0])
    # Convert angle from degrees to radians
    angle_radians = np.deg2rad(angle_degrees)
    
    # Define rotation matrices for each axis
    if rot_axis == "X":
        rot_matrix = np.array([
            [1, 0, 0],
            [0, np.cos(angle_radians), -np.sin(angle_radians)],
            [0, np.sin(angle_radians), np.cos(angle_radians)]
        ])
    elif rot_axis == "Y":
        rot_matrix = np.array([
            [np.cos(angle_radians), 0, np.sin(angle_radians)],
            [0, 1, 0],
            [-np.sin(angle_radians), 0, np.cos(angle_radians)]
        ])
    elif rot_axis == "Z":
        rot_matrix = np.array([
            [np.cos(angle_radians), -np.sin(angle_radians), 0],
            [np.sin(angle_radians), np.cos(angle_radians), 0],
            [0, 0, 1]
        ])
    else:
        # Default to identity matrix if axis is not specified
        rot_matrix = np.eye(3)
    
    # Rotate the vector using the rotation matrix
    rotated_vector = np.dot(rot_matrix, vector)
    return rotated_vector

# Camera class to manage camera view and transformations
class Camera:
    def __init__(self, view_mode = "front"):
        # Initialize camera view mode and position
        self.view_mode = view_mode
        self.eye_pos = np.array([0.0, 10.0, 50.0])
        self.look_at = np.array([0.0, 0.0, -1.0])
        self.view_up = np.array([0.0, 1.0, 0.0])
        # Initialize camera tilt and zoom angles
        self.tilt_angle_horizontal = 0.0
        self.tilt_angle_vertical = 0.0
        self.zoom_distance = 0.0
        self.roll_angle = 0.0

    # Method to switch between different camera views
    def switch_view(self):
        # Reset camera tilt and zoom angles
        self.tilt_angle_horizontal = 0.0
        self.tilt_angle_vertical = 0.0
        self.zoom_distance = 0.0
        self.roll_angle = 0.0
        # Update camera position and view mode
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
            self.view_mode = "first_person"
        elif self.view_mode == "first_person":
            self.eye_pos = np.array([0.0, 10.0, 50.0])
            self.look_at = np.array([0.0, 0.0, -1.0])
            self.view_up = np.array([0.0, 1.0, 0.0])
            self.view_mode = "front"

    # Method to update camera view based on tilt and zoom angles
    def update_view(self, scarecrow=None):
        # If in first person mode, update camera view based on scarecrow position
        if self.view_mode == "first_person" and scarecrow is not None:
            # Calculate camera position and view direction based on scarecrow position
            base = scarecrow.walk_vector
            head_offset = np.array(scarecrow.head_offset)
            head_pos = base + head_offset
            facing = np.array([0.0, 0.0, 1.0])
            facing = rotate_vector(facing, scarecrow.walk_angle, "Y")
            facing = rotate_vector(facing, scarecrow.head_rotation[0], "X")
            nose_offset = np.array([0.0, 0.5, scarecrow.nose_cylinder[2]])
            eye_pos = head_pos + rotate_vector(nose_offset, scarecrow.head_rotation[0], "X")
            eye_pos = base + rotate_vector(eye_pos - base, scarecrow.walk_angle, "Y")
            new_eye_pos = eye_pos
            new_lookat = new_eye_pos + 2 * facing
            return new_eye_pos, new_lookat
        
        # Calculate gaze vector and unit gaze vector
        base_look_at = self.eye_pos + self.look_at
        gaze_vector = base_look_at - self.eye_pos
        norm = np.linalg.norm(gaze_vector)
        unit_gaze = gaze_vector / norm if norm != 0 else np.array([0,0,-1])
        
        # Apply tilt and zoom angles to unit gaze vector
        gaze_h = rotate_vector(unit_gaze, self.tilt_angle_horizontal, "Y")
        gaze_hv = rotate_vector(gaze_h, self.tilt_angle_vertical, "X")
        new_eye_pos = self.eye_pos + (gaze_hv * self.zoom_distance)
        new_lookat = new_eye_pos + gaze_hv
        
        # Apply roll angle around the gaze axis to tilt camera
        axis = gaze_hv / np.linalg.norm(gaze_hv)
        theta = np.deg2rad(self.roll_angle)
        ux, uy, uz = axis
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        R = np.array([
            [cos_t + ux*ux*(1-cos_t), ux*uy*(1-cos_t)-uz*sin_t, ux*uz*(1-cos_t)+uy*sin_t],
            [uy*ux*(1-cos_t)+uz*sin_t, cos_t + uy*uy*(1-cos_t), uy*uz*(1-cos_t)-ux*sin_t],
            [uz*ux*(1-cos_t)-uy*sin_t, uz*uy*(1-cos_t)+ux*sin_t, cos_t + uz*uz*(1-cos_t)]
        ])
        self.view_up = R.dot(np.array([0.0, 1.0, 0.0]))
        return new_eye_pos, new_lookat

# Scarecrow class to manage scarecrow position and transformations
class Scarecrow:
    def __init__(self, version = "basic"):
        # Initialize scarecrow version and body parts
        self.version = version
        self.head_sphere = 2.5
        self.nose_cylinder = [0.3, 0.0, 1.8]
        self.torso_cylinder = [2.5, 2.5, 10.0]
        self.head_offset = [0.0, 12.5, 0.0]
        self.head_rotation = [0.0, 0.0, 0.0]
        self.nose_offset = [0.0, 12.5, 2.5]
        self.torso_offset = [0.0, 0.0, 0.0]
        self.torso_rotation = [-90.0, 0.0, 0.0]
        self.leg_cylinder = [1.0, 1.0, 12.0]
        self.arm_cylinder = [1.0, 1.0, 10.0]
        self.leg_offset = [1.2, 0.0, 0.0]
        self.arm_offset = [2.5, 9.0, 0.0]
        self.leg_rotation = [90.0, 0.0, 0.0]
        self.arm_rotation = [90.0, 0.0, 0.0]
        self.upper_lower_leg_cylinder = [1.0, 1.0, 6.0]
        self.upper_lower_arm_cylinder = [1.0, 1.0, 5.0]
        self.joint_leg_sphere = 1.0
        self.joint_arm_sphere = 1.0
        self.hand_sphere = 1.1
        self.foot_cube = 1.5
        self.head_angle = 0.0
        self.arm_angle = 0.0
        self.arm_direction = 1
        self.leg_angle = 0.0
        self.leg_direction = 1
        self.swing_speed = 1.0
        self.walk_vector = np.array([0.0, 0.0, 0.0])
        self.walk_direction = np.array([0.0, 0.0, 1.0])
        self.walk_speed = 0.1
        self.walk_angle = 0.0

    # Method to update scarecrow walk vector
    def update_walk_vector(self):
        # Update walk direction and vector
        self.walk_direction = rotate_vector(np.array([0.0, 0.0, 1.0]), self.walk_angle, "Y")
        norm = np.linalg.norm(self.walk_direction)
        if norm != 0:
            self.walk_direction = self.walk_direction / norm
        self.walk_vector += self.walk_speed * self.walk_direction

    # Method to draw scarecrow
    def draw_Scarecrow(self, camera=None):
        # Create a new quadric object
        quadratic = gluNewQuadric()
        gluQuadricDrawStyle(quadratic, GLU_FILL)
        glPushMatrix()
        
        # Draw head
        glPushMatrix()
        glTranslatef(self.head_offset[0], self.head_offset[1], self.head_offset[2])
        glRotatef(self.head_rotation[0], 0, 1, 0)
        glColor3f(0.0, 1.0, 0.0)
        gluSphere(quadratic, self.head_sphere, 32, 32)
        
        # Draw nose
        glPushMatrix()
        glTranslatef(0.0, 0.0, self.head_sphere)
        glColor3f(1.0, 0.5, 0.0)
        gluCylinder(quadratic, self.nose_cylinder[0], self.nose_cylinder[1], self.nose_cylinder[2], 32, 32)
        glPopMatrix()
        glPopMatrix()
        
        # Draw torso
        glPushMatrix()
        glTranslatef(self.torso_offset[0], self.torso_offset[1], self.torso_offset[2])
        glRotatef(self.torso_rotation[0], 1, 0, 0)
        glColor3f(1.0, 1.0, 0.0)
        gluCylinder(quadratic, self.torso_cylinder[0], self.torso_cylinder[1], self.torso_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw right leg
        glPushMatrix()
        glTranslatef(self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        glRotatef(self.leg_rotation[0], 1, 0, 0)
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.leg_cylinder[0], self.leg_cylinder[1], self.leg_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw left leg
        glPushMatrix()
        glTranslatef(-self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        glRotatef(self.leg_rotation[0], 1, 0, 0)
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.leg_cylinder[0], self.leg_cylinder[1], self.leg_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw right arm
        glPushMatrix()
        glTranslatef(self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(self.arm_rotation[0], 0, 1, 0)
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.arm_cylinder[0], self.arm_cylinder[1], self.arm_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw left arm
        glPushMatrix()
        glTranslatef(-self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(-self.arm_rotation[0], 0, 1, 0)
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.arm_cylinder[0], self.arm_cylinder[1], self.arm_cylinder[2], 32, 32)
        glPopMatrix()
        
        glPopMatrix()

    # Method to draw scarecrow with upgrades
    def draw_Scarecrow_Upgrade(self, camera=None): 
        # Clear the screen and set the background color
        glClearColor(0, 0, 0, 1)
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        
        # Create a new quadric object
        quadratic = gluNewQuadric()
        gluQuadricDrawStyle(quadratic, GLU_FILL)  
        glPushMatrix() # DO NOT DELETE THIS
        
        # Translate to scarecrow walk vector
        glTranslatef(self.walk_vector[0], self.walk_vector[1], self.walk_vector[2])
        glRotatef(self.walk_angle, 0, 1, 0)
        
        # Draw head and nose
        if camera is None or camera.view_mode != "first_person":
            glPushMatrix()
            glTranslatef(*self.head_offset)
            glRotatef(self.head_rotation[0], 0, 1, 0)
            glColor3f(0.0, 1.0, 0.0)
            gluSphere(quadratic, self.head_sphere, 32, 32)
            
            # Draw nose
            glPushMatrix()
            glTranslatef(0.0, 0.0, self.head_sphere)
            glColor3f(1.0, 0.5, 0.0)
            gluCylinder(quadratic, self.nose_cylinder[0], self.nose_cylinder[1], self.nose_cylinder[2], 32, 32)
            glPopMatrix()
            glPopMatrix()

        # Draw torso
        glPushMatrix()
        glTranslatef(*self.torso_offset)
        glRotatef(self.torso_rotation[0], 1, 0, 0)
        glColor3f(1.0, 1.0, 0.0)
        gluCylinder(quadratic, self.torso_cylinder[0], self.torso_cylinder[1], self.torso_cylinder[2], 32, 32)
        glPopMatrix()

        # Draw right arm
        glPushMatrix()
        glTranslatef(self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(self.arm_angle, 1, 0, 0)
        glRotatef(15, 0, 0, 1)
        glRotatef(90, 1, 0, 0)
        
        # Draw upper arm
        glPushMatrix()
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw elbow joint
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2])
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5)
        gluSphere(quadratic, self.joint_arm_sphere, 32, 32)
        glPopMatrix()
        
        # Draw lower arm and hand
        glPushMatrix()
        if self.arm_angle < 0:
            bend_angle = -60 * (abs(self.arm_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        
        # Draw hand
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2])
        glPushMatrix()
        glColor3f(0.0, 1.0, 0.0)
        gluSphere(quadratic, self.hand_sphere, 32, 32)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()

        # Draw left arm
        glPushMatrix()
        glTranslatef(-self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(-self.arm_angle, 1, 0, 0)
        glRotatef(-15, 0, 0, 1)
        glRotatef(90, 1, 0, 0)
        
        # Draw upper arm
        glPushMatrix()
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw elbow joint
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2])
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5)
        gluSphere(quadratic, self.joint_arm_sphere, 32, 32)
        glPopMatrix()
        
        # Draw lower arm and hand
        glPushMatrix()
        if self.arm_angle > 0:
            bend_angle = -60 * (abs(self.arm_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        glColor3f(0.0, 0.0, 1.0)
        gluCylinder(quadratic, self.upper_lower_arm_cylinder[0], self.upper_lower_arm_cylinder[1], self.upper_lower_arm_cylinder[2], 32, 32)
        
        # Draw hand
        glTranslatef(0.0, 0.0, self.upper_lower_arm_cylinder[2])
        glPushMatrix()
        glColor3f(0.0, 1.0, 0.0)
        gluSphere(quadratic, self.hand_sphere, 32, 32)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()

        # Draw right leg
        glPushMatrix()
        glTranslatef(self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        glRotatef(self.leg_angle, 1, 0, 0)
        glRotatef(90, 1, 0, 0)
        
        # Draw upper leg
        glPushMatrix()
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw knee joint
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2])
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5)
        gluSphere(quadratic, self.joint_leg_sphere, 32, 32)
        glPopMatrix()
        
        # Draw lower leg and foot
        glPushMatrix()
        if self.leg_angle > 0:
            bend_angle = -30 * (abs(self.leg_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        
        # Draw foot
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2])
        glPushMatrix()
        glScalef(0.8 * self.foot_cube, 1.8 * self.foot_cube, 1.0 * self.foot_cube)
        glTranslatef(0.0, 0.3, 0.0)
        glColor3f(0.8, 0.4, 0.0)
        glPushMatrix()
        glutSolidCube(1.0)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()

        # Draw left leg
        glPushMatrix()
        glTranslatef(-self.leg_offset[0], self.leg_offset[1], self.leg_offset[2])
        glRotatef(-self.leg_angle, 1, 0, 0)
        glRotatef(90, 1, 0, 0)
        
        # Draw upper leg
        glPushMatrix()
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        glPopMatrix()
        
        # Draw knee joint
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2])
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5)
        gluSphere(quadratic, self.joint_leg_sphere, 32, 32)
        glPopMatrix()
        
        # Draw lower leg and foot
        glPushMatrix()
        if self.leg_angle < 0:
            bend_angle = -30 * (abs(self.leg_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        glColor3f(1.0, 0.0, 0.0)
        gluCylinder(quadratic, self.upper_lower_leg_cylinder[0], self.upper_lower_leg_cylinder[1], self.upper_lower_leg_cylinder[2], 32, 32)
        
        # Draw foot
        glTranslatef(0.0, 0.0, self.upper_lower_leg_cylinder[2])
        glPushMatrix()
        glScalef(1.0 * self.foot_cube, 1.8 * self.foot_cube, 0.8)
        glTranslatef(0.0, 0.5, 0.0)
        glColor3f(0.8, 0.4, 0.0)
        glPushMatrix()
        glutSolidCube(1.0)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        #--------------Write your code above -------------------
        glPopMatrix() # DO NOT DELETE THIS