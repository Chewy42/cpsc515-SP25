import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

def rotate_vector(vector, angle_degrees, rot_axis="Y"):
    angle_radians = np.deg2rad(angle_degrees)
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
        rot_matrix = np.eye(3)
    return np.dot(rot_matrix, vector)

class Camera:
    def __init__(self, view_mode="front"):
        self.view_mode = view_mode
        self.eye_pos = np.array([0.0, 10.0, 50.0])
        self.look_at = np.array([0.0, 0.0, -1.0])
        self.view_up = np.array([0.0, 1.0, 0.0])
        self.tilt_angle_horizontal = 0.0
        self.tilt_angle_vertical = 0.0
        self.zoom_distance = 0.0

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
    
    def rotateViewLeft(self, angle):
        
        

    def rotateViewRight(self, angle):
        
        

    def update_view(self):
        base_look_at = np.array([0.0, 0.0, 0.0])
        if self.view_mode == "front":
            base_look_at = self.eye_pos + np.array([0.0, -10.0, -50.0])
        elif self.view_mode == "side":
            base_look_at = self.eye_pos + np.array([-50.0, -10.0, 0.0])
        elif self.view_mode == "back":
            base_look_at = self.eye_pos + np.array([0.0, -10.0, 50.0])
        gaze_vector = base_look_at - self.eye_pos
        norm = np.linalg.norm(gaze_vector)
        unit_gaze = gaze_vector / norm if norm != 0 else np.array([0, 0, -1])
        gaze_h = rotate_vector(unit_gaze, self.tilt_angle_horizontal, "Y")
        gaze_hv = rotate_vector(gaze_h, self.tilt_angle_vertical, "X")
        new_eye_pos = self.eye_pos + (gaze_hv * self.zoom_distance)
        new_lookat = new_eye_pos + gaze_hv
        self.view_up = np.array([0.0, 1.0, 0.0])
        return new_eye_pos, new_lookat 

class Wizard:
    def __init__(self):
        # Head attributes
        self.head_radius = 2.5
        self.head_offset = [0.0, 12.5, 0.0]
        self.head_rotation = [0.0, 0.0, 0.0]
        # Torso attributes
        self.torso_dims = [2.5, 2.5, 10.0]  # [base radius, top radius, height]
        self.torso_offset = [0.0, 0.0, 0.0]
        self.torso_rotation = [-90.0, 0.0, 0.0]
        # Arm attributes (split into upper and lower segments)
        self.upper_arm_dims = [1.0, 1.0, 5.0]  # cylinder dimensions for upper arm
        self.lower_arm_dims = [1.0, 1.0, 5.0]  # cylinder dimensions for lower arm
        self.joint_arm_sphere = 1.0           # elbow joint sphere radius
        self.hand_sphere = 1.1                # hand sphere radius
        self.arm_offset = [2.5, 9.0, 0.0]       # offset for arms (right arm uses positive x, left negative)
        self.arm_angle = 0.0
        self.arm_direction = 1
        self.swing_speed = 1.0
        # Leg attributes (split into upper and lower segments)
        self.upper_leg_dims = [1.0, 1.0, 6.0]  # cylinder dimensions for upper leg
        self.lower_leg_dims = [1.0, 1.0, 6.0]  # cylinder dimensions for lower leg
        self.joint_leg_sphere = 1.0           # knee joint sphere radius
        self.foot_dims = 1.5                  # scaling factor for foot cube
        self.leg_offset = [1.2, 0.0, 0.0]
        self.leg_angle = 0.0
        self.leg_direction = 1
        # Staff attributes (held in right hand, attached to lower right arm)
        self.staff_length = 10.0
        self.staff_radius = 0.3
        self.orb_radius = 1.0
        # Walking attributes
        self.walk_vector = np.array([0.0, 0.0, 0.0])
        self.walk_direction = np.array([0.0, 0.0, 1.0])
        self.walk_speed = 0.1
        self.walk_angle = 0.0

    # WASD walking logic is now handled in main.py; update_walk_vector is no longer needed.

    def draw_Wizard(self):
        quadratic = gluNewQuadric()
        glPushMatrix()
        # Apply walking transformation and rotation
        glTranslatef(self.walk_vector[0], self.walk_vector[1], self.walk_vector[2])
        glRotatef(self.walk_angle, 0, 1, 0)
        # Draw Head (white)
        glPushMatrix()
        glTranslatef(*self.head_offset)
        glRotatef(self.head_rotation[0], 0, 1, 0)
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.head_radius, 32, 32)
        # Draw Hat (purple)
        glPushMatrix()
        glTranslatef(0.0, self.head_radius, 0.0)
        glRotatef(-90, 1, 0, 0)
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.head_radius, 0.0, 3.0, 32, 32)
        glPopMatrix()
        glPopMatrix()
        # Draw Torso (purple)
        glPushMatrix()
        glTranslatef(*self.torso_offset)
        glRotatef(self.torso_rotation[0], 1, 0, 0)
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.torso_dims[0], self.torso_dims[1], self.torso_dims[2], 32, 32)
        glPopMatrix()
        # Draw Right Arm with Staff (purple)
        glPushMatrix()
        glTranslatef(self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(self.arm_angle, 1, 0, 0)
        glRotatef(15, 0, 0, 1)
        glRotatef(90, 1, 0, 0)
        # Upper right arm
        glPushMatrix()
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.upper_arm_dims[0], self.upper_arm_dims[1], self.upper_arm_dims[2], 32, 32)
        glPopMatrix()
        glTranslatef(0.0, 0.0, self.upper_arm_dims[2])
        # Elbow joint (white)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.joint_arm_sphere, 32, 32)
        glPopMatrix()
        # Lower right arm with potential bending
        glPushMatrix()
        if self.arm_angle < 0:
            bend_angle = -60 * (abs(self.arm_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.lower_arm_dims[0], self.lower_arm_dims[1], self.lower_arm_dims[2], 32, 32)
        glTranslatef(0.0, 0.0, self.lower_arm_dims[2])
        # Hand joint (white)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.hand_sphere, 32, 32)
        glPopMatrix()
        # Draw Staff attached to right hand
        glPushMatrix()
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.staff_radius, self.staff_radius, self.staff_length, 32, 32)
        glTranslatef(0.0, 0.0, self.staff_length)
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.orb_radius, 32, 32)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        # Draw Left Arm (purple)
        glPushMatrix()
        glTranslatef(-self.arm_offset[0], self.arm_offset[1], self.arm_offset[2])
        glRotatef(-self.arm_angle, 1, 0, 0)
        glRotatef(-15, 0, 0, 1)
        glRotatef(90, 1, 0, 0)
        # Upper left arm
        glPushMatrix()
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.upper_arm_dims[0], self.upper_arm_dims[1], self.upper_arm_dims[2], 32, 32)
        glPopMatrix()
        glTranslatef(0.0, 0.0, self.upper_arm_dims[2])
        # Elbow joint (white)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.joint_arm_sphere, 32, 32)
        glPopMatrix()
        # Lower left arm
        glPushMatrix()
        if self.arm_angle > 0:
            bend_angle = -60 * (abs(self.arm_angle) / 30)
            glRotatef(bend_angle, 1, 0, 0)
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.lower_arm_dims[0], self.lower_arm_dims[1], self.lower_arm_dims[2], 32, 32)
        glTranslatef(0.0, 0.0, self.lower_arm_dims[2])
        # Hand joint (white)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.hand_sphere, 32, 32)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        # Draw Right Leg (purple)
        glPushMatrix()
        glTranslatef(self.leg_offset[0], 0.0, 0.0)
        glRotatef(self.leg_angle, 1, 0, 0)
        glRotatef(90, 1, 0, 0)
        # Upper right leg
        glPushMatrix()
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.upper_leg_dims[0], self.upper_leg_dims[1], self.upper_leg_dims[2], 32, 32)
        glPopMatrix()
        glTranslatef(0.0, 0.0, self.upper_leg_dims[2])
        # Knee joint (white)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.joint_leg_sphere, 32, 32)
        glPopMatrix()
        # Lower right leg
        glPushMatrix()
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.lower_leg_dims[0], self.lower_leg_dims[1], self.lower_leg_dims[2], 32, 32)
        glTranslatef(0.0, 0.0, self.lower_leg_dims[2])
        # Foot (white as cube)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        glScalef(self.foot_dims, 1.8 * self.foot_dims, 0.8 * self.foot_dims)
        glTranslatef(0.0, 0.3, 0.0)
        glutSolidCube(1.0)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        # Draw Left Leg (purple)
        glPushMatrix()
        glTranslatef(-self.leg_offset[0], 0.0, 0.0)
        glRotatef(-self.leg_angle, 1, 0, 0)
        glRotatef(90, 1, 0, 0)
        # Upper left leg
        glPushMatrix()
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.upper_leg_dims[0], self.upper_leg_dims[1], self.upper_leg_dims[2], 32, 32)
        glPopMatrix()
        glTranslatef(0.0, 0.0, self.upper_leg_dims[2])
        # Knee joint (white)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        gluSphere(quadratic, self.joint_leg_sphere, 32, 32)
        glPopMatrix()
        # Lower left leg
        glPushMatrix()
        glColor3f(0.5, 0.0, 0.5)
        gluCylinder(quadratic, self.lower_leg_dims[0], self.lower_leg_dims[1], self.lower_leg_dims[2], 32, 32)
        glTranslatef(0.0, 0.0, self.lower_leg_dims[2])
        # Foot (white as cube)
        glPushMatrix()
        glColor3f(1.0, 1.0, 1.0)
        glScalef(self.foot_dims, 1.8 * self.foot_dims, 0.8 * self.foot_dims)
        glTranslatef(0.0, 0.3, 0.0)
        glutSolidCube(1.0)
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
