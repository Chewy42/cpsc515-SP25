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
        self.tilt_angle_horizontal -= angle
        if self.tilt_angle_horizontal > 360:
            self.tilt_angle_horizontal -= 360
        elif self.tilt_angle_horizontal < -360:
            self.tilt_angle_horizontal += 360
        self.eye_pos = rotate_vector(self.eye_pos, -angle, "Y")
        

    def rotateViewRight(self, angle):
        self.tilt_angle_horizontal += angle
        if self.tilt_angle_horizontal > 360:
            self.tilt_angle_horizontal -= 360
        elif self.tilt_angle_horizontal < -360:
            self.tilt_angle_horizontal += 360
        self.eye_pos = rotate_vector(self.eye_pos, angle, "Y")

        
        

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

class Panther:
    def __init__(self):
        # Basic color
        self.color = [0.1, 0.1, 0.1] # Dark grey/black

        # Head attributes
        self.head_radius = 2.0
        self.head_offset = [0.0, 1.5, 6.0] # Positioned at the front of the torso

        # Torso attributes (using a scaled cube for simplicity)
        self.torso_dims = [3.0, 2.0, 8.0] # width, height, length
        self.torso_offset = [0.0, 0.0, 0.0] # Centered at origin before translation

        # Leg attributes (simplified cylinders)
        self.leg_radius = 0.5
        self.leg_height = 4.0
        self.front_leg_offset = [1.5, -1.0, 4.0] # x, y, z relative to torso center
        self.hind_leg_offset = [1.5, -1.0, -4.0] # x, y, z relative to torso center

        # Tail attributes
        self.tail_radius = 0.3
        self.tail_length = 7.0
        self.tail_segments = 5 # Number of cylinders for the tail
        self.tail_angle = -30.0 # Initial downward angle

        # Position and orientation (can be controlled externally)
        self.position = np.array([0.0, 0.0, 0.0])
        self.angle = 0.0 # Y-axis rotation
        
        # For AABB collision detection
        # Use a box that encompasses the entire panther
        self.aabb_half_size = np.array([3.0, 3.0, 7.0])  # Wider in z-direction to cover head and body

    def get_aabb(self):
        """Return min and max coordinates of the AABB for this panther"""
        min_coords = self.position - self.aabb_half_size
        max_coords = self.position + self.aabb_half_size
        return min_coords, max_coords
        
    def draw_Panther(self):
        quadratic = gluNewQuadric()
        glPushMatrix()

        # Apply overall position and rotation
        glTranslatef(self.position[0], self.position[1], self.position[2])
        glRotatef(self.angle, 0, 1, 0)

        glColor3fv(self.color)

        # Draw Torso (scaled cube)
        glPushMatrix()
        glTranslatef(*self.torso_offset)
        glScalef(*self.torso_dims)
        glutSolidCube(1.0)
        glPopMatrix()

        # Draw Head (sphere)
        glPushMatrix()
        glTranslatef(*self.head_offset)
        gluSphere(quadratic, self.head_radius, 32, 32)
        # Simple ears (cones)
        glPushMatrix()
        glTranslatef(-0.8, self.head_radius * 0.8, 0.0)
        glRotatef(-90, 1, 0, 0)
        glRotatef(-20, 0, 1, 0)
        gluCylinder(quadratic, 0.5, 0.0, 1.0, 16, 16) # Left ear
        glPopMatrix()
        glPushMatrix()
        glTranslatef(0.8, self.head_radius * 0.8, 0.0)
        glRotatef(-90, 1, 0, 0)
        glRotatef(20, 0, 1, 0)
        gluCylinder(quadratic, 0.5, 0.0, 1.0, 16, 16) # Right ear
        glPopMatrix()
        glPopMatrix()

        # Draw Legs (cylinders) - Front Right
        glPushMatrix()
        glTranslatef(self.front_leg_offset[0], self.front_leg_offset[1], self.front_leg_offset[2])
        glRotatef(90, 1, 0, 0) # Orient vertically
        gluCylinder(quadratic, self.leg_radius, self.leg_radius, self.leg_height, 16, 16)
        # Simple foot (sphere)
        glTranslatef(0, 0, self.leg_height)
        gluSphere(quadratic, self.leg_radius * 1.2, 16, 16)
        glPopMatrix()

        # Front Left
        glPushMatrix()
        glTranslatef(-self.front_leg_offset[0], self.front_leg_offset[1], self.front_leg_offset[2])
        glRotatef(90, 1, 0, 0)
        gluCylinder(quadratic, self.leg_radius, self.leg_radius, self.leg_height, 16, 16)
        glTranslatef(0, 0, self.leg_height)
        gluSphere(quadratic, self.leg_radius * 1.2, 16, 16)
        glPopMatrix()

        # Hind Right
        glPushMatrix()
        glTranslatef(self.hind_leg_offset[0], self.hind_leg_offset[1], self.hind_leg_offset[2])
        glRotatef(90, 1, 0, 0)
        gluCylinder(quadratic, self.leg_radius, self.leg_radius, self.leg_height, 16, 16)
        glTranslatef(0, 0, self.leg_height)
        gluSphere(quadratic, self.leg_radius * 1.2, 16, 16)
        glPopMatrix()

        # Hind Left
        glPushMatrix()
        glTranslatef(-self.hind_leg_offset[0], self.hind_leg_offset[1], self.hind_leg_offset[2])
        glRotatef(90, 1, 0, 0)
        gluCylinder(quadratic, self.leg_radius, self.leg_radius, self.leg_height, 16, 16)
        glTranslatef(0, 0, self.leg_height)
        gluSphere(quadratic, self.leg_radius * 1.2, 16, 16)
        glPopMatrix()

        # Draw Tail (segmented cylinder)
        glPushMatrix()
        # Position tail base at the back of the torso
        glTranslatef(0.0, 0.5, -self.torso_dims[2] / 2)
        glRotatef(self.tail_angle, 1, 0, 0) # Initial angle
        segment_length = self.tail_length / self.tail_segments
        for _ in range(self.tail_segments):
            gluCylinder(quadratic, self.tail_radius, self.tail_radius * 0.9, segment_length, 16, 16) # Taper slightly
            glTranslatef(0.0, 0.0, segment_length)
            glRotatef(5, 1, 0, 0) # Slight upward curve per segment
            self.tail_radius *= 0.95 # Taper radius
        # Reset tail radius for next draw call if needed (or manage state better)
        self.tail_radius = 0.3
        glPopMatrix()


        glPopMatrix() # End of Panther model matrix
