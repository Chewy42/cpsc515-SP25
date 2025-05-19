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

def rotate_around_axis(vector, axis, angle_degrees):
    """Rotate a vector around an arbitrary axis using Rodrigues' rotation formula."""
    angle_radians = np.deg2rad(angle_degrees)
    axis = np.asarray(axis, dtype=float)
    norm_axis = np.linalg.norm(axis)
    if norm_axis < 1e-8: # Avoid division by zero for zero vector axis
        return vector
    axis = axis / norm_axis # Normalize axis
    
    # Rodrigues' rotation formula components
    # K = [[0, -az, ay], [az, 0, -ax], [-ay, ax, 0]]
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    I = np.eye(3) # Identity matrix
    cos_theta = np.cos(angle_radians)
    sin_theta = np.sin(angle_radians)
    
    # R = I + sin(theta)*K + (1 - cos(theta))*K^2
    R = I + sin_theta * K + (1 - cos_theta) * np.dot(K, K)
    return np.dot(R, vector)

class Camera:
    def __init__(self, view_mode="front"):
        self.view_mode = view_mode
        self.eye_pos = np.array([0.0, 10.0, 50.0])
        self.look_at_target = np.array([0.0, 0.0, 0.0]) # Renamed for clarity, as look_at is dynamic
        self.view_up = np.array([0.0, 1.0, 0.0])
        self.tilt_angle_horizontal = 0.0
        self.tilt_angle_vertical = 0.0
        self.manual_zoom = 0.0  # persistent horizontal zoom offset for follow/orbital views
        if self.view_mode == "front":
            # Side View
            self.eye_pos = np.array([50.0, 10.0, 0.0]) # Positioned to the right of origin
            self.look_at_target = np.array([0.0, 0.0, 0.0]) # Looks towards origin
            self.view_mode = "side"
        elif self.view_mode == "side":
            # Back View (Original 'front' view)
            self.eye_pos = np.array([0.0, 10.0, 50.0]) # Positioned behind origin
            self.look_at_target = np.array([0.0, 0.0, 0.0]) # Looks towards origin
            self.view_mode = "back"
        elif self.view_mode == "back":
            # FPV View
            self.view_mode = "fpv"
        elif self.view_mode == "fpv":
            # Third Person Follow View
            self.view_mode = "third_person_follow"
            # Initial offset for third_person_follow, will be updated relative to wizard
            self.eye_pos = np.array([0.0, 15.0, 25.0]) 
            self.look_at_target = np.array([0.0, 5.0, 0.0]) # Initially look at wizard's general area
        elif self.view_mode == "third_person_follow":
            # Original Front View (now cycles back to this)
            self.eye_pos = np.array([0.0, 10.0, 50.0])
            self.look_at_target = np.array([0.0, 0.0, 0.0])
            self.view_mode = "front"
        
        # Common for all non-FPV modes that use orbital controls
        if self.view_mode != "fpv":
            self.view_up = np.array([0.0, 1.0, 0.0])

    def switch_view(self):
        self.tilt_angle_horizontal = 0.0
        self.tilt_angle_vertical = 0.0
        self.manual_zoom = 0.0
        if self.view_mode == "front":
            # Side View
            self.eye_pos = np.array([50.0, 10.0, 0.0]) # Positioned to the right of origin
            self.look_at_target = np.array([0.0, 0.0, 0.0]) # Looks towards origin
            self.view_mode = "side"
        elif self.view_mode == "side":
            # Back View (Original 'front' view)
            self.eye_pos = np.array([0.0, 10.0, 50.0]) # Positioned behind origin
            self.look_at_target = np.array([0.0, 0.0, 0.0]) # Looks towards origin
            self.view_mode = "back"
        elif self.view_mode == "back":
            # FPV View
            self.view_mode = "fpv"
        elif self.view_mode == "fpv":
            # Third Person Follow View
            self.view_mode = "third_person_follow"
            # Initial offset for third_person_follow, will be updated relative to wizard
            self.eye_pos = np.array([0.0, 15.0, 25.0]) 
            self.look_at_target = np.array([0.0, 5.0, 0.0]) # Initially look at wizard's general area
        elif self.view_mode == "third_person_follow":
            # Original Front View (now cycles back to this)
            self.eye_pos = np.array([0.0, 10.0, 50.0])
            self.look_at_target = np.array([0.0, 0.0, 0.0])
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
        if self.view_mode == "fpv" and self.wizard is not None:
            # For FPV mode, position camera at wizard's head position
            wizard_pos = self.wizard.walk_vector.copy()
            head_height_offset = self.wizard.head_offset[1]
            
            # Base eye position at wizard's head height
            eye_base_pos = wizard_pos + np.array([0, head_height_offset, 0])

            # Look in the direction the wizard is facing (yaw) + camera pitch. Assume wizard's intrinsic front is +Z.
            forward_dir_yaw = rotate_vector(np.array([0.0, 0.0, 1.0]), self.wizard.walk_angle + self.tilt_angle_horizontal, "Y")
            
            # Calculate the right vector based on the new forward_dir_yaw for proper pitch rotation axis
            world_up = np.array([0.0, 1.0, 0.0])
            right_dir = np.cross(forward_dir_yaw, world_up)
            if np.linalg.norm(right_dir) < 1e-6: # Avoid issues if forward_dir_yaw is parallel to world_up
                right_dir = np.array([1.0, 0.0, 0.0]) # Default to X-axis if looking straight up/down
            else:
                right_dir /= np.linalg.norm(right_dir)

            # Apply pitch rotation around the new right_dir to get the final 3D viewing direction
            final_forward_dir = rotate_around_axis(forward_dir_yaw, right_dir, self.tilt_angle_vertical)

            # Offset the eye position from the head center along the final_forward_dir
            # This places the camera just outside the wizard's head model.
            eye_offset_scalar = self.wizard.head_radius + 0.05 
            new_eye_pos = eye_base_pos + final_forward_dir * eye_offset_scalar
            
            new_lookat = new_eye_pos + final_forward_dir # Look-at point is 1 unit from new_eye_pos along final_forward_dir

            # Keep the up direction correctly oriented by recalculating based on new forward and right vectors
            self.view_up = np.cross(final_forward_dir, -right_dir) # up = forward x (-right) to ensure it's mostly upwards
            if np.linalg.norm(self.view_up) < 1e-6:
                self.view_up = world_up # Fallback if looking straight along right_dir
            else:
                self.view_up /= np.linalg.norm(self.view_up)

            return new_eye_pos, new_lookat
        
        elif self.view_mode == "third_person_follow" and self.wizard is not None:
            wizard_center_pos = self.wizard.walk_vector.copy()
            wizard_center_pos[1] += self.wizard.torso_dims[1] / 2 + self.wizard.head_offset[1] / 2 # Approx mid-body height

            # Base offset from wizard (over-the-shoulder: left, above, behind)
            base_offset = np.array([-6.0, 8.0, 15.0]) # x (left), y (height), z (distance behind)

            # Rotate offset by wizard's walk_angle (yaw)
            rotated_offset = rotate_vector(base_offset, self.wizard.walk_angle, "Y")
            
            # Further rotate offset by camera's independent horizontal tilt (around wizard's Y-axis)
            rotated_offset = rotate_vector(rotated_offset, self.tilt_angle_horizontal, "Y") # horizontal orbit around wizard

            # Disable vertical tilt for third_person_follow to keep camera level

            # Apply zoom: adjust only the horizontal (XZ) distance while keeping Y offset constant
            orig_y = rotated_offset[1]
            horiz_vec = np.array([rotated_offset[0], 0.0, rotated_offset[2]])
            horiz_len = np.linalg.norm(horiz_vec)
            if horiz_len < 1e-6:
                horiz_vec = np.array([0.0, 0.0, 1.0])
                horiz_len = 1.0

            new_horiz_len = max(1.0, horiz_len + self.manual_zoom)

            horiz_vec = horiz_vec / horiz_len * new_horiz_len
            offset_zoomed = np.array([horiz_vec[0], orig_y, horiz_vec[2]])

            new_eye_pos = wizard_center_pos + offset_zoomed
            new_lookat = wizard_center_pos # Camera always looks at the wizard's center

            # Up vector can be tricky with arbitrary rotations. A common approach is to use world up
            # and then adjust if the camera is looking too far up/down. For simplicity, start with world up.
            # More robust: calculate from new_eye_pos and new_lookat
            view_dir = new_lookat - new_eye_pos
            if np.linalg.norm(view_dir) > 1e-6: view_dir /= np.linalg.norm(view_dir)
            
            # Calculate a 'right' vector that is horizontal
            temp_right = np.cross(np.array([0.0,1.0,0.0]), -view_dir) # temp_right = world_up x (-view_dir)
            if np.linalg.norm(temp_right) < 1e-6:
                # if view_dir is vertical, choose a default right (e.g. global X rotated by wizard's yaw)
                temp_right = rotate_vector(np.array([1.0,0.0,0.0]), self.wizard.walk_angle, "Y") 
            else: 
                temp_right /= np.linalg.norm(temp_right)

            self.view_up = np.cross(-view_dir, temp_right) # self.view_up = (-view_dir) x temp_right
            if np.linalg.norm(self.view_up) < 1e-6: self.view_up = np.array([0.0,1.0,0.0]) # Fallback
            else: self.view_up /= np.linalg.norm(self.view_up)

            return new_eye_pos, new_lookat
        else:
            # Standard third-person camera views (front, side, back - orbital around origin)
            # These views orbit (0,0,0) or a fixed point. `self.look_at_target` is that point.
            current_eye_pos = self.eye_pos # Start with the base eye_pos for the view mode
            
            # Calculate initial gaze vector based on mode's base eye_pos and look_at_target
            gaze_vector = self.look_at_target - current_eye_pos
            norm = np.linalg.norm(gaze_vector)
            unit_gaze = gaze_vector / norm if norm != 0 else np.array([0, 0, -1]) # Default gaze

            # Apply horizontal tilt (rotates eye_pos around look_at_target's Y axis)
            # To do this, translate eye_pos to origin, rotate, then translate back
            translated_eye_pos = current_eye_pos - self.look_at_target
            rotated_translated_eye_pos_h = rotate_vector(translated_eye_pos, self.tilt_angle_horizontal, "Y")
            
            # Vertical tilt disabled for orbital views – keep camera level
            final_eye_pos_offset = rotated_translated_eye_pos_h

            # Apply zoom along the gaze direction (horizontal orbit plane)
            if self.manual_zoom != 0:
                # Preserve original Y height; adjust only horizontal (XZ) distance
                current_offset = rotated_translated_eye_pos_h
                orig_height = current_offset[1]

                # Horizontal vector (XZ plane)
                horizontal_vec = np.array([current_offset[0], 0.0, current_offset[2]])
                horiz_len = np.linalg.norm(horizontal_vec)

                if horiz_len < 1e-6:
                    # If horizontal length is nearly zero, default to a small forward vector on -Z
                    horizontal_vec = np.array([0.0, 0.0, -1.0])
                    horiz_len = 1.0

                # Adjust distance by manual_zoom
                new_horiz_len = max(0.1, horiz_len + self.manual_zoom)  # prevent going through target
                horizontal_vec = horizontal_vec / horiz_len * new_horiz_len

                final_eye_pos_offset = np.array([horizontal_vec[0], orig_height, horizontal_vec[2]])
             
            new_eye_pos = self.look_at_target + final_eye_pos_offset
            new_lookat = self.look_at_target # These modes always look at self.look_at_target

            self.view_up = np.array([0.0, 1.0, 0.0]) # Standard up for these orbital views
            return new_eye_pos, new_lookat 
        # The return is already handled in the blocks above 

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
        self.arm_angle = -20.0
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
        self.staff_rotation = -90.0
        self.staff_radius = 0.3
        self.orb_radius = 1.0
        # Shooting animation attributes
        self.shooting = False
        self.shoot_timer = 0.0 
        self.shoot_phase = 0
        self.shoot_initial_arm_angle = 0.0
        self.shoot_raise_duration = 0.2
        self.shoot_lower_duration = 0.3
        self.shoot_target_arm_angle = 80.0
        self.pending_projectile = None
        self.projectile_ready = False
        
        # For AABB collision detection
        # Use a box that encompasses the wizard's body
        self.aabb_half_size = np.array([3.0, 6.0, 3.0])
        self.aabb_center = np.array([0.0, 0.0, 0.0])

        # Walking attributes
        self.walk_vector = np.array([0.0, 0.0, 0.0])
        self.walk_direction = np.array([0.0, 0.0, 1.0])
        self.walk_speed = 0.1
        self.walk_angle = 0.0

    def get_aabb(self):
        """Return min and max coordinates of the AABB for this wizard"""
        # Center the AABB on the wizard's position
        min_coords = self.walk_vector - self.aabb_half_size
        max_coords = self.walk_vector + self.aabb_half_size
        return min_coords, max_coords
        
    def get_wand_tip(self):
        """Return world-space position of the orb at the end of the staff."""
        # Approximate offset of the orb from the wizard origin (before rotation)
        local_offset = np.array([1.5, self.head_offset[1] - 2.5,  self.staff_length + 2.0])
        # Rotate offset by current facing angle
        rotated_offset = rotate_vector(local_offset, self.walk_angle, "Y")
        return self.walk_vector + rotated_offset

    # WASD walking logic is now handled in main.py; update_walk_vector is no longer needed.

    def start_shoot(self, projectile_start, projectile_dir):
        """Begin the shoot animation. The actual projectile will be spawned
        automatically once the staff reaches the peak of the animation."""
        if self.shooting:
            return  # Ignore if an animation is already running
        self.shooting = True
        self.shoot_timer = 0.0
        self.shoot_phase = 0
        self.shoot_initial_arm_angle = self.arm_angle
        self.pending_projectile = (projectile_start, projectile_dir)
        self.projectile_ready = False

    def update(self, dt):
        """Update wizard state. Returns (start_pos, dir_vec) when a projectile
        should be spawned, or None otherwise."""
        if not self.shooting:
            return None

        if self.shoot_phase == 0:  # Raising staff
            self.shoot_timer += dt
            progress = min(self.shoot_timer / self.shoot_raise_duration, 1.0)
            # Linear interpolation of the arm angle
            self.arm_angle = (1 - progress) * self.shoot_initial_arm_angle + progress * self.shoot_target_arm_angle
            if progress >= 1.0:
                # Reached peak – fire projectile and begin lowering
                self.shoot_phase = 1
                self.shoot_timer = 0.0
                self.projectile_ready = True
                return self.pending_projectile

        elif self.shoot_phase == 1:  # Lowering staff back to rest
            self.shoot_timer += dt
            progress = min(self.shoot_timer / self.shoot_lower_duration, 1.0)
            self.arm_angle = (1 - progress) * self.shoot_target_arm_angle + progress * self.shoot_initial_arm_angle
            if progress >= 1.0:
                # Animation finished
                self.shooting = False
                self.pending_projectile = None
        return None

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
        glRotatef(15.0, 0, 0, 1)
        glRotatef(90 + self.staff_rotation, 1, 0, 0)
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
