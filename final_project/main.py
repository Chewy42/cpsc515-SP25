import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import time

from model import Camera, Wizard, rotate_vector
from model_cat import Panther # Import the Panther class

# Screen dimensions
WIDTH, HEIGHT = 800, 600

# Game constants
PATH_HALF_WIDTH = 75.0
HALLWAY_END = 600.0
DASH_DISTANCE = 5.0
DASH_COOLDOWN = 1.0  # seconds
GROUND_Y = -12.6 # Ground plane Y coordinate used for ray-intersection
NUM_PANTHERS = 30

# --- Utility Functions ---

def collision_test_aabbs(min_coords1, max_coords1, min_coords2, max_coords2):
    """AABB Collision detection function."""
    # Check range (min, max) overlaps for each axis
    for i in range(3):
        if min_coords1[i] > max_coords2[i] or max_coords1[i] < min_coords2[i]:
            return False
    return True

# --- Classes ---

class Projectile:
    def __init__(self, position, direction, speed=1.2, lifetime=3.0, radius=1.5):
        # Normalize direction
        direction = np.array(direction, dtype=float)
        norm = np.linalg.norm(direction)
        self.direction = direction / norm if norm != 0 else np.array([0.0, 0.0, 1.0])
        self.position = np.array(position, dtype=float)
        self.speed = speed
        self.lifetime = lifetime
        self.radius = radius
        self.alive_time = 0.0
        self.aabb_half_size = np.array([radius, radius, radius])

    def get_aabb(self):
        """Return min and max coordinates of the AABB for this projectile."""
        min_coords = self.position - self.aabb_half_size
        max_coords = self.position + self.aabb_half_size
        return min_coords, max_coords

    def update(self, dt):
        """Move projectile and check lifetime. Return True if still alive."""
        self.position += self.direction * self.speed
        self.alive_time += dt
        return self.alive_time < self.lifetime

    def draw(self):
        """Draw the projectile as a sphere."""
        quadratic = gluNewQuadric()
        glPushMatrix()
        glTranslatef(*self.position)
        glColor3f(1.0, 0.5, 0.0) # Orange color
        gluSphere(quadratic, self.radius, 16, 16)
        glPopMatrix()

class ProjectileManager:
    def __init__(self):
        self.projectiles = []

    def spawn(self, start_pos, direction):
        """Create and add a new projectile."""
        self.projectiles.append(Projectile(start_pos, direction))

    def update(self, dt):
        """Update all projectiles, removing ones that expire."""
        # Iterate over a copy for safe removal
        for p in self.projectiles[:]:
            if not p.update(dt):
                self.projectiles.remove(p)

    def draw(self):
        """Draw all active projectiles."""
        for p in self.projectiles:
            p.draw()

class Game:
    def __init__(self):
        """Initialize Pygame, OpenGL, and game components."""
        pygame.init()
        glutInit()
        self.screen_size = (WIDTH, HEIGHT)
        self.display_surface = pygame.display.set_mode(self.screen_size, DOUBLEBUF | OPENGL)
        pygame.display.set_caption('CPSC515 Final Project: Escape Pete the Panther')

        self._init_opengl()
        self._init_game_objects()
        self._init_game_state()

    def _init_opengl(self):
        """Configure OpenGL settings."""
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (WIDTH / HEIGHT), 0.1, 1000.0)
        glMatrixMode(GL_MODELVIEW)
        # Store the initial modelview matrix for resets
        self.init_model_matrix = glGetFloatv(GL_MODELVIEW_MATRIX)
        self.model_matrix = self.init_model_matrix.copy() # Start with the initial matrix

    def _init_game_objects(self):
        """Create wizard, panthers, camera, and projectile manager."""
        self.wizard = Wizard()
        self.panthers = []
        self._spawn_panthers(NUM_PANTHERS)

        self.camera = Camera(view_mode="front")
        self.camera.wizard = self.wizard # Link camera to wizard for FPV/Third Person
        self.projectile_manager = ProjectileManager()

    def _spawn_panthers(self, num_panthers):
        """Distribute panthers along the hallway."""
        if num_panthers <= 0: return
        # Calculate segment length excluding the start area (first 20 units)
        spawn_zone_length = HALLWAY_END - 40.0
        segment_len = spawn_zone_length / num_panthers

        for i in range(num_panthers):
            panther = Panther()
            # Calculate base Z position within the spawn zone
            base_z = 40.0 + segment_len * i
            # Add random offset within the first half of the segment for variation
            z_pos = base_z + np.random.uniform(0, segment_len * 0.5)
            # Random X position within hallway bounds (leaving some margin)
            x_pos = np.random.uniform(-PATH_HALF_WIDTH + 3.0, PATH_HALF_WIDTH - 3.0)
            # Place panther slightly above ground level (Y=0 is model base)
            panther.position = np.array([x_pos, 0.0, z_pos]) # Assuming Panther model base is at Y=0
            self.panthers.append(panther)

    def _init_game_state(self):
        """Initialize score, timers, and input flags."""
        self.score = 0
        self.last_dash_time = 0.0
        self.clock = pygame.time.Clock()
        self.running = True

        # Input states
        self.key_i_on = False
        self.key_o_on = False
        self.moving_forward = False
        self.moving_backward = False
        self.moving_left = False
        self.moving_right = False

    def run(self):
        """Main game loop."""
        while self.running:
            dt = self.clock.tick(60) / 1000.0 # Delta time in seconds

            self._handle_input()
            self._update(dt)
            self._render()

        pygame.quit()

    def _handle_input(self):
        """Process Pygame events for input."""
        reset_model_matrix = False

        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False
                return # Exit event loop immediately on quit

            # --- Mouse Input ---
            if event.type == MOUSEMOTION:
                self._handle_mouse_motion(event)
            elif event.type == MOUSEBUTTONDOWN:
                self._handle_mouse_button_down(event)

            # --- Keyboard Input ---
            elif event.type == KEYDOWN:
                reset_model_matrix = self._handle_key_down(event)
            elif event.type == KEYUP:
                self._handle_key_up(event)

        # Apply matrix reset if requested by key press
        if reset_model_matrix:
            self._reset_view()

    def _handle_mouse_motion(self, event):
        """Handle mouse movement for camera control."""
        # FPV / Third Person Look
        if self.camera.view_mode in ["fpv", "third_person_follow"]:
            dx, dy = event.rel
            sensitivity = 0.2
            self.camera.tilt_angle_horizontal -= dx * sensitivity
            self.camera.tilt_angle_vertical -= dy * sensitivity
            # Clamp vertical angle
            v_min, v_max = (-85.0, 85.0) if self.camera.view_mode == "fpv" else (-89.0, 89.0)
            self.camera.tilt_angle_vertical = max(v_min, min(v_max, self.camera.tilt_angle_vertical))
        # Orbital View Drag (if mouse not grabbed)
        elif pygame.mouse.get_pressed()[0] and not pygame.event.get_grab():
            dx, _ = event.rel
            sensitivity = 0.3
            self.camera.tilt_angle_horizontal -= dx * sensitivity # Horizontal orbit only

    def _handle_mouse_button_down(self, event):
        """Handle mouse button clicks for zoom and shooting."""
        if event.button == 4:  # Scroll up – zoom in
            self.camera.manual_zoom -= 2.0
        elif event.button == 5:  # Scroll down – zoom out
            self.camera.manual_zoom += 2.0
        elif event.button == 1:  # LMB click – shoot
            self._shoot_projectile(event.pos)

    def _shoot_projectile(self, mouse_pos):
        """Cast ray and shoot projectile if ground hit."""
        mx, my = mouse_pos
        hit_point = self._raycast_screen_to_ground(mx, my)
        if hit_point is not None:
            # NOTE: Assumes wizard has get_wand_tip() and start_shoot() methods
            start_pos = self.wizard.get_wand_tip()
            dir_vec = hit_point - start_pos
            self.wizard.start_shoot(start_pos, dir_vec) # Tell wizard to start animation

    def _raycast_screen_to_ground(self, mouse_x, mouse_y):
        """Cast a ray from camera through mouse pos to find ground intersection."""
        # Get current camera state
        eye_pos, look_at = self.camera.update_view() # Ensure view is up-to-date
        up_vec = self.camera.view_up

        # Build camera basis vectors
        forward = look_at - eye_pos
        f_norm = np.linalg.norm(forward)
        if f_norm < 1e-6: return None
        forward /= f_norm
        right = np.cross(forward, up_vec)
        r_norm = np.linalg.norm(right)
        if r_norm < 1e-6: return None
        right /= r_norm
        up = np.cross(right, forward) # Recalculate 'up' to ensure orthogonality
        u_norm = np.linalg.norm(up)
        if u_norm < 1e-6: return None
        up /= u_norm

        # Convert mouse coords to Normalized Device Coordinates (NDC)
        x_ndc = (2.0 * mouse_x) / WIDTH - 1.0
        y_ndc = 1.0 - (2.0 * mouse_y) / HEIGHT # Y is inverted

        # Project NDC to view space using FOV (must match projection matrix)
        fov_y_rad = np.deg2rad(45.0) # Match gluPerspective
        aspect = WIDTH / HEIGHT
        tan_half_fov = np.tan(fov_y_rad * 0.5)
        x_cam = x_ndc * tan_half_fov * aspect
        y_cam = y_ndc * tan_half_fov

        # Create world-space ray direction
        dir_world = (x_cam * right) + (y_cam * up) + (1.0 * forward)
        dir_world /= np.linalg.norm(dir_world) # Normalize

        # Ray-plane intersection (Plane: Y = GROUND_Y)
        # Ray: P = eye_pos + t * dir_world
        # Plane equation: N · (P - P0) = 0, where N = (0, 1, 0), P0 = (0, GROUND_Y, 0)
        # (0, 1, 0) · (eye_pos + t*dir_world - (0, GROUND_Y, 0)) = 0
        # eye_pos[1] + t*dir_world[1] - GROUND_Y = 0
        # t = (GROUND_Y - eye_pos[1]) / dir_world[1]
        denom = dir_world[1]
        if abs(denom) < 1e-6: return None # Ray parallel to ground
        t = (GROUND_Y - eye_pos[1]) / denom
        if t < 0: return None # Intersection behind camera

        intersection_point = eye_pos + t * dir_world
        return intersection_point

    def _handle_key_down(self, event):
        """Handle key press events."""
        reset_matrix = False
        current_time = time.time()

        if event.key == K_0:
            reset_matrix = True
        elif event.key == K_SPACE:
            self._toggle_camera_view()
        elif event.key == K_i: self.key_i_on = True
        elif event.key == K_o: self.key_o_on = True
        elif event.key == K_q: self.camera.rotateViewLeft(-1.0) # NOTE: Assumes Camera has rotateViewLeft
        elif event.key == K_e: self.camera.rotateViewLeft(1.0)  # NOTE: Assumes Camera has rotateViewLeft
        elif event.key == K_w: self.moving_forward = True
        elif event.key == K_s: self.moving_backward = True
        elif event.key == K_a: self.moving_left = True
        elif event.key == K_d: self.moving_right = True
        elif event.key == K_LEFT: # Dash Left
            if current_time - self.last_dash_time > DASH_COOLDOWN:
                # NOTE: Assumes wizard has walk_angle attribute
                dash_vec = rotate_vector(np.array([0.0, 0.0, 1.0]), self.wizard.walk_angle + 90, "Y") * DASH_DISTANCE
                # NOTE: Assumes wizard has walk_vector attribute
                self.wizard.walk_vector += dash_vec
                self.last_dash_time = current_time
        elif event.key == K_RIGHT: # Dash Right
             if current_time - self.last_dash_time > DASH_COOLDOWN:
                # NOTE: Assumes wizard has walk_angle attribute
                dash_vec = rotate_vector(np.array([0.0, 0.0, 1.0]), self.wizard.walk_angle - 90, "Y") * DASH_DISTANCE
                # NOTE: Assumes wizard has walk_vector attribute
                self.wizard.walk_vector += dash_vec
                self.last_dash_time = current_time
        elif event.key in (K_LSHIFT, K_RSHIFT):
             # NOTE: Assumes wizard has walk_speed attribute
            self.wizard.walk_speed = 0.3 # Sprint speed

        return reset_matrix # Return whether to reset the view

    def _handle_key_up(self, event):
        """Handle key release events."""
        if event.key == K_i: self.key_i_on = False
        elif event.key == K_o: self.key_o_on = False
        elif event.key == K_w: self.moving_forward = False
        elif event.key == K_s: self.moving_backward = False
        elif event.key == K_a: self.moving_left = False
        elif event.key == K_d: self.moving_right = False
        elif event.key in (K_LSHIFT, K_RSHIFT):
             # NOTE: Assumes wizard has walk_speed attribute
            self.wizard.walk_speed = 0.1 # Normal speed

    def _toggle_camera_view(self):
        """Switch camera view mode and manage mouse grab."""
        prev_mode = self.camera.view_mode
        self.camera.switch_view() # NOTE: Assumes Camera has switch_view
        new_mode = self.camera.view_mode

        is_interactive_view = new_mode in ["fpv", "third_person_follow"]
        was_interactive_view = prev_mode in ["fpv", "third_person_follow"]

        if is_interactive_view and not was_interactive_view:
            pygame.event.set_grab(True)
            pygame.mouse.set_visible(False)
            pygame.mouse.get_rel() # Consume any pending motion
        elif not is_interactive_view and was_interactive_view:
            pygame.event.set_grab(False)
            pygame.mouse.set_visible(True)

    def _reset_view(self):
        """Reset wizard position, angle, and the modelview matrix."""
        # NOTE: Assumes wizard has walk_vector and walk_angle attributes
        self.wizard.walk_vector = np.array([0.0, 0.0, 0.0])
        self.wizard.walk_angle = 0.0
        self.model_matrix = self.init_model_matrix.copy() # Reset matrix
        # We load identity before applying camera view anyway,
        # but resetting model_matrix ensures transformations don't accumulate after reset.
        glLoadIdentity() # Good practice to clear current matrix state before setting view

    def _update(self, dt):
        """Update all game elements."""
        self._update_wizard(dt)
        self._update_panthers(dt)
        self._update_projectiles(dt)
        self._check_collisions()
        self._check_game_over()

    def _update_wizard(self, dt):
        """Update wizard state: head rotation, movement, animations."""
        # --- Head Rotation ---
        # NOTE: Assumes wizard has head_rotation attribute (list/array)
        if self.key_i_on: self.wizard.head_rotation[0] += 1.0
        elif self.key_o_on: self.wizard.head_rotation[0] -= 1.0
        # Clamp head pitch
        self.wizard.head_rotation[0] = max(-85.0, min(85.0, self.wizard.head_rotation[0]))

        # --- Movement ---
        # Determine if wizard is moving based on input flags
        is_moving = self.moving_forward or self.moving_backward or self.moving_left or self.moving_right

        # Calculate movement vector based on input flags
        # NOTE: Assumes wizard has walk_vector, walk_speed, walk_angle attributes
        # NOTE: Assumes wizard has a walk_direction calculated or accessible
        # Let's assume walk_direction needs to be calculated based on walk_angle
        self.wizard.walk_direction = rotate_vector(np.array([0.0, 0.0, 1.0]), self.wizard.walk_angle, "Y")

        move_delta = np.zeros(3)
        if self.moving_forward: move_delta += self.wizard.walk_speed * self.wizard.walk_direction
        if self.moving_backward: move_delta -= self.wizard.walk_speed * self.wizard.walk_direction
        if self.moving_left:
            left_dir = rotate_vector(np.array([0.0, 0.0, 1.0]), self.wizard.walk_angle + 90, "Y")
            move_delta += self.wizard.walk_speed * left_dir
        if self.moving_right:
            right_dir = rotate_vector(np.array([0.0, 0.0, 1.0]), self.wizard.walk_angle - 90, "Y")
            move_delta += self.wizard.walk_speed * right_dir

        # Apply movement delta (simple addition for now, might need dt scaling if speed is per-second)
        # Let's assume walk_speed is distance per *frame* for simplicity matching original code.
        self.wizard.walk_vector += move_delta

        # Clamp wizard X position to stay within hallway bounds
        # NOTE: Assumes wizard AABB half-size is roughly 1.0 or less for this margin
        self.wizard.walk_vector[0] = max(-PATH_HALF_WIDTH + 1.0, min(PATH_HALF_WIDTH - 1.0, self.wizard.walk_vector[0]))
        # Prevent moving back past the start (optional)
        # self.wizard.walk_vector[2] = max(0.0, self.wizard.walk_vector[2])

        # --- Animation Update ---

        # Handle limb swing animation when moving (restoring original logic)
        # NOTE: Assumes wizard has shooting, arm_angle, arm_direction, leg_angle, leg_direction, swing_speed attributes
        if is_moving and not self.wizard.shooting:
            # Swing Arms
            new_arm_angle = self.wizard.arm_angle + self.wizard.arm_direction * self.wizard.swing_speed
            if abs(new_arm_angle) > 30:
                overshoot = abs(new_arm_angle) - 30
                self.wizard.arm_angle = (np.sign(new_arm_angle) * 30) - (np.sign(new_arm_angle) * overshoot)
                self.wizard.arm_direction *= -1
            else:
                self.wizard.arm_angle = new_arm_angle
            # Swing Legs (Opposite direction)
            new_leg_angle = self.wizard.leg_angle + self.wizard.leg_direction * self.wizard.swing_speed
            if abs(new_leg_angle) > 30:
                overshoot = abs(new_leg_angle) - 30
                self.wizard.leg_angle = (np.sign(new_leg_angle) * 30) - (np.sign(new_leg_angle) * overshoot)
                self.wizard.leg_direction *= -1
            else:
                self.wizard.leg_angle = new_leg_angle
        else:
            # Gradually return limbs to rest when not moving OR while shooting
            self.wizard.arm_angle *= 0.8
            self.wizard.leg_angle *= 0.8

        # Update wizard's internal state (e.g., shooting cooldown/animation)
        # NOTE: Assumes wizard has an update method handling animations (shooting)
        # This method might return projectile details if one needs spawning.
        proj_to_spawn = self.wizard.update(dt) # Pass dt for time-based animations
        if proj_to_spawn:
            self.projectile_manager.spawn(*proj_to_spawn)

    def _update_panthers(self, dt):
        """Update panther AI (simple chase behavior)."""
        # NOTE: Assumes panther has position and angle attributes
        # NOTE: Assumes wizard has walk_vector as its position
        wizard_pos = self.wizard.walk_vector
        for panther in self.panthers:
            direction_to_wizard = wizard_pos - panther.position
            direction_to_wizard[1] = 0.0 # Ignore vertical difference
            dist = np.linalg.norm(direction_to_wizard)

            if dist > 0.1: # Avoid division by zero and jittering at close range
                dir_unit = direction_to_wizard / dist
                 # Simple linear movement towards wizard
                 # NOTE: Consider scaling by dt if speed is per second
                panther_speed = 0.05 # Speed per frame, matching original
                panther.position += dir_unit * panther_speed
                # Update facing angle based on movement direction
                panther.angle = -np.rad2deg(np.arctan2(dir_unit[0], dir_unit[2]))
                # Clamp panther X position to stay within hallway bounds (leaving margin)
                panther.position[0] = max(-PATH_HALF_WIDTH + 2.0, min(PATH_HALF_WIDTH - 2.0, panther.position[0]))
                # Clamp panther Z position (optional, prevent going behind start?)
                # panther.position[2] = max(0.0, panther.position[2])

            # NOTE: Assumes Panther class might have its own update for animations, etc.
            # panther.update(dt) # If applicable

    def _update_projectiles(self, dt):
        """Update projectile positions and lifetimes."""
        self.projectile_manager.update(dt)

    def _check_collisions(self):
        """Check for collisions between projectiles, panthers, and wizard."""
        # --- Projectile vs Panther ---
        projectiles_to_remove = set()
        panthers_to_remove = set()

        for proj in self.projectile_manager.projectiles:
            if proj in projectiles_to_remove: continue # Already marked for removal
            proj_min, proj_max = proj.get_aabb()

            for panther in self.panthers:
                if panther in panthers_to_remove: continue # Already marked
                panther_min, panther_max = panther.get_aabb() # NOTE: Assumes Panther has get_aabb()

                if collision_test_aabbs(proj_min, proj_max, panther_min, panther_max):
                    projectiles_to_remove.add(proj)
                    panthers_to_remove.add(panther)
                    self.score += 1
                    break # Move to next projectile once this one hits

        # Remove hit objects after checking all collisions
        self.projectile_manager.projectiles = [p for p in self.projectile_manager.projectiles if p not in projectiles_to_remove]
        self.panthers = [p for p in self.panthers if p not in panthers_to_remove]

        # --- Wizard vs Panther ---
        wizard_min, wizard_max = self.wizard.get_aabb() # NOTE: Assumes Wizard has get_aabb()
        for panther in self.panthers:
             panther_min, panther_max = panther.get_aabb() # NOTE: Assumes Panther has get_aabb()
             if collision_test_aabbs(wizard_min, wizard_max, panther_min, panther_max):
                print("Game Over! You were caught by a panther.")
                self.running = False
                return # End game

    def _check_game_over(self):
        """Check for win/loss conditions."""
        if not self.running: return # Already ended by collision or quit

        # Win Condition: Reached the end
        # NOTE: Assumes wizard walk_vector[2] is the forward position
        if self.wizard.walk_vector[2] >= HALLWAY_END:
            print(f"You Win! Final Score: {self.score}")
            self.running = False

    def _render(self):
        """Render the entire scene."""
        # Store current matrix state
        glPushMatrix()
        glLoadIdentity() # Clear matrix before setting camera

        # Set camera view
        eye_pos, look_at = self.camera.update_view()
        gluLookAt(eye_pos[0], eye_pos[1], eye_pos[2],
                  look_at[0], look_at[1], look_at[2],
                  self.camera.view_up[0], self.camera.view_up[1], self.camera.view_up[2])

        # Apply global model transformations (if any, like world rotation - not used here)
        # This matrix accumulates rotations/translations from previous frames if not reset
        # For this game, the model_matrix seems mostly used for the initial state/reset
        # We apply it here before drawing world objects.
        glMultMatrixf(self.model_matrix)

        # --- Drawing ---
        glClearColor(0.1, 0.1, 0.2, 1.0) # Dark blue background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        self._draw_environment()
        # NOTE: Assumes Wizard has draw_wizard() method
        self.wizard.draw_Wizard()
        # NOTE: Assumes Panther has draw_panther() method
        for panther in self.panthers:
            panther.draw_Panther()
        self.projectile_manager.draw()

        # Restore matrix state
        glPopMatrix()

        # Update window title with score
        pygame.display.set_caption(f"Score: {self.score} | Camera: {self.camera.view_mode}")
        pygame.display.flip() # Swap buffers

    def _draw_environment(self):
        """Draw static environment elements."""
        self._draw_axes()
        self._draw_ground()
        self._draw_walls()
        self._draw_finish_line()

    def _draw_axes(self):
        """Draw X, Y, Z axes."""
        glLineWidth(3.0)
        glBegin(GL_LINES)
        # X (Red)
        glColor3f(1.0, 0.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0); glVertex3f(10.0, 0.0, 0.0) # Shorter axes
        # Y (Green)
        glColor3f(0.0, 1.0, 0.0)
        glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 10.0, 0.0)
        # Z (Blue)
        glColor3f(0.0, 0.0, 1.0)
        glVertex3f(0.0, 0.0, 0.0); glVertex3f(0.0, 0.0, 10.0)
        glEnd()
        glLineWidth(1.0) # Reset line width

    def _draw_ground(self):
        """Draw the ground plane."""
        # Extend ground further to avoid seeing edges
        ground_half_size = max(PATH_HALF_WIDTH * 2, HALLWAY_END)
        vertices = [
            [-ground_half_size, GROUND_Y, -ground_half_size],
            [-ground_half_size, GROUND_Y,  ground_half_size],
            [ ground_half_size, GROUND_Y,  ground_half_size],
            [ ground_half_size, GROUND_Y, -ground_half_size]
        ]
        glColor3f(0.4, 0.4, 0.4) # Dark grey
        glBegin(GL_QUADS)
        for v in vertices: glVertex3fv(v)
        glEnd()

    def _draw_walls(self):
        """Draw the hallway walls."""
        wall_height = 15.0 # Relative to ground plane
        wall_bottom_y = GROUND_Y
        wall_top_y = GROUND_Y + wall_height
        wall_start_z = 0.0
        wall_end_z = HALLWAY_END

        glColor3f(0.6, 0.6, 0.6) # Light grey
        glBegin(GL_QUADS)
        # Left Wall
        glVertex3f(-PATH_HALF_WIDTH, wall_bottom_y, wall_start_z)
        glVertex3f(-PATH_HALF_WIDTH, wall_top_y,    wall_start_z)
        glVertex3f(-PATH_HALF_WIDTH, wall_top_y,    wall_end_z)
        glVertex3f(-PATH_HALF_WIDTH, wall_bottom_y, wall_end_z)
        # Right Wall
        glVertex3f(PATH_HALF_WIDTH, wall_bottom_y, wall_start_z)
        glVertex3f(PATH_HALF_WIDTH, wall_top_y,    wall_start_z)
        glVertex3f(PATH_HALF_WIDTH, wall_top_y,    wall_end_z)
        glVertex3f(PATH_HALF_WIDTH, wall_bottom_y, wall_end_z)
        glEnd()

    def _draw_finish_line(self):
        """Draw the finish line marker on the ground."""
        finish_y = GROUND_Y + 0.01 # Slightly above ground to avoid z-fighting
        line_half_depth = 0.5
        z1 = HALLWAY_END - line_half_depth
        z2 = HALLWAY_END + line_half_depth

        glColor3f(0.0, 1.0, 0.0) # Bright Green
        glBegin(GL_QUADS)
        glVertex3f(-PATH_HALF_WIDTH, finish_y, z1)
        glVertex3f( PATH_HALF_WIDTH, finish_y, z1)
        glVertex3f( PATH_HALF_WIDTH, finish_y, z2)
        glVertex3f(-PATH_HALF_WIDTH, finish_y, z2)
        glEnd()

# --- Main Execution ---

if __name__ == "__main__":
    game = Game()
    game.run()
