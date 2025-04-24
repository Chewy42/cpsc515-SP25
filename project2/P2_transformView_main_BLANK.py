import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

from P2_transformView_model_BLANK import Camera, Scarecrow

width, height = 800, 600  # width and height of the screen created

# drawing x, y, z axis in world space
def drawAxes():  # draw x-axis and y-axis
    glLineWidth(3.0)  # specify line size (1.0 default)
    glBegin(GL_LINES)  # replace GL_LINES with GL_LINE_STRIP or GL_LINE_LOOP
    glColor3f(1.0, 0.0, 0.0)  # x-axis: red
    glVertex3f(0.0, 0.0, 0.0)  # v0
    glVertex3f(100.0, 0.0, 0.0)  # v1
    glColor3f(0.0, 1.0, 0.0)  # y-axis: green
    glVertex3f(0.0, 0.0, 0.0)  # v0
    glVertex3f(0.0, 100.0, 0.0)  # v1
    glColor3f(0.0, 0.0, 1.0)  # z-axis: blue
    glVertex3f(0.0, 0.0, 0.0)  # v0
    glVertex3f(0.0, 0.0, 100.0)  # v1
    glEnd()

# drawing the ground
def drawGround():
    ground_vertices = [[-500, -12.6, -500],
                       [-500, -12.6, 500],
                       [500, -12.6, 500],
                       [500, -12.6, -500]]

    glColor3f(0.4, 0.4, 0.4)
    glBegin(GL_QUADS)
    for vertex in ground_vertices:
        glVertex3fv(vertex)
        
    glEnd()

def main():
    # --- Initialization ---
    pygame.init()  # Start pygame
    glutInit()  # Start OpenGL GLUT

    screen = (width, height)  # Set window size
    display_surface = pygame.display.set_mode(screen, DOUBLEBUF | OPENGL)  # Create OpenGL window
    pygame.display.set_caption('CPSC515: Transform & View - Matt Favela')  # Set window title

    glEnable(GL_DEPTH_TEST)  # Enable depth testing for 3D rendering
    glMatrixMode(GL_PROJECTION)  # Switch to projection matrix
    glLoadIdentity()  # Reset projection matrix
    zoom_fov = 45.0  # Set initial field of view (zoom)
    gluPerspective(zoom_fov, (width / height), 1.0, 1000.0)  # Set perspective projection

    glMatrixMode(GL_MODELVIEW)  # Switch to modelview matrix
    initmodelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)  # Save initial modelview matrix
    modelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)  # Working modelview matrix

    # --- Scarecrow and Camera Setup ---
    basic_scarecrow = Scarecrow(version="basic")  # Basic scarecrow
    upgraded_scarecrow = Scarecrow(version="upgraded")  # Upgraded scarecrow
    scarecrow = basic_scarecrow  # Default: basic scarecrow

    camera = Camera(view_mode="front")  # Default camera view

    # --- Key State Initialization ---
    key_i_on = False        # 'I' held
    key_o_on = False        # 'O' held
    key_l_on = False        # 'L' toggles walk-in-place animation
    key_r_on = False        # 'R' toggles freeform walking
    key_a_on = False        # 'A' held
    key_d_on = False        # 'D' held
    key_w_on = False        # 'W' held
    key_s_on = False        # 'S' held
    key_q_on = False        # 'Q' held
    key_e_on = False        # 'E' held

    zoom_fov = 45.0  # Redundant, already set above
    while True:  # --- Main Loop ---
        bResetModelMatrix = False
        glPushMatrix()
        glLoadIdentity()

        # --- Event Handling ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.MOUSEMOTION:
                if camera.view_mode == "first_person":
                    dx, dy = event.rel
                    scarecrow.walk_angle -= dx * 0.2
                    scarecrow.head_rotation[0] += dy * 0.2
                    scarecrow.head_rotation[0] = max(-85.0, min(85.0, scarecrow.head_rotation[0]))
                elif pygame.mouse.get_pressed()[0]:
                    glRotatef(event.rel[1], 1, 0, 0)
                    glRotatef(event.rel[0], 0, 1, 0)
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:  # Scroll up to zoom in
                    zoom_fov -= 2
                    zoom_fov = max(20, zoom_fov)
                    glMatrixMode(GL_PROJECTION)
                    glLoadIdentity()
                    gluPerspective(zoom_fov, (width / height), 1.0, 1000.0)
                    glMatrixMode(GL_MODELVIEW)
                elif event.button == 5:  # Scroll down to zoom out
                    zoom_fov += 2
                    zoom_fov = min(80, zoom_fov)
                    glMatrixMode(GL_PROJECTION)
                    glLoadIdentity()
                    gluPerspective(zoom_fov, (width / height), 1.0, 1000.0)
                    glMatrixMode(GL_MODELVIEW)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_0:
                    bResetModelMatrix = True
                elif event.key == pygame.K_SPACE:
                    prev_mode = camera.view_mode
                    camera.switch_view()
                    if camera.view_mode == "first_person" and prev_mode != "first_person":
                        pygame.event.set_grab(True)
                        pygame.mouse.set_visible(False)
                    elif prev_mode == "first_person" and camera.view_mode != "first_person":
                        pygame.event.set_grab(False)
                        pygame.mouse.set_visible(True)
                elif event.key == pygame.K_u:
                    if scarecrow == basic_scarecrow:
                        scarecrow = upgraded_scarecrow
                    else:
                        scarecrow = basic_scarecrow
                elif event.key == pygame.K_i:
                    key_i_on = True
                elif event.key == pygame.K_o:
                    key_o_on = True
                elif event.key == pygame.K_l:
                    key_l_on = not key_l_on
                    if not key_l_on:
                        key_r_on = False
                elif event.key == pygame.K_r:
                    key_r_on = not key_r_on
                    scarecrow = upgraded_scarecrow
                    if key_r_on:
                        key_l_on = True
                    else:
                        key_l_on = False
                elif event.key == pygame.K_a:
                    key_a_on = True
                elif event.key == pygame.K_d:
                    key_d_on = True
                elif event.key == pygame.K_w:
                    key_w_on = True
                elif event.key == pygame.K_s:
                    key_s_on = True
                elif event.key == pygame.K_q:
                    key_q_on = True
                elif event.key == pygame.K_e:
                    key_e_on = True

            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_i:
                    key_i_on = False
                elif event.key == pygame.K_o:
                    key_o_on = False
                elif event.key == pygame.K_a:
                    key_a_on = False
                elif event.key == pygame.K_d:
                    key_d_on = False
                elif event.key == pygame.K_w:
                    key_w_on = False
                elif event.key == pygame.K_s:
                    key_s_on = False
                elif event.key == pygame.K_q:
                    key_q_on = False
                elif event.key == pygame.K_e:
                    key_e_on = False

        # --- Handle Key States and Animation ---
        if key_i_on:
            scarecrow.head_rotation[0] += 1.0  # Tilt head up
        elif key_o_on:
            scarecrow.head_rotation[0] -= 1.0  # Tilt head down
        scarecrow.head_rotation[0] = max(-85.0, min(85.0, scarecrow.head_rotation[0]))  # Clamp head rotation

        # Animate arms and legs if walking or animation toggled
        if key_l_on or key_w_on or key_s_on or key_a_on or key_d_on:
            new_arm_angle = scarecrow.arm_angle + scarecrow.arm_direction * scarecrow.swing_speed
            if abs(new_arm_angle) > 30:
                overshoot = abs(new_arm_angle) - 30
                scarecrow.arm_angle = (np.sign(new_arm_angle) * 30) - (np.sign(new_arm_angle) * overshoot)
                scarecrow.arm_direction *= -1
            else:
                scarecrow.arm_angle = new_arm_angle
            
            new_leg_angle = scarecrow.leg_angle + scarecrow.leg_direction * scarecrow.swing_speed
            if abs(new_leg_angle) > 30:
                overshoot = abs(new_leg_angle) - 30
                scarecrow.leg_angle = (np.sign(new_leg_angle) * 30) - (np.sign(new_leg_angle) * overshoot)
                scarecrow.leg_direction *= -1
            else:
                scarecrow.leg_angle = new_leg_angle

        # Freeform walking (R key)
        if key_r_on:
            scarecrow.walk_speed = scarecrow.swing_speed * 0.5
            scarecrow.walk_vector += scarecrow.walk_speed * scarecrow.walk_direction

        # Move scarecrow in non-first-person mode
        if camera.view_mode != "first_person":
            if key_w_on:
                scarecrow.walk_vector += scarecrow.walk_speed * np.array([0.0, 0.0, 1.0])  # Forward
            if key_s_on:
                scarecrow.walk_vector += scarecrow.walk_speed * np.array([0.0, 0.0, -1.0])  # Backward
            if key_a_on:
                scarecrow.walk_vector += scarecrow.walk_speed * np.array([1.0, 0.0, 0.0])   # Left
            if key_d_on:
                scarecrow.walk_vector += scarecrow.walk_speed * np.array([-1.0, 0.0, 0.0])  # Right
            if key_q_on:
                camera.roll_angle -= 1.0  # Roll camera left
            if key_e_on:
                camera.roll_angle += 1.0  # Roll camera right

        # Reset model matrix and scarecrow position if requested
        if bResetModelMatrix:
            glLoadIdentity()
            modelMatrix = initmodelMatrix
            if scarecrow:
                 scarecrow.walk_vector = np.array([0.0, 0.0, 0.0])
                 scarecrow.walk_angle = 0.0 
                 scarecrow.walk_direction = np.array([0.0, 0.0, 1.0]) 

        # --- Apply Model Transformations ---
        glMultMatrixf(modelMatrix)  # Apply transformation matrix
        modelMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)  # Save current modelview matrix

        # --- Clear Screen ---
        glLoadIdentity()
        glClearColor(0.0, 0.0, 0.0, 1.0)  # Set background color
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # Clear color and depth buffers

        # --- Set Camera/View ---
        if camera.view_mode == "first_person":
            new_eye_pos, new_lookat = camera.update_view(scarecrow)  # First-person follows scarecrow
        else:
            new_eye_pos, new_lookat = camera.update_view()  # Other views
        gluLookAt(new_eye_pos[0], new_eye_pos[1], new_eye_pos[2],
                  new_lookat[0], new_lookat[1], new_lookat[2],
                  camera.view_up[0], camera.view_up[1], camera.view_up[2])

        glMultMatrixf(modelMatrix)  # Apply model matrix again after view

        # --- Draw Scarecrow ---
        if scarecrow.version == "basic":
            scarecrow.draw_Scarecrow(camera)
        elif scarecrow.version == "upgraded":
            scarecrow.draw_Scarecrow_Upgrade(camera)

        # --- Draw Scene Objects ---
        drawAxes()  # Draw axes
        drawGround()  # Draw ground
        # Draw three colored cubes at fixed positions
        glPushMatrix()
        glColor3f(1,0,0)
        glTranslatef(10,0,10)
        glutSolidCube(5)
        glPopMatrix()
        glPushMatrix()
        glColor3f(0,1,0)
        glTranslatef(-10,0,10)
        glutSolidCube(5)
        glPopMatrix()
        glPushMatrix()
        glColor3f(0,0,1)
        glTranslatef(0,0,-15)
        glutSolidCube(5)
        glPopMatrix()

        # --- End of Frame ---
        glPopMatrix()  # Restore matrix stack
        pygame.display.flip()  # Swap display buffers
        pygame.time.wait(10)  # Delay for ~10ms per frame

main()