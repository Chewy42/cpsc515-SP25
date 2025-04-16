import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

from P2_transformView_model_BLANK import Camera, Scarecrow

width, height = 800, 600                                                    # width and height of the screen created

# drawing x, y, z axis in world space
def drawAxes():                                                             # draw x-axis and y-axis
    glLineWidth(3.0)                                                        # specify line size (1.0 default)
    glBegin(GL_LINES)                                                       # replace GL_LINES with GL_LINE_STRIP or GL_LINE_LOOP
    glColor3f(1.0, 0.0, 0.0)                                                # x-axis: red
    glVertex3f(0.0, 0.0, 0.0)                                               # v0
    glVertex3f(100.0, 0.0, 0.0)                                             # v1
    glColor3f(0.0, 1.0, 0.0)                                                # y-axis: green
    glVertex3f(0.0, 0.0, 0.0)                                               # v0
    glVertex3f(0.0, 100.0, 0.0)                                             # v1
    glColor3f(0.0, 0.0, 1.0)                                                # z-axis: blue
    glVertex3f(0.0, 0.0, 0.0)                                               # v0
    glVertex3f(0.0, 0.0, 100.0)                                             # v1
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
    pygame.init()                                                           # initialize a pygame program
    glutInit()                                                              # initialize glut library 

    screen = (width, height)                                                # specify the screen size of the new program window
    display_surface = pygame.display.set_mode(screen, DOUBLEBUF | OPENGL)   # create a display of size 'screen', use double-buffers and OpenGL
    pygame.display.set_caption('CPSC515: Transform & View - Matt Favela')     # set title of the program window

    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION)                                             # set mode to projection transformation
    glLoadIdentity()                                                        # reset transf matrix to an identity
    gluPerspective(45, (width / height), 0.1, 1000.0)                       # specify perspective projection view volume

    glMatrixMode(GL_MODELVIEW)                                              # set mode to modelview (geometric + view transf)
    initmodelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)
    modelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)

    # initialize the Scarecrow: body dimensions and transformation parameters 
    basic_scarecrow = Scarecrow(version="basic")
    upgraded_scarecrow = Scarecrow(version="upgraded")

    scarecrow = basic_scarecrow # by default, draw the basic Scarecrow

    # initialize the camera: camera parameters 
    camera = Camera(view_mode="front") # default view mode is "front"

    # initialize the states of all the designated keys
    key_i_on = False        # if key 'I' is HELD on now
    key_o_on = False        # if key 'O' is HELD on now
    key_l_on = False        # if key 'L' is PRESSED - Switch to turn on/off arm/leg swinging animation for walk-in-place 
    key_r_on = False        # if key 'R' is PRESSED - Switch to turn on/off freeform walking

    while True:
        bResetModelMatrix = False
        glPushMatrix()
        glLoadIdentity()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            if event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]:
                    glRotatef(event.rel[1], 1, 0, 0)
                    glRotatef(event.rel[0], 0, 1, 0)
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_0:
                    bResetModelMatrix = True
                elif event.key == pygame.K_SPACE:
                    camera.switch_view()
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
                    if key_r_on:
                        key_l_on = True
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_i:
                    key_i_on = False
                elif event.key == pygame.K_o:
                    key_o_on = False

        if key_i_on:
            scarecrow.head_rotation[0] += 1.0
        elif key_o_on:
            scarecrow.head_rotation[0] -= 1.0
        scarecrow.head_rotation[0] = max(-85.0, min(85.0, scarecrow.head_rotation[0]))

        if key_l_on:
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

        if key_r_on:
            scarecrow.walk_speed = scarecrow.swing_speed * 0.5
            scarecrow.walk_vector += scarecrow.walk_speed * scarecrow.walk_direction

        if bResetModelMatrix:
            glLoadIdentity()
            modelMatrix = initmodelMatrix
            if scarecrow:
                 scarecrow.walk_vector = np.array([0.0, 0.0, 0.0])
                 scarecrow.walk_angle = 0.0 
                 scarecrow.walk_direction = np.array([0.0, 0.0, 1.0]) 

        glMultMatrixf(modelMatrix)
        modelMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)

        glLoadIdentity()
        glClearColor(0.4, 0.4, 0.4, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        new_eye_pos, new_lookat = camera.update_view()
        gluLookAt(new_eye_pos[0], new_eye_pos[1], new_eye_pos[2],
                  new_lookat[0], new_lookat[1], new_lookat[2],
                  camera.view_up[0], camera.view_up[1], camera.view_up[2])

        glMultMatrixf(modelMatrix)

        if scarecrow.version == "basic":
            scarecrow.draw_Scarecrow()
        elif scarecrow.version == "upgraded":
            scarecrow.draw_Scarecrow_Upgrade()

        drawAxes()
        drawGround()

        glPopMatrix()
        pygame.display.flip()
        pygame.time.wait(10)

main()