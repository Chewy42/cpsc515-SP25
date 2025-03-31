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
    pygame.display.set_caption('CPSC515: Transform & View - YOUR NAME')     # set title of the program window

    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION)                                             # set mode to projection transformation
    glLoadIdentity()                                                        # reset transf matrix to an identity
    gluPerspective(45, (width / height), 0.1, 1000.0)                       # specify perspective projection view volume

    glMatrixMode(GL_MODELVIEW)                                              # set mode to modelview (geometric + view transf)
    initmodelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)
    modelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)

    # initialize the Scarecrow: body dimensions and transformation parameters 
    scarecrow = Scarecrow(version="basic") # by default, draw the basic Scarecrow

    # initialize the camera: camera parameters 
    camera = Camera(view_mode="front") # default view mode is "front"

    # initialize the states of all the designated keys
    key_i_on = False        # if key 'I' is HELD on now
    key_o_on = False        # if key 'O' is HELD on now
    key_l_on = False        # if key 'L' is PRESSED - Switch to turn on/off arm/leg swinging animation for walk-in-place 
    key_r_on = False        # if key 'R' is PRESSED (not held) - Switch to turn on/off straightline/freeform walking
    key_left_on = False     # if left-arrow key is HELD on now
    key_right_on = False    # if right-arrow key is held on now
    key_a_on = False        # if key 'A' is HELD on now
    key_d_on = False        # if key 'D' is HELD on now
    key_w_on = False        # if key 'W' is HELD on now
    key_s_on = False        # if key 'S' is HELD on now
    key_q_on = False        # if key 'Q' is HELD on now
    key_e_on = False        # if key 'E' is HELD on now

    while True:
        bResetModelMatrix = False
        glPushMatrix()
        glLoadIdentity()

        #--------START: pygame.event.get()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()

            if event.type == pygame.MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]:
                    glRotatef(event.rel[1], 1, 0, 0)
                    glRotatef(event.rel[0], 0, 1, 0)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_0:                 # Press key '0' to reset the view by resetting the current model-view matrix
                    bResetModelMatrix = True
                elif event.key == pygame.K_SPACE:           # Press the space bar to switch view modes: front/side/back/first_person..
                    camera.switch_view()                    # TODO: Task 7: Switch front/side/back views here
                elif event.key == pygame.K_u:               # Press key 'U' to switch between the basic and upgraded Scarecrow
                    if scarecrow.version == "basic":
                        scarecrow.version = "upgraded"
                    elif scarecrow.version == "upgraded":
                        scarecrow.version = "basic"
                # add more key events and set key states here...

            

        
        #--------END: pygame.event.get()

        # TODO: Task 2: Update Scarecrow's head angle based on I, O key states



        # TODO: Task 4: Update Scarecrow's limb motion parameters (arm_angle, leg_angle) for walk-in-place 
        #        based on L, R key states

            
        
        # TODO: Task 5 and Task 6: Update Scarecrow's walk motion parameters 
        #       (walk_angle, walk_direction, walk_vector) for straightline walk and freeform walk  
        #        based on LEFT and RIGHT key states
        #       Call `update_walk_vector()` after updating walk_angle




        
        # TODO: Task 8: Update viewing parameters to transform (tilt, move forward/backward) the camera
        #       based on W/A/S/D/Q/E key states




        # When '0' is tapped, reset the view 
        if (bResetModelMatrix):
            glLoadIdentity()
            modelMatrix = initmodelMatrix
        glMultMatrixf(modelMatrix)
        modelMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)

        glLoadIdentity()
        
        # TODO: Task 7: Upate camera parameters: eye_position and look_at point
        new_eye_pos, new_lookat = camera.update_view()

        # Use updated camera parameters to update camera model
        gluLookAt(new_eye_pos[0], new_eye_pos[1], new_eye_pos[2], 
                  new_lookat[0], new_lookat[1],new_lookat[2],
                  camera.view_up[0], camera.view_up[1], camera.view_up[2])


        glMultMatrixf(modelMatrix)        

        # TODO: Task 1 & Task 3: Draw Scarecrow
        if scarecrow.version == "basic":        
            scarecrow.draw_Scarecrow()
        elif scarecrow.version == "upgraded":
            scarecrow.draw_Scarecrow_Upgrade()

        # draw other entities in the scene
        drawAxes()
        drawGround()

        glPopMatrix()
        pygame.display.flip()
        pygame.time.wait(10)

main()