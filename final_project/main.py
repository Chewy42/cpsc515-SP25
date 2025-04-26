import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

from model import Camera, Wizard, rotate_vector

width, height = 800, 600  # Screen dimensions

def drawAxes():
    glLineWidth(3.0)
    glBegin(GL_LINES)
    # X axis (red)
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(100.0, 0.0, 0.0)
    # Y axis (green)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 100.0, 0.0)
    # Z axis (blue)
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, 0.0)
    glVertex3f(0.0, 0.0, 100.0)
    glEnd()

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
    pygame.init()
    glutInit()
    screen = (width, height)
    display_surface = pygame.display.set_mode(screen, DOUBLEBUF | OPENGL)
    pygame.display.set_caption('CPSC515: Transform & View - Wizard Model')
    
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (width / height), 0.1, 1000.0)
    glMatrixMode(GL_MODELVIEW)
    initmodelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)
    modelMatrix = glGetFloat(GL_MODELVIEW_MATRIX)
    
    wizard = Wizard()
    camera = Camera(view_mode="front")
    
    key_i_on = False   # head rotation increase
    key_o_on = False   # head rotation decrease
    key_l_on = False   # toggle arm/leg swinging
    moving_forward = False
    moving_backward = False
    moving_left = False
    moving_right = False

    clock = pygame.time.Clock()

    while True:
        bResetModelMatrix = False
        glPushMatrix()
        glLoadIdentity()
        
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                return
            if event.type == MOUSEMOTION:
                if pygame.mouse.get_pressed()[0]:
                    glRotatef(event.rel[1], 1, 0, 0)
                    glRotatef(event.rel[0], 0, 1, 0)
            if event.type == KEYDOWN:
                if event.key == K_0:
                    bResetModelMatrix = True
                elif event.key == K_SPACE:
                    camera.switch_view()
                elif event.key == K_i:
                    key_i_on = True
                elif event.key == K_o:
                    key_o_on = True
                elif event.key == K_l:
                    key_l_on = not key_l_on
                elif event.key == K_q:
                    camera.rotateViewLeft(-1.0);
                elif event.key == K_e:
                    camera.rotateViewLeft(1.0);
                elif event.key == K_w:
                    moving_forward = True
                elif event.key == K_s:
                    moving_backward = True
                elif event.key == K_a:
                    moving_left = True
                elif event.key == K_d:
                    moving_right = True
            if event.type == KEYUP:
                if event.key == K_i:
                    key_i_on = False
                elif event.key == K_o:
                    key_o_on = False
                elif event.key == K_w:
                    moving_forward = False
                elif event.key == K_s:
                    moving_backward = False
                elif event.key == K_a:
                    moving_left = False
                elif event.key == K_d:
                    moving_right = False

        # Update head rotation
        if key_i_on:
            wizard.head_rotation[0] += 1.0
        elif key_o_on:
            wizard.head_rotation[0] -= 1.0
        wizard.head_rotation[0] = max(-85.0, min(85.0, wizard.head_rotation[0]))
        
        # Swinging arms and legs
        if key_l_on:
            new_arm_angle = wizard.arm_angle + wizard.arm_direction * wizard.swing_speed
            if abs(new_arm_angle) > 30:
                overshoot = abs(new_arm_angle) - 30
                wizard.arm_angle = (np.sign(new_arm_angle) * 30) - (np.sign(new_arm_angle) * overshoot)
                wizard.arm_direction *= -1
            else:
                wizard.arm_angle = new_arm_angle

            new_leg_angle = wizard.leg_angle + wizard.leg_direction * wizard.swing_speed
            if abs(new_leg_angle) > 30:
                overshoot = abs(new_leg_angle) - 30
                wizard.leg_angle = (np.sign(new_leg_angle) * 30) - (np.sign(new_leg_angle) * overshoot)
                wizard.leg_direction *= -1
            else:
                wizard.leg_angle = new_leg_angle

        wizard.walk_direction = rotate_vector(np.array([0.0, 0.0, 1.0]), wizard.walk_angle, "Y")
        if moving_forward:
            wizard.walk_vector += wizard.walk_speed * wizard.walk_direction
        if moving_backward:
            wizard.walk_vector -= wizard.walk_speed * wizard.walk_direction
        if moving_left:
            left_direction = rotate_vector(np.array([0.0, 0.0, 1.0]), wizard.walk_angle + 90, "Y")
            wizard.walk_vector += wizard.walk_speed * left_direction
        if moving_right:
            right_direction = rotate_vector(np.array([0.0, 0.0, 1.0]), wizard.walk_angle - 90, "Y")
            wizard.walk_vector += wizard.walk_speed * right_direction

        if bResetModelMatrix:
            glLoadIdentity()
            modelMatrix = initmodelMatrix
            wizard.walk_vector = np.array([0.0, 0.0, 0.0])
            wizard.walk_angle = 0.0

        glMultMatrixf(modelMatrix)
        modelMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)

        glLoadIdentity()
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        new_eye_pos, new_lookat = camera.update_view()
        gluLookAt(new_eye_pos[0], new_eye_pos[1], new_eye_pos[2],
                  new_lookat[0], new_lookat[1], new_lookat[2],
                  camera.view_up[0], camera.view_up[1], camera.view_up[2])

        glMultMatrixf(modelMatrix)

        wizard.draw_Wizard()
        drawAxes()
        drawGround()

        glPopMatrix()
        pygame.display.flip()
        clock.tick(60)

if __name__ == "__main__":
    main()
