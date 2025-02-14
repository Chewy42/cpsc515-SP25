import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *

from A1_pencil_canvas import Canvas

def main():
    pygame.init()                                                                                   # initialize a pygame program
    glutInit()                                                                                      # initialize glut library    
    display = (500, 500)                                                                            # specify the screen size of the new program window
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)                                            # create a display of size 'screen', use double-buffers and OpenGL
    pygame.display.set_caption('CPSC515: Pencils - Matt Favela')                                      # set title of the program window

    # Set up the OpenGL viewport and projection
    glViewport(0, 0, display[0], display[1])
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glOrtho(-1, 1, -1, 1, -1, 1)                                                                    # Set up a simple orthographic projection
    glMatrixMode(GL_MODELVIEW)

    # Initialize the canvas
    width, height = 10, 10                                                                          # canvas' resolution (different from the pygame window's)
    pixel_size = 50                                                                                 # Size of each pixel on canvas to fill the window
    canvas = Canvas(width=width, height=height, pixel_size=pixel_size, canvas_type="color")   # TODO: Switch `canvas_type` here for Task 1.3, Task 4.2

    # Main loop
    running = True
    left_mouse_held = False 
    right_mouse_clicked = False
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:                        # Left mouse button pressed
                left_mouse_held = True
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:                          # Left mouse button released
                left_mouse_held = False
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 3:
                right_mouse_clicked = True
        
        # Implement actions you'd like to do when the LEFT-button is held
        if left_mouse_held:
            mouse_x, mouse_y = pygame.mouse.get_pos()                                               # get mouse's pixel coordinates on the display
            print(f"mouse clicks on display at pixel ({mouse_x}, {mouse_y})")                        
            
            # TODO: Task 3.1: Convert mouse's (x,y) coordinates on display into canvas' (x, y) coordinates
            #   write your code below

            if(mouse_x < 0 or mouse_x > width*pixel_size or mouse_y < 0 or mouse_y > height*pixel_size):
                continue
            else:
                index = canvas.posToIndex(mouse_y, mouse_x)                                # convert canvas' (x, y) coordinates into an index
                #print("index: ", index)
                canvas_x = index % width
                canvas_y = index // width

                # uncomment the code below to verify your code
                # print(f"mouse clicks on canvas at pixel ({canvas_x}, {canvas_y})")                     
                
                # TODO: Task 3.2: Turn on the pixel being clicked - write your code below
                on = canvas.floatToInt(1.0)
                canvas.data[index] = (on, on, on, on)

                # TODO: Task 6: Implmement a "leaky" pencil - write your code below


        # Implement actions you'd like to do when the RIGHT-button is held
        # TODO: Task 5.2: Draw a flower - write your code below
        if right_mouse_clicked:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            canvas.draw_flower(mouse_y, mouse_x)
            right_mouse_clicked = False




        glClear(GL_COLOR_BUFFER_BIT)                                                                # clear the color buffer
        
        canvas.render()                                                                             # Draw the image on the canvas

        glFlush()                                                                                   # ensure drawing is done befoer flipping double-buffer
        pygame.display.flip()

    pygame.quit()

if __name__ == "__main__":
    main()