import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np

# Draw an image using OpenGL functions
def draw_image(pixels, pixel_size, width, height, show_grid = False):
    # Draw the pixels 
    for index, pixel in enumerate(pixels):
        row_idx = index // width
        col_idx = index % width
        glColor4ub(pixel[0], pixel[1], pixel[2], pixel[3])
        x = -1 + col_idx * pixel_size / 250.0  # Map to [-1, 1] in X
        y = -1 + row_idx * pixel_size / 250.0  # Map to [-1, 1] in Y
        glRectf(x, y, x + pixel_size / 250.0, y + pixel_size / 250.0)

    # Draw grid lines
    if show_grid == True:
        draw_gridline(width=width, height=height, pixel_size=pixel_size)

# Draw the grid lines on canvas (optional)
def draw_gridline(width, height, pixel_size):
    glColor3f(0.5, 0.5, 0.5)  # Grid line color (dark gray)
    for row_idx in range(height + 1):
        y = -1 + row_idx * pixel_size / 250.0
        glBegin(GL_LINES)
        glVertex2f(-1, y)
        glVertex2f(1, y)
        glEnd()
    for col_idx in range(width + 1):
        x = -1 + col_idx * pixel_size / 250.0
        glBegin(GL_LINES)
        glVertex2f(x, -1)
        glVertex2f(x, 1)
        glEnd()


# Define a Canvas for drawing an image
class Canvas:
    def __init__(self, width=500, height=500, pixel_size = 1, canvas_type="randomColor", show_grid = False):
        self.width = width
        self.height = height
        self.pixel_size = pixel_size
        self.canvas_type = canvas_type
        self.show_grid = show_grid

        # Initialize the canvas data (list of RGBA namedtuples)
        self.data = []
        if canvas_type == "color":
            self.initColorCanvas()
        elif canvas_type == "grayscale":
            self.initGrayCanvas()
            self.show_grid = True
        elif canvas_type == "randomColor":
            self.data = [(r, g, b, 255) for r, g, b in ((np.random.randint(0, 256), np.random.randint(0, 256), np.random.randint(0, 256)) for _ in range(width * height))]

    # Function to convert intensity (float, [0, 1]) to an integer [0 - 255]
    #   Return grayscale as an integer
    def floatToInt(self, intensity):
        intensity = float(intensity)
        grayscale = 0.123 # initialize variable grayscale
        # TODO: Task 1.1: Write your code below
        if(self.canvas_type == "grayscale"):
            if(intensity > grayscale):
                grayscale = 255
            else:
                grayscale = 0
        elif(self.canvas_type == "color"):
            grayscale = intensity * 255
        return int(grayscale)

    # Function to convert (row, column) coordinates 
    #   into an index in a 1D array in row-majored order
    #   Return index
    def posToIndex(self, row, column):
        index = 0 # initialize variable index

        # TODO: Task 2.1: Write your code below
        row = row - self.width * self.pixel_size * -1
        column = column - self.height * self.pixel_size * -1
        num_cells_x = (self.pixel_size // self.width) * 2
        num_cells_y = (self.pixel_size // self.height) * 2
        row_index = num_cells_y - ((num_cells_x - (row // self.pixel_size)) * -1)
        column_index = (num_cells_y - (column // self.pixel_size)) * -1
        index = row_index * num_cells_x + column_index
        index = (num_cells_y - index)*-1
        return index  

    # Initialize the canvas data with grayscale (int, [0 - 255])
    def initGrayCanvas(self):
        # TODO: Task 1.2: Write your code below
        intensity = 0.123 # TODO: Task 1.4: Modify this to see the difference in grayscale
        #intensity = 0.85 # Made a lighter color
        grayscale = self.floatToInt(intensity)
        self.data = [(grayscale, grayscale, grayscale, 255) for i in range(self.width * self.height)]
        
        # TODO: Task 2.3: call your function `createHeart()` below
        #self.createHeart()


    # Initialize the canvas data with RGBA color data
    def initColorCanvas(self):
        constant_color = (0, 123, 123, 255) # (r,g,b,a)
        # TODO: Task 4.1: Write your code below
        self.data = [constant_color for i in range(self.width * self.height)]


    # Function to create a heart on a grayscale image
    #   Modify specific pixels in self.data by set the intensity to 1.0 so it draws a heart 
    def createHeart(self):
        # TODO: Task 2.2: Write your code below
        pixels_to_intensify = [25, 34, 36, 43, 47, 52, 58, 62, 65, 68, 73, 74, 76, 77]
        for pixel in pixels_to_intensify:
            intensity = 255
            intensity = self.floatToInt(intensity)
            self.data[pixel] = (intensity, intensity, intensity, 255)

    # Function to draw a flower centered onthe input position
    def draw_flower(self, x, y):
        # TODO: Task 5.1: Write your code below
        data_index = self.posToIndex(x, y)
        print(f"DATA INDEX: {data_index}")
        data_index = data_index - self.width
        print(f"DATA INDEX MINUS WIDTH{self.width}: {data_index}")
        pink = (245, 0, 145, 255)
        yellow = (236, 255, 0, 255)
        if(data_index-self.width < 0 or data_index+self.width > (self.width * self.height) - 1):
            return
        self.data[data_index] = pink
        self.data[data_index + 1] = yellow
        self.data[data_index - 1] = yellow
        self.data[data_index + self.width] = yellow
        self.data[data_index - self.width] = yellow

    # Function to render the image on the canvas using OpenGL
    def render(self):
        draw_image(pixels=self.data, pixel_size=self.pixel_size, width=self.width, height=self.height, show_grid=self.show_grid)
        

if __name__ == "__main__":
    canvas = Canvas(canvas_type="grayscale")
    canvas.render()