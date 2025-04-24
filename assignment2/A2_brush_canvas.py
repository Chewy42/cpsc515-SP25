import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
import numpy as np
import random


# Define a Canvas for drawing an image
class Canvas:
    def __init__(self, width=500, height=500, pixel_size = 1):
        self.width = width
        self.height = height
        self.pixel_size = pixel_size

        # Initialize the canvas data
        self.data = []
        self.initCanvas()

        # brush related variables
        self.brush_radius = 1
        self.brush_size = 3                 # mask's height or width (equal); TODO: update its value based on brush_radius in the code below
        self.brush_color = (0, 0, 0, 255)   # black brush by default
        self.brush_mask = []                # use a 1D array of brush_size * brush_size to store mask data
        self.density = 0.25
    
    def initCanvas(self):
        canvas_color = (255, 255, 255, 255) # background color
        
        self.data = np.array([canvas_color] * (self.width * self.height), dtype=np.uint8)
    
    def initBrush(self, brush_radius, brush_color, brush_type, density):
        self.brush_radius = brush_radius
        self.brush_color = brush_color
        self.density = density

        # TODO: Calculate self.brush_size based on brush_radius
        self.brush_size = (brush_radius * 2) + 1
        
        # Initialize a mask of different distributions based on brush type
        if brush_type == "constant":
            self.brush_mask = self.createConstantBrushMask()
        elif brush_type == "linear":
            self.brush_mask = self.createLinearBrushMask()
        elif brush_type == "quadratic":
            self.brush_mask = self.createQuadraticBrushMask()
        elif brush_type == "spray":
            self.brush_mask = self.createSprayPaintMask(self.density)
        elif brush_type == "speed":
            self.brush_mask = self.createSpeedBrushMask()

    #helper method to flatten a 2d mask to a 1d array
    def flattenMask(self, mask):
        flattened_mask = []
        for i in range(self.brush_size):
            for j in range(self.brush_size):
                flattened_mask.append(mask[i][j])
        return flattened_mask


    # TODO: Create a constant brush mask of brush_size x brush_size, 
    #       Populate the mask with 1s where the distance is within the radius and 0s elsewhere 
    #       Return the mask as a 1D array (float)
    def createConstantBrushMask(self):
        mask = np.zeros((self.brush_size, self.brush_size), dtype=float)
        
        # TODO: iterate and fill up the mask
        center = self.brush_radius
        for i in range(self.brush_size):
            for j in range(self.brush_size):
                if (i - center)**2 + (j - center)**2 <= self.brush_radius**2:
                    mask[i][j] = 1.0

        # TODO: flatten the mask into a 1D array
        return self.flattenMask(mask)
    

    # TODO: Create a linear brush mask of brush_size x brush_size,
    #       Linearly descrease the value at each pixel as moving away from the center (1.0)
    #           and the values at the edge is 0.
    #       Return the mask as a 1D array (float)
    def createLinearBrushMask(self):
        mask = np.zeros((self.brush_size, self.brush_size), dtype=float)
        
        # TODO: iterate and fill up the mask based on f(d)
        center = self.brush_radius
        for i in range(self.brush_size):
            for j in range(self.brush_size):
                dist = np.sqrt((i - center)**2 + (j - center)**2)
                if dist <= self.brush_radius:
                    mask[i][j] = 1.0 - (dist / self.brush_radius)
                else:
                    mask[i][j] = 0.0

        # TODO: flatten the mask into a 1D array
        return self.flattenMask(mask) # TODO: return the mask in a 1D array 
    

    def createQuadraticBrushMask(self):       
        # initialize the mask as an empty 2D array of brush_size x brush_size
        mask = np.zeros((self.brush_size, self.brush_size), dtype=float)
        
        A = -1.0 / (self.brush_radius**2)
        B = 0.0
        C = 1.0
        
        # Iterate and fill up the mask based on f(d)
        center = self.brush_radius
        for i in range(self.brush_size):
            for j in range(self.brush_size):
                distance = np.sqrt((i - center)**2 + (j - center) **2) #using the pythagorean theorem to calculate the distance
                if distance <= self.brush_radistanceius:
                    mask[i][j] = max(0.0, A * distance**2 + B * distance + C)
                else:
                    mask[i][j] = 0.0
    
        return self.flattenMask(mask)
    
    def createSprayPaintMask(self, density):
        mask = np.zeros((self.brush_size, self.brush_size), dtype=float)
        
        center = self.brush_radius
        for i in range(self.brush_size):
            for j in range(self.brush_size):
                if (np.random.rand() < density) and ((i - center)**2 + (j - center)**2 <= self.brush_radius**2):
                    mask[i][j] = density
                else:
                    mask[i][j] = 0.0
                    
        return self.flattenMask(mask)
    
    def createSpeedBrushMask(self):
        mask = np.zeros((self.brush_size, self.brush_size), dtype=float)
        center = self.brush_radius
        
        for i in range(self.brush_size):
            for j in range(self.brush_size):
                dist = np.sqrt((i - center)**2 + (j - center)**2)
                if dist <= self.brush_radius:
                    intensity = (1.0 - (dist / self.brush_radius)) ** 4
                    mask[i][j] = intensity
                else:
                    mask[i][j] = 0.0

        return self.flattenMask(mask)
    
    def get_coords_to_idx(self, canvas_x, canvas_y):
        return canvas_x * self.pixel_size + canvas_y

    def apply_brush(self, canvas_x, canvas_y):
        if canvas_x < 0 or canvas_x >= self.width or canvas_y < 0 or canvas_y >= self.height:
            return

        for i in range(self.brush_size):
            for j in range(self.brush_size):
                x = canvas_x - self.brush_radius + i
                y = canvas_y - self.brush_radius + j
                
                if(0 <= x < self.width and 0 <= y < self.height):
                    mask_idx = j * self.brush_size + i
                    canvas_idx = y * self.width + x
                    
                    mask_value = float(self.brush_mask[mask_idx])
                    
                    canvas_color = self.data[canvas_idx] / 255.0
                    brush_color = np.array(self.brush_color) / 255.0
                    
                    mixed_color = (1.0 - mask_value) * canvas_color + mask_value * brush_color
                    self.data[canvas_idx] = (mixed_color * 255.0).astype(np.uint8)
