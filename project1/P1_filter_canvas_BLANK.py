import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from collections import namedtuple
import numpy as np
import P1_filter_utils_BLANK

# TODO: Task 2 - This function should return the gray value of a pixel 
#       by computing a weighted sum of its red, green, and blue components.
#       Recommend use the "luma method".
#       Return the gray value as np.uint8 
def rgbaToGray(color):

    
    pass # you may delete this once you write code in this function

# Define a Canvas for drawing an image
class Canvas:
    def __init__(self, width=500, height=500, pixel_size = 1):
        self.width = width
        self.height = height
        self.pixel_size = pixel_size

        # Initialize the canvas data
        self.data = []
        self.initCanvas()

        # Status of filter application
        self.b_filtered = False

    def initCanvas(self):
        canvas_color = (0, 0, 0, 255) # black color
        self.data = np.array([canvas_color] * (self.width * self.height), dtype=np.uint8)

    def load_image(self, image_path):
        image = pygame.image.load(image_path)
        image_width, image_height = image.get_size() # get image dimensions

        # update teh canvas dimensions to match the image's aspect ratio
        self.width = image_width
        self.height = image_height

        # scale the image to fit the updated canvas dimensions
        image = pygame.transform.scale(image, (self.width, self.height)) 
        image_data = pygame.image.tobytes(image, "RGBA", True)  # Convert to raw pixel data
        self.data = np.frombuffer(image_data, dtype=np.uint8).reshape(self.width * self.height, 4).copy()
        
    # TODO: Task 1 - Implement Grayscale Filter:
    #       Iterate through every pixel in the image (i.e., self.data)
    #       Convert each pixel color into a grayscale value
    #       Update the image (self.data) with the grayscale value (np.uint8)
    def filterGray(self):
        # TODO: iterate each pixel in the image
       
        # TODO: call rgbaToGray() to convert rbg into grayscale

        # TODO: update self.data by updating current pixel's color by setting r,g,b with the same grayscale value

        pass # you may delete this once you complete this function
    
    # TODO: Task 3 - Implement Invert filter:
    #       Iterate through every pixel in the image (i.e., self.data)
    #       Inverting each color channel by subtracting its value from the maximum value, i.e., 255.
    #       Update the image (self.data) with the inverted r,g,b values (np.uint8)
    def filterInvert(self):
        # TODO: iterate each pixel in the image
      
        # TODO: invert each color component

        # TODO: Update self.data


        pass # you may delete this once you complete this function

    # TODO: Task 4 & Task 5 - Implement Brighten filter:
    #       Iterate through every pixel in the image
    #       Increase each color channel by 30%
    #       Update the image with the brightened r, g, b values (np.uint8)
    def filterBrighten(self):
        # TODO: iterate each pixel in the image


        # TODO: increase each RGB channel by 30%, but still within [0, 255]


        # TODO: Update self.data 

        
        pass # you may delete this once you complete this function

    # TODO: Task 8 - Create an identity filter kernel and call convolve2D() from FilterUtils.py with it.
    #   If your kernel is correct, convolving with the identity kernel returns the original image.
    def filterIdentity(self, edge_pixel_method):
        # TODO: Create a 3x3 Identity kernel as a 1D list (row-major order)
        kernel = [] 

        # Convolve the identity kernel to the image
        P1_filter_utils_BLANK.convolve2D(data=self.data, width=self.width, height=self.height, kernel=kernel, edge_pixel_method=edge_pixel_method)


    # TODO: Task 9 - Create a Shift filter kernel
    """
    Goal: Applies a shift filter to self.data using 2D convolution.

    Args:
        shiftDir (str): "shiftLeft" or "shiftRight"
        num (int): Number of pixels to shift 
    """
    def filterShift(self, shiftDir, num, edge_pixel_method):
        # TODO: Create a shift kernel by calling `createShiftKernel()`
        kernel = P1_filter_utils_BLANK.createShiftKernel(shiftDir=shiftDir, num=num)

        # TODO: Apply 2d convolution using the shift kernel
        P1_filter_utils_BLANK.convolve2D(data=self.data, width=self.width, height=self.height, kernel=kernel, edge_pixel_method=edge_pixel_method)

        pass # you may delete this once you complete this function


    # TODO: Task 10 - Implement Edge Detection using Sobel operators (Separable kernels)
    def edgeDetection(self, sensitivity = 1, edge_pixel_method = 'Rep'):
        # TODO: Convert the input image (self.data) into grayscale using `filterGray()`
        
        
        # TODO: Represent the separatable Sobel-x kernel (flipped) using a row vector and a column vector (list)
         
        
        # TODO: Represent the separatable Sobel-y kernel (flipped) using a row vector and a column vector (list)


        # TODO: Apply Sobel-x kernel, separated into a row vector and a column vector, to the original image using 1D convolution implemented in "P1_filter_utils.py"
        #         Store the processed image into, Gx, of the same size as the original image


        # TODO: Apply Sobel-y kernel, separated into a row vector and a column vector, to the original image using 1D convolution implemented in "P1_filter_utils.py"
        #         Store the processed image into, Gy, of the same size as the original image


        # TODO: Combine each the corresponding pixel values from Gx and Gy into G
        #         Apply sensibity parameter by multiplying it with G for each pixel color channel
        #         Then clamp the scaled gradient value to be [0,255]


        # TODO: Determine a threshold (>=) to determine "edges" in the image


        
        # TODO: Initialize a 1D numpy array (np.uint8), called 'result', of the same size as the input image, 
        #         Initialize the pixel color to be black



        # TODO: Set those pixels in `result` whose corresponding values in G exceed the threshold with color white



        # TODO: Copy `result` to self.data 



        pass # you may delete this once you complete this function



    # TODO: Task 13 - Implement Triangle Blur using a Triangle filter (separable kernels)
    def triangleBlur(self, kernel_size):
        # TODO: check if the input kernel size is value (i.e., odd number, >= 3)


        # TODO: create a 1D triangle kernel based on kernel_size by calling `createTriangleFilter` from "P1_filter_utils.py"


        # TODO: Apply two 1D triangle kernels sequentially to the image, self.data, using `convolve1D()` from "P1_filter_utils.py"


        # TODO: Update self.data


        pass # you may delete this once you complete this function


    # TODO: Task 15 - Implement Gaussian Blur using a Gaussian filter (separable kernels)
    def GaussianBlur(self, blur_radius):
        # TODO: create a 1D triangle kernel based on blur radius by calling `createGaussianFilter` from "P1_filter_utils.py"


        # TODO: Apply two 1D Gaussian kernels sequentially to the image, self.data, using `convolve1D()` from "P1_filter_utils.py"


        # TODO: Update self.data


        pass # you may delete this once you complete this function