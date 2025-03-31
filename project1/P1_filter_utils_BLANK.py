import numpy as np

#============ TODO: Task 7 - Implement Different Methdos for Out-Of-Bounds Pixels
'''
README: getPixelRepeated(), getPixelReflected(), and getPixelWrapped() all
have the same input arguments:
- data:     the image's data
- width:    the image's width
- height:   the image's height
- x:        the x coordinate of the pixel you are attempting to access
- y:        the y coordinate of the pixel you are attempting to access
'''

# Repeats the pixel on the edge of the image such that A,B,C,D looks like ...A,A,A,B,C,D,D,D...
# Return the pixel at (new_x, new_y)
def getPixelRepeated(data, width, height, x, y):
    new_x = 0 if x < 0 else min(x, width - 1)
    new_y = 0 if y < 0 else min(y, height - 1)
    return data[width * new_y + new_x]

# TODO: Reflect pixel values about the edge of the image such that A,B,C,D looks like ...C,B,A,B,C,D,C,B...
# Return the pixel at (new_x, new_y)
def getPixelReflected(data, width, height, x, y):
    

    pass # you may delete this once completing the function

# TODO: Wrap the image such that A,B,C,D looks like ...C,D,A,B,C,D,A,B
# Return the pixel at (new_x, new_y)
def getPixelWrapped(data, width, height, x, y):
    
    pass # you may delete this once completing the function

#============ END Task 7

# TODO: Task 6 - Implement Convolve2D
'''
Switch between different edge-pixel handling methods:
`edge_pixel_method`:
- 'Rep': use `getPixelRepeated` method
- 'Ref': use `getPixelReflected` method
- 'Wra': use `getPixelWrapped` method
'''
def convolve2D(data, width, height, kernel, edge_pixel_method):
    # TODO: Initialize a 1D numpy array (np.uint8), called 'result', to temporarily store your output image data

    # TODO: Obtain the side size of the input kernel (square, odd number)

    # TODO: Obtain the padding size based on kernel size

    # TODO: Rotate the kernel by 180 - think of a clever way to flip the kernel through indexing

    for row in range(height):
        for col in range(width):
            print(f"\r--- row: {row}, column: {col}", end='', flush=True)
            index = width * row + col # row-major order

            # TODO: Initialize redAcc, greenAcc, and blueAcc to store accumulated color channels, float


            # Iterate over the kernel using its dimensions

                    # TODO: get 'weight' from the kernel at the current position (k_row, k_col)
  

                    # TODO: Compute the corresponding pixel coordinates (img_x,img_y) based on the current kernel element and padding


                    # TODO: Get the pixel at (img_x, img_y) by handling out-of-bounding pixels - swtich between 'Rep', 'Rep', 'Wra' here

            
                    # TODO: Accumulate `weight * pixel` for each channel in redAcc, greeAcc, and blueAcc accordingly

        
            # TODO: Update `result`` with the new RGBA pixel value created from redAcc, greenAcc, and blueAcc

    
    # TODO: Copy the RGBA data from `result` to `data`


# TODO: Task 9 - Create a shift kerenl
'''
Goal: Create a 2D kernel that shifts an image `num` pixels in the given direction `shiftDir`

Args:
    shiftDir (str): "shiftLeft" or "shiftRight"
    num (int): Number of pixels to shift

Returns:
    shift kernel as a 1D list (row-major)
'''
def createShiftKernel(shiftDir, num):
    if shiftDir not in ["shiftLeft", "shiftRight"]:
        raise ValueError("shiftDir must be 'shiftLeft' or 'shiftRight'")

    # TODO: Determine kernel size (odd-numbered)


    # TODO: Initialize kernel as a 1D array with zeros


    # TODO: Set `1` at a proper position based on `shiftDir` and `num`




    # TODO: Return the kernel 

    
    pass # you may delete this once completing the function
    

# TODO: Task 11 - Implement Convolve1D with the row and column kernels from 
#                   a separable 2D kernel on the input image (i.e., `data`)
def convolve1D(data, width, height, row_kernel, column_kernel, edge_pixel_method = 'Rep'):
    # TODO: initialize a 1D array, `result`, to stored the processed image data


    # TODO: Slide row_kernel horizontally to each row of the input image (padded, using repeated method) 
    #           from top to bottom row and then update `result`


    # TODO: Slide column_kernel vertically to each column of `result`
    #           from left to right column and then update `result`


    # TODO: return `result` of the original image size


    pass # you may delete this once you complete this function


# TODO: Task 12 - Create a 1D triangle kernel based on any input kernel size (odd number)
def createTriangleKernel(kernel_size):
    # TODO: initial a 1D array as the kernel

    # TODO: determine the formula for deriving the weights within the kernel

    # TODO: iterate and fill up the kernel with weights based on the formula in a loop

    # TODO: return the kernel

    pass # you may delete this once you complete this function 


# TODO: Task 14 - Create a 1D Gaussian kernel based on an arbitrary input `blur radius``
def createGaussianKernel(blur_radius):
    # TODO: compute standard deviation and kernel size based on blur_radius, list your formulas here


    # TODO: initial a 1D array as the kernel with kernel size



    # TODO: iterate and fill up the kernel with weights in a loop based on the Gaussian function


    # TODO: return the kernel


    pass # you may delete this once you complete this function 