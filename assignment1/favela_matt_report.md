# Assignment 1 Report

By Matthew Favela

Describe your approach to implementing the task outline the key algorithms and techniques you used to address the task.

Explain any important design decisions and how they impact your implementation.

DO NOT copy code here, instead use my own words of my solution, i may reference the line numbers though

## Task 1

### Methods and Implementation

To complete the task of creating a grayscale canvas I needed to setup a few main components:
1. A way to convert a 32-bit float to 3, 8-bit integers for a total of 24 bits in size (Theoretically), is by mapping the [0,1] float to a [0,255] uint8 value.
2. A way to ensure that the canvas_type is set to grayscale.
3. To change the canvas_type to a grayscale mode.
My approach for creating the floatToInt() method is by taking the intensity float value and multiplying it by 255. Next to create the initGrayCanvas() function I needed to make the integer value either 0 or 255 if the intensity float value is a higher value than the grayscale threshhold. Afterwards, I updated the argument that defines the canvas_type to be set to grayscale. An important factor to also keep in mind is if the canvas_type color gets passed in, I would need to pass along the original intensity which is the float value * 255.

## Task 2

### Methods and Implementation
To create a heart in grayscale on the canvas I had to figure out a way to map the mouses x and y coordinates to the indexes of the matrix I defined. This is because one pixel is technically 50x50 pixels in a matrix of 10x10, so I had to map the coordinate between 0-500 between one of the 10 pixels it fell into. There is also an issue of the origin being set at the top-left when it should be in the bottom-left, so I had to do some transformations to reverse the origin used.
My approach to this problem was to:
1. Create a working way to map mouse_x and mouse_y coordinates to the correct indecies based on which "bucket" of pixels its in. This is to determine which group of 50x50 pixels belongs to each index.
2. Next, I would need a way of chaning the intensity of the group of pixels in that "bucket" to 255 intensity or ON.
3. Finally I wanted a way to print out the array indexes of a pixel I clicked on, so that I can compile a list of indexes to turn on their intensities.
My approach to this was going to be inside of the posToIndex method. I started by printing out the values and trying to transform them each time to the expected value. I ended up deciding on starting by flipping the origin by taking the row value, minus the width of array (in pixel size) * the size of a pixel. Then I multiplied this value by -1 to invert the value which made the correct value which was negative to a positive value. Next, I had to determine how many "cells" or in mouse coordinates would be equivalent to one pixel or "bucket" of 50x50 pixels in both the x and y direction, so I could figure out which index in the x and y values the pixel corresponded to such as [1,2] [3,6] etc. Finally I used the technique of multiplying the row by the width of pixels in the image that we defined, plus the index of the column in its row. Then i subtracted the number of cells or "buckets" on the y axis minus the index in the 1D array * -1 to invert the value and offset, to account for the y axis.


## Task 3

### Methods and Implementation

## Task 4

### Methods and Implementation

## Task 5

### Methods and Implementation

## Task 6

### Methods and Implementation