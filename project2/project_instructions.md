# Project 2: Transformation and Viewing

This document contains guidelines and instructions for required tasks and extra credit[cite: 1]. Please carefully read and follow this guideline and refer to Lectures 6-9 to understand the necessary knowledge and terminology[cite: 3].

## 1. Introduction

This project has two parts:
1.  Build your first 3D character, animate their gait, and make them walk in the 3D scene using geometric transformations[cite: 4].
2.  Implement dynamic viewing by changing camera parameters through keyboard input[cite: 5].

### Objectives

1.  Understand how to build 3D models using basic shapes (cylinders, spheres, cubes) through geometric transformations and OpenGL functions[cite: 6].
2.  Learn how scene graphs and hierarchical modeling work and implement them for animating complex structures[cite: 7].
3.  Implement 3D viewing transformations using OpenGL functions[cite: 8].
4.  Develop a user-controlled camera model and 3D viewing transformations[cite: 8].

### Code Structure and Program Running

There are two Python files:
* `P2_transformView_main.py`: Main function (drawing, viewing, keyboard input)[cite: 9].
* `P2_transformView_model.py`: Contains 2 classes:
    * `Scarecrow`: Creates the 3D model and animates its walking, holding parameters for limb and walking motions[cite: 10].
    * `Camera`: Contains viewing parameters for camera control[cite: 11].

**Notes:**
* Follow the provided code structure and function settings[cite: 12].
* Use the designated keyboard input (see Look-Up Table)[cite: 13].
* The project has eight incremental tasks. Ensure each task works individually and collectively[cite: 14, 15].

## 2. Geometric Transformations

Assemble a 3D character, Scarecrow, using basic 3D shapes (cylinders, spheres, cubes) by scaling, rotating, or translating them[cite: 17]. First, build a basic Scarecrow, then upgrade it with more joints and limbs for walking animations[cite: 18].

### 2.1 Create a Scarecrow with a Rotatable Head

**Task 1: Create a Basic Scarecrow**
Create a scarecrow using basic shapes with specified dimensions and positions[cite: 19, 20].
* Head: Sphere, radius = 2.5, green[cite: 20].
* Nose: Cone (cylinder base radius = 0.3, top radius = 0.0, length = 1.8), red[cite: 21].
* Torso: Cylinder, radius = 2.5, length = 10, yellow[cite: 22].
* Legs: Cylinder, radius = 1.0, length = 12, red[cite: 22].
* Arms: Cylinder, radius = 1.0, length = 10, blue[cite: 22].
Default geometry creation places spheres at the origin, cylinder/cone center lines along the z-axis with bases in the x-y plane centered at the origin[cite: 23].
* **Code Location:** Write transformation code in `draw_Scarecrow()` within the `Scarecrow` class[cite: 24].

**Task 2: Rotate the Scarecrow's Head & Nose with Keyboard Input**
Rotate the head (and nose) left and right without moving other body parts using keyboard input[cite: 26].
* "i": Rotate head left, 1 degree at a time[cite: 26].
* "o": Rotate head right, 1 degree at a time[cite: 27].
* Confine `head_angle` rotation to [-85, 85] degrees[cite: 28].
* Ensure the nose rotates with the head[cite: 29].
* **Code Location:** Add transformation code to `draw_Scarecrow()`. Update `head_angle` based on key input in the main function[cite: 30, 31].

### 2.2 Upgrade the Scarecrow with More Joints and Animate Walk-in-Place

Upgrade the Scarecrow with more joints, limbs, hands, and feet[cite: 33]. Create a walk-in-place animation[cite: 34].

**Task 3: Upgrade the Scarecrow with More Joints**
In `draw_Scarecrow_Upgrade()`, modify the basic Scarecrow as follows[cite: 35]:
* Arms: Use two cylinders (upper/lower arm, same radius, half-length) connected by a spherical joint (same radius/color). Total length remains the same. Initial angle with y-axis is 15 degrees[cite: 35, 36].
* Legs: Use two cylinders (upper/lower leg, same radius, half-length) connected by a spherical joint (same radius/color). Total length remains the same[cite: 37, 38].
* Hands: Attach spheres (radius 1.1, same color as head) to arm ends[cite: 39, 40].
* Feet: Attach scaled cubes (size 1.5, scaled longer x1.8, flatter x0.8) to leg ends using `glutSolidCube()` and OpenGL transformations[cite: 41, 42, 43].
* Key "u": Switch between basic and upgraded Scarecrow[cite: 44].
* **Code Location:** Write creation and transformation code in `draw_Scarecrow_Upgrade()` within the `Scarecrow` class[cite: 45]. Switching code is already in `main`[cite: 46].
* **Report:** Build a scene graph for the upgraded Scarecrow, showing shapes, dimensions, local transformations, and parent-child relationships[cite: 48, 49, 50]. The root has "upper body" and "lower body" children[cite: 51]. Detail the hierarchy (trunk, arms, legs, head, nose, feet) and associated transformations (e.g., Rotation $R_{axis}(angle)$, Translation $T(x,y,z)$) consistent with OpenGL code[cite: 52, 53, 54, 55, 56, 57, 58, 59]. Refer to Lecture 6 ("Scene Graph") and Lecture 7 Exercise 3 ("Construct a Joint Structure")[cite: 60, 61].

**Task 4: "Walk in Place" Animation**
Simulate walking by swinging arms and legs[cite: 63]. Opposite arms/legs move together (e.g., right arm/left leg forward, left arm/right leg backward)[cite: 63].
* Arms: Lower arm rotates more than upper arm when moving forward past the torso. Arm straightens as it moves backward[cite: 64, 65].
* Legs: Lower leg rotates more than upper leg when moving backward past the torso. Leg straightens as it moves forward[cite: 66, 67, 68].
* Swinging angles (`arm_angle`, `leg_angle`) range [-30, 30] (adjustable)[cite: 69, 70]. Still position has limbs aligned with torso (angles = 0)[cite: 71].
* Use `swing_speed` to update angles for adjustable speed[cite: 72].
* Key 'l': Toggle walk-in-place animation (default off)[cite: 73, 74].
* **Code Location:** Write transformation code in `draw_Scarecrow_Upgrade()`. Update limb motion parameters based on key input in the main function[cite: 74, 75].

### 2.3 Scarecrow Freeform Walking with Keyboard Input Control

Allow the Scarecrow to walk along an adjustable direction[cite: 78].

**Task 5: Walk in a Straight Line**
Default walk along the positive z-axis (`walk_direction` = (0,0,1)) at `walk_speed` = 0.5[cite: 79]. Translate the Scarecrow by `walk_vector` each iteration: `walk_vector += walk_speed * walk_direction`[cite: 79].
* Synchronize `walk_speed` with limb swing speed for realism[cite: 80].
* Key 'r': Toggle freeform walking (default off). Enables walk-in-place automatically[cite: 81, 82, 83].
* **Code Location:** Write code in `draw_Scarecrow_Upgrade()`. Key input handling in the main function[cite: 83, 84].

**Task 6: Freeform Walk with Keyboard Control**
Default facing +z direction, walk along `walk_direction` = (0,0,1) at `walk_speed`[cite: 85]. Use arrow keys to rotate `walk_direction` around the y-axis by `walk_angle`, then translate by `walk_vector`[cite: 86]. Rotate the whole body by `walk_angle` accordingly[cite: 87].
* Key '←': Update `walk_angle`, rotate body left, translate based on transformed `walk_direction`[cite: 88].
* Key '→': Update `walk_angle`, rotate body right, translate based on transformed `walk_direction`[cite: 89].
* Default (no arrow key press while 'r' is active): Walk straight along current `walk_direction`[cite: 90].
* **Code Location:**
    * Complete `rotate_vector(vector, angle, axis)`: Rotates input vector by angle (degrees) around x, y, or z-axis using a 3x3 rotation matrix (use Numpy). Normalize `walk_direction` after rotation[cite: 91, 92, 93, 94, 95].
    * Complete `update_walk_vector()` in `Scarecrow` class: Rotate current `walk_direction` by `walk_angle` using `rotate_vector()`, then update `walk_vector` using Eq.1[cite: 96, 97].
    * In `draw_Scarecrow_Upgrade()`: Rotate and translate the entire Scarecrow using `walk_angle` and `walk_vector`[cite: 97].
    * Key input handling in the main function. Allow holding arrow keys for continuous `walk_angle` update. Call `update_walk_vector()` after update[cite: 98, 99, 100].

## 3. Viewing Transformations

Implement dynamic 3D viewing by adjusting camera parameters (`gluLookAt()`) using keyboard control[cite: 101, 103]. Use perspective projection (`gluPerspective()`) throughout[cite: 102].

**Task 7: Switch Between Front, Side, and Back Views**
Implement three view modes with view-up vector (0, 1, 0)[cite: 105, 106].
* Front view (Default): Camera on positive z-axis, looking negative z. Sees Scarecrow front[cite: 106]. Already implemented[cite: 107].
* Side view: Camera on positive x-axis, looking negative x. Place camera for clear view, no clipping. Good for walk-in-place animation[cite: 108, 109, 110].
* Back view: Camera behind, looking at Scarecrow's back diagonally. Position for clear view of walking animation over time[cite: 111, 112].
* Space bar: Switch views: Front -> Side -> Back -> Front[cite: 113, 114, 116]. Mouse rotation is still possible but space bar follows the defined sequence[cite: 115, 116].
* **Code Location:** Complete `switch_view()` in the `Camera` class. Initialize parameters for 'side' and 'back' views and implement switching logic. Function already called in `main`[cite: 117, 118, 119].

**Task 8: Dynamic Viewing with Keyboard Control**
Update camera position and look-at point based on real-time key input[cite: 121].
* Horizontal Rotation:
    * Hold 'a': Rotate camera view left[cite: 122].
    * Hold 'd': Rotate camera view right[cite: 123].
    * Update `tilt_angle_horizontal`, rotate current gaze vector using `rotate_vector()`, calculate new look-at point. Eye position constant[cite: 124, 125, 126].
* Vertical Rotation:
    * Hold 'w': Rotate camera view upward[cite: 127].
    * Hold 's': Rotate camera view downward[cite: 128].
    * Update `tilt_angle_vertical`, rotate gaze vector, calculate new look-at point. Eye position constant[cite: 129, 130, 131].
* Forward/Backward Movement:
    * Hold 'q': Move camera forward along gaze vector[cite: 132].
    * Hold 'e': Move camera backward along gaze vector[cite: 133].
    * Update `zoom_distance`, update camera position using a vector (unit gaze direction * `zoom_distance`)[cite: 134, 135].
* **Code Location:**
    * Update camera parameters inside `update_view()` within the `Scarecrow` class (function already called in `main`)[cite: 136, 137].
    * Update viewing parameters (`tilt_angle_horizontal`, `zoom_distance`, etc.) based on key input in the main function before `update_view()` is called. Allow holding keys for continuous updates[cite: 137, 138].

## 4. Extra Credit

**First-Person View:**
Implement a first-person view from the Scarecrow's head perspective[cite: 140, 141, 142].
* Calculate real-time camera position: Track the head's exact position during walking animation by updating with `walk_vector`[cite: 143, 144].
* Calculate real-time look-at point: Track head's facing direction. Start with (0, 0, 1), rotate by body rotation (`walk_angle`) and head rotation (`head_angle`) using matrices[cite: 145, 146, 147]. Calculate look-at point from the updated facing direction[cite: 148]. Assume view-up vector remains constant[cite: 149].
* Enrich the 3D scene with objects (cubes, etc.) to better demonstrate the first-person view[cite: 150].
* **Code Location:**
    * Create a new view mode "first_person" in `update_view()`, switchable with the space bar[cite: 151].
    * Adjust camera parameters based on head transformation when in first-person view[cite: 152].
    * Disable original camera keyboard inputs (a/d/w/s/q/e) in this mode[cite: 153].

**Other Ideas:** Implement your own 3D animation or dynamic camera control (check with instructor)[cite: 153, 154].

## Look-Up Table for Keyboard Input

Use these keys exactly[cite: 155]:
* "i" / "o": Rotate Scarecrow head left/right[cite: 156].
* "l": Toggle walk-in-place animation[cite: 156].
* "r": Toggle freeform walking (limb motion auto-enabled, default z-axis walk)[cite: 156, 157].
* "←" / "→": Turn Scarecrow left/right during freeform walk[cite: 158].
* "a" / "d": Tilt camera left/right[cite: 159].
* "w" / "s": Tilt camera up/down[cite: 160].
* "q" / "e": Move camera forward/backward[cite: 161].
* "u": Switch Scarecrow version (basic/upgraded)[cite: 162].
* Space bar: Switch camera view (front -> side -> back -> first_person [if implemented])[cite: 162].
* "0": Reset view (already implemented)[cite: 163].

## Grading

* Task 1 & 2 (Rotatable Head): 15 pts
* Task 3 & 4 (Upgraded Scarecrow, Walk-in-Place): 25 pts
* Task 5 & 6 (Freeform Walking): 20 pts
* Task 7 & 8 (Dynamic Viewing): 15 pts
* Software Engineering/Efficiency/Stability: 5 pts
* Written Report (Clarity, Formatting, Comprehensiveness): 20 pts
* Extra Credit (First-Person View / Own Choice): Up to 20 pts each

## Project 2 Submission Guideline

* **(80%) Code:** TWO Python files (`P2_transformView_model.py`, `P2_transformView_main.py`) with complete, running code for all required tasks and extra credit. Modify window title to include your name ("CPSC515: Transformation & Viewing - YOUR NAME")[cite: 163, 164, 165].
* **(20%) Write-up (PDF) and 1 x Demo Video (MP4):** [cite: 166]
    * **For each Task (1-8 & Extra Credit):**
        * **Method:** Briefly explain implementation details (transformations, OpenGL functions, parameters, formulas, keyboard input integration, scene graph for Task 3, mathematical calculations for Task 6 & 8, algorithm for Extra Credit)[cite: 167, 168, 170, 172, 173, 179, 182, 187, 189, 192, 195, 196, 197, 201].
        * **Result:** Refer to Task 7 results for Tasks 1 & 3. For others, provide screen-recording video demonstrating the functionality clearly (key presses not needed, just outcome). Show start/pause, turning, view switching, camera tilting/zooming as required per task. For Extra Credit, show the first-person view during dynamic walking in an enriched scene[cite: 168, 169, 173, 180, 181, 185, 186, 190, 191, 193, 194, 199, 200, 202, 203].
    * **Formatting:** Well-organized, clear, structured report with headings, bullets, concise explanations, labeled screenshots[cite: 204, 205].
    * **Demo Video:** ONE MP4 file (combine clips if needed). No narration needed. Brief and effective. Clearly annotate which task is demonstrated. Use built-in screen recorders or online converters if needed. Show the entire program window with your name in the title[cite: 205, 206, 207, 208, 209, 210].
* **Using AI/Internet Resources:** Use to support learning, but final work must be your own understanding/implementation. Cite sources properly in the report, explaining their contribution. Refer to syllabus for guidelines[cite: 211, 212, 213].