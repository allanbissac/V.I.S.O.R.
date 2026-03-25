# This interface sends detected object centroids, a flag whether object has been detected, information about
# the colour of object and whether the object is a bin, a small cube or a medium sized cube.
# x, y, z: object centroids (in mm)
# detected: informs whether a target object is detected in the frame or not
# colour: colour of the detected target; 0: red, 1: green, 2: blue
# object_type: informs whether detected target is bin, small cube or medium cube; small cube: 0, medium cube: 1, bin: 2

float32 x
float32 y
float32 z
bool detected
int32 colour
int32 object_type