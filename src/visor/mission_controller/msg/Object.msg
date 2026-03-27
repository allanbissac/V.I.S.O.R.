# This interface sends detected object centroids, a flag whether object has been detected, information about
# the colour of object and whether the object is a bin or a cube
# x, y, z: object centroids
# detected: informs whether a target object is detected or not
# colour: colour of the detected target
# object_type: informs whether detected target is bin or cube

float32 x
float32 y
float32 z
bool detected
int32 colour
bool object_type