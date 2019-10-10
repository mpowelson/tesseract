# The start position of the robot
tesseract_msgs/Waypoint start

# All of the raster segments with approaches and departures
tesseract_msgs/ProcessSegmentDefinition[] segments

# All of the transition to/from a given segment. Must be same length as segments
tesseract_msgs/ProcessTransitionDefinition[] transitions
