#  A transition plans from the start of segment[i] to the end of segment[i+1], this data can be used for finding collision free exit moves after cancelling an ongoing process
tesseract_msgs/Waypoint[] transition_from_start

# A transition plans from the end of segment[i] to the start of segment[i+1]
tesseract_msgs/Waypoint[] transition_from_end
