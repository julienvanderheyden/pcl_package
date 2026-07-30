# General primitive estimate, published every frame by the classification+fitting node.
# One message type generalizes over SPHERE, CYLINDER, and FLAT_BOX.

string primitive_type          # "SPHERE", "CYLINDER", "FLAT_BOX", or "UNKNOWN"
bool valid                     # false if no object / no confirmed classification this frame

geometry_msgs/PoseStamped pose # object pose (only meaningful if valid == true)

# Dimension fields - only the relevant ones are populated per type, others left at 0.
# SPHERE:    diameter
# CYLINDER:  diameter, height
# FLAT_BOX:  width, thickness, depth (depth is fixed assumption, not fitted)
float64 diameter
float64 height
float64 width
float64 thickness
float64 depth