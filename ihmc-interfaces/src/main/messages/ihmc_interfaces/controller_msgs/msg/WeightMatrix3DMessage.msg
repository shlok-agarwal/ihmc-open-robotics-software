# This message is part of the IHMC whole-body controller API.
# This message allows the user to precisely choose the priority of each component of a taskspace command.

# The ID of the reference frame defining the weight frame.
# This reference frame defines the x axis, y axis, z axis for the weights.
# This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal to 0,
# the weight matrix will then be generated regardless to what frame is it used in.
int64 weight_frame_id 0
# Specifies the qp weight for the x-axis, if set to NaN the controller will use the default weight for this axis. The weight is NaN by default.
float64 x_weight
# Specifies the qp weight for the y-axis, if set to NaN the controller will use the default weight for this axis. The weight is NaN by default.
float64 y_weight
# Specifies the qp weight for the z-axis, if set to NaN the controller will use the default weight for this axis. The weight is NaN by default.
float64 z_weight