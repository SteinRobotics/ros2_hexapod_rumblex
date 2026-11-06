# Source image timestamp and optical frame (+X right, +Y down, +Z forward).
std_msgs/Header header
# False means no usable observation; all remaining fields must be ignored.
bool valid
# Temporary association ID, not a person's identity.
uint32 track_id
float32 confidence
# Undistorted normalized ray coordinates, (u-cx)/fx and (v-cy)/fy.
float64 x_error
float64 y_error
# True only with matching measured camera calibration; false for desktop FOV approximation.
bool calibrated
