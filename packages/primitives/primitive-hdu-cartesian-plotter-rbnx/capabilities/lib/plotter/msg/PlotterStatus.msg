# Periodic state of the plotter primitive.
bool   connected
bool   busy
string last_error

# Last known / commanded position, mm.
float64 x
float64 y
float64 z

# Current pen heights, mm.
float64 z_up
float64 z_down

# Drawable area in machine mm. The paper corner is the origin (0,0),
# so x/y minima are always 0 and only the maxima are reported.
float64 area_x_max
float64 area_y_max

# IDLE | MOVING | HOMING | ERROR | DISCONNECTED
string state
# Monotonic counter for this stream.
uint64 seq