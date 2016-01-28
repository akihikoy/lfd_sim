# Joint angles
float64[] JointAngles
# Poses (x,y,z,qx,qy,qz,qw) of links; [7]*(11)
float64[] LinkX
# Force/torque (fx,fy,fz,tx,ty,tz) of sensors and joints; [6]*(1+4)
float64[] Forces
# Masses of links; [11]
float64[] Masses
# Collision status (0: not colliding, 1: colliding)
int32[] Collisions

# Simulation time
float64 Time
