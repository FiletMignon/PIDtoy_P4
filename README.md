# PIDtoy_P4

A wheel on a motor, position-controlled by adjustable PID parameters.\n
Written in Processing 4.
Drag sliders to set; mouseover and mousewheel up/down for nonlinear adjustment.
Drag setpoint to move it.
Click and drag the wheel to pull it with a spring, adding external force.

Known issues:
-setpoint drag position wraps back around at -90 degrees
-setpoint can go behind the graphs/sliders if wheel radius is large (just drag around the edge of the wheel where it should be to pull it from a distance)
-no actual values displayed for motor torque output, power, etc; various other simulation parameters that would be neat to see
-KP is calculated by radians, might not be an issue but the values won't mean anything useful
