# lara8

- lara8_macro.xacro : macro that contains structure of robot
- lara8_limits : contains information about joint limits, joint velocity limits and joint effort lara8_limits
- lara8_kinematic_parameters_nomial : contains the nominal kinematic parameters 
- lara8_dynamic_parameters_nominal : contains the nominal dynamic parameters

exchange these files after calibration and recreate description files (urdf, sdf, ...)

to create the urdf :

xacro lara8.urdf.xacro > lara8.urdf

