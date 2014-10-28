%  Copyright (C) 2012, by Arturo Gil Aparicio
% 
%  This file is part of ARTE (A Robotics Toolbox for Education).
%  
%  ARTE is free software: you can redistribute it and/or modify
%  it under the terms of the GNU Lesser General Public License as published by
%  the Free Software Foundation, either version 3 of the License, or
%  (at your option) any later version.
%  
%  ARTE is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU Lesser General Public License for more details.
%  
%  You should have received a copy of the GNU Leser General Public License
%  along with ARTE.  If not, see <http://www.gnu.org/licenses/>.
% 
%    ARTE
%    A Robotics Toolbox for Education
% 
%    Version 3.2.3
% 
%  DEMOS
%        run demos to see all the demos
% 
%        demos/inversedynamics_2DOFplanar.m       -inverse dynamics for the 2 DOF planar mechanism
%        demos/inversedynamics_3DOFplanar.m       -inverse dynamics for the 3 DOF planar mechanism
%        demos/inversedynamics_puma560.m          -inverse dynamics for the UNIMATE puma 560 manipulator
%        demos/inversedynamics_stanford_arme.m    -inverse dynamics for the STANFORD ARM
%        demos/forwarddynamics_demo.m             -direct dynamics for the UNIMATE puma 560 manipulator
%        demos/kinematics_demo.m                  -direct and inverse kinematics demo for several robots.
% 

% 
%    LIST OF ROBOT PROGRAMMING LANGUAGES
%        RAPID    - currently, only ABB RAPID is supported.
% 
%  LIST OF SUPPORTED ROBOTS
%  EXAMPLE ROBOTS:        
% 
%  MANUFACTURER MODEL              Kinematics Dynamics 3D graphics
%  
%  ABB         IRB 140             |YES|      |NO|     |YES|  
%  ABB         IRB 6620            |YES|      |NO|     |YES|  
%  KUKA        KR5 arc             |YES|      |NO|     |YES|  
%  KUKA        KR5 scara R350 Z200 |YES|      |NO|     |YES|  
%  KUKA        KR5 sixx R650       |YES|      |NO|     |YES|  
%  KUKA        KR5 sixx R850       |YES|      |NO|     |YES|  
%  KUKA        KR90 R2700 pro      |YES|      |NO|     |YES|  
%  MITSUBISHI  PA-10 6DOF          |YES|      |YES|     |YES|  
%  UNIMATE     PUMA 560            |YES|      |YES|     |YES|  
%  --          STANFORD            |YES|      |YES|     |NO|  
%  EXAMPLE     SCARA               |YES|      |YES|     |NO|    
%  EXAMPLE     2 DOF PLANAR        |YES|      |YES|     |YES|   
%  EXAMPLE     3 DOF PLANAR        |YES|      |YES|     |YES|     
%  EXAMPLE     PRISMATIC           |YES|      |NO|     |NO| 

%    LIST OF FUNCTIONS
%        Please navigate to arte_lib/html/index.html to get a list of
%        functions
%
% Files
%   accel                            - [Qdd]= ACCEL(ROBOT, Q, Qd, TORQUE)
%   adjust_view                      - ADJUST_VIEW(ROBOT) 
%   animate                          - ANIMATE(ROBOT, Q) 
%   call_direct_dynamics             - qdd = call_direct_dynamics(input)
%   compute_configuration            - CONF= COMPUTE_CONFIGURATION(Q) 
%   compute_end_velocity             - computes the velocity V of the end effector as a
%   compute_jacobian                 - computes the conventional jacobian as a function
%   compute_joint_trajectory_indep   - [qt, qdt, qddt] = compute_joint_trajectory(q_ini, q_final, time_vector, qd_ini, qd_final)
%   compute_joint_velocity           - computes the velocity of the joint given the velocity
%   deg2rad                          - rad = deg2rad(deg)
%   demos                            - Copyright (C) 2012, by Arturo Gil Aparicio
%   dh                               - DENAVIT Compute an homogeneous transform matrix DH in terms of
%   direct_jacobian_demo             - DIRECT JACOBIAN DEMO
%   directkinematic                  - Direct Kinematic for serial robots.
%   draw_axes                        - DRAW_AXES(T, X_text, Y_text, Z_text, scale) 
%   draw_circle                      - DRAW_CIRCLE(POS, RADIUS)
%   draw_ellipse                     - DRAW_ELLIPSE(POS, COV, COLOR) 
%   draw_errors_scara                - PROPAGATE A GAUSSIAN ERROR DISTRIBUTION OF EACH JOINT TO AN ERROR IN
%   draw_errors_scara_monte_carlo    - THE DEMO PRESENTS AN ERROR PROPAGATION USING A MONTE-CARLO METHOD
%   draw_link                        - DRAW_LINK(ROBOT, I, T) 
%   draw_patch                       - DRAW_PATCH(F, V, C, transparent)
%   drawrobot3d                      - DRAWROBOT3D(ROBOT, Q)	3D drawing with DH reference systems of the robot at the
%   drawrobot3d_simulation           - ROBOT=DRAWROBOT3D_simulation(ROBOT, Q)	
%   find_first_in_zone_data          - index = find_first_in_zone_data(robot, q, T, radius)
%   find_singular_points             - LOOK FOR SINGULAR POINTS IN JOINT SPACE
%   follow_line_pa10                 - Copyright (C) 2012, by Arturo Gil Aparicio
%   follow_line_scara                - FOLLOW A LINE IN SPACE WITH A SCARA ROBOT
%   forwarddynamic                   - [T Q QD]= FORWARDDYNAMIC(ROBOT, TIME_END, Q0, Qd0, TAU, torqfun, varargin)
%   forwarddynamics_demo             - SCRIPT TEST THE DIRECT DYNAMICS OF THE PUMA 560 ROBOT
%   forwarddynamics_demo_3dof        - SCRIPT TEST THE DIRECT DYNAMICS OF THE PUMA 560 ROBOT
%   friction                         - Torque = FRICTION(ROBOT, QD, j)
%   get_conf_data                    - Obtain joint configuration values from robtarget data type
%   init_lib                         - init_lib: INITIALIZATION OF THE LIBRARY 
%   init_sim_variables               - initializes the variables needed
%   inversedynamic                   - TAU= INVERSEDYNAMIC: Compute inverse dynamics via recursive Newton-Euler
%   inversedynamics_2DOFplanar       - SCRIPT TEST FOR THE 2DOF arm
%   inversedynamics_3DOFplanar       - SCRIPT TEST FOR THE 3 DOF planar manipulator
%   inversedynamics_puma560          - SCRIPT TO TEST THE DYNAMICS OF THE PUMA 560 ROBOT
%   inversekinematic                 - Inverse kinematic for serial robots.
%   joint_references                 - JOINT_REFERENCES
%   kinematics_demo                  - SCRIPT TEST FOR THE KINEMATIC PROBLEM
%   load_motors                      - Loads a matrix containing the mechanical and electrical data
%   load_robot                       - Loads a data structure corresponding to the specified robot.
%   manufacturing_demo               - SCRIPT TO TEST THE GRAPHIC CAPABILITIES OF THE TOOLBOX: ROBOT IN A MANUFACTURING CELL
%   motor_selection                  - SCRIPT TO FIND THE TORQUES AT EACH JOINT FOR DIFFERENT MOTION STATES OF
%   MoveAbsJ                         - RAPID_WRAPPER: MoveAbsJ:
%   MoveC                            - RAPID_WRAPPER: MoveC: Make a circular path in space.
%   MoveJ                            - RAPID_WRAPPER: MoveJ:
%   MoveL                            - RAPID_WRAPPER: MoveL: Make a linear planning in space
%   normalize                        - Q = NORMALIZE(Q)
%   obtain_joint_speed               - compute joint speed according to speeddata variable
%   obtain_linear_speed              - compute linear end effector's speed according to speeddata variable
%   obtain_zone_data                 - given a tag specifying the zone data. Return a radius in meters
%   Offs                             - RAPID_WRAPPER: Offs(robot, robtarget, deltaX, deltaY, deltaZ, gripper, Wobj)
%   planners_demo                    - DEMO: Path planning
%   plot_every_robot_demo            - SCRIPT TEST TO LOAD ALL ROBOTS
%   plot_joint_data                  - PLOT_JOINT_DATA(robot)
%   plot_results_line                - PLOT_RESULTS_line
%   program_robot                    - DEMO: Simulate a RAPID programming
%   qprod                            - Q=QPROD(Q1, Q2)
%   qRot                             - Q = QROT(THETA, AXIS)
%   quaternion2T                     - T = quaternion2T(Q, P)
%   rad2deg                          - transform radians to degrees
%   read_graphics                    - ROBOT=READ_GRAPHICS(ROBOT)	
%   Reset                            - RAPID_WRAPPER: Reset:
%   select_closest_configuration     - Q=SELECT_CLOSEST_CONFIGURATION(ROBOT, Qinv, CONF)
%   select_closest_joint_coordinates - Q=SELECT_CLOSEST_JOINT_COORDINATES(ROBOT, Qinv, Qcurrent)
%   select_configuration             - Q=SELECT_CONFIGURATION(ROBOT, Qinv, CONF)
%   Set                              - RAPID_WRAPPER: Set:
%   single_joint_spline              - poly = spline(tacel, thetaini, thetafinal, velini, velfinal, acelini)
%   solve_spherical_wrist            - q = solve_spherical_wrist(robot, q, T, wrist)	
%   solve_spherical_wrist2           - q = solve_spherical_wrist(robot, q, T, wrist)	
%   spline                           - poly = spline(tacel, thetaini, thetafinal, velini, velfinal, acelini)
%   spot_welding_demo                - SCRIPT TO TEST THE GRAPHIC CAPABILITIES OF THE TOOLBOX: ROBOT IN A MANUFACTURING CELL
%   stl_read                         - [F, V, C] = STL_READ(filename)
%   synchronize                      - [velo2, tmax]= SINCHRONIZE(qini, qfinal, velocity) Finds a mean speed and the required 
%   T2quaternion                     - Q = T2quaternion(T)
%   teach                            - MATLAB code for teach.fig
%   test_initial                     - DEMO: Simulate a RAPID programming
%   test_joint_limits                - [joint, time] = test_joint_limits(robot)
%   test_joints                      - Test whether any of the joint angles exceeds the mechanical
%   test_rapid                       - DEMO: Simulate a RAPID programming
%   TPWrite                          - TPWrite: Print a message on the screen.
%   transform_to_homogeneous         - T=transform_to_homogeneous(robtarget)
%   transform_to_own                 - Script used to change the coordinates of the points stored in STL format
%   vect_arrow                       - VECT_ARROW(P0, P1, COLOR) 
%   WaitTime                         - WAITTIME(seconds) stops execution and waits for a specific amount of time
