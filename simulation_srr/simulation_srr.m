% the x axis is parallel to long/easy axis of the satellite, the y and z
% axis are parallel to each corresponing short/hard axis of the satellite

unit = 3; % number of stacked cubesat units

% dimensions of satellite
height = unit*0.1; % points along x axis
length = 0.1; % points along y axis
width = 0.1; % points along z axis

volume = length*width*height; % [m^3]
mass_max = unit*1.33; % [kg], 1.33kg is the max weight of a cubesat unit
I_x = 1/12*mass*(width^2+length^2); % [kg*m^2], inertia around x-axis
I_y = 1/12*mass*(width^2+height^2); % [kg*m^2], inertia around y-axis
I_z = 1/12*mass*(length^2+height^2); % [kg*m^2], inertia around z-axis
I = [I_x 0 0; 0 I_y 0; 0 0 I_z]; % [kg*m^2], inertia tensor
omega = [0; 0; 0]; % [1/s], rotational speed of satellite

% the geometrical reference frame is placed at the exact center of the cubesat
mass_payload = 1.2; % [kg]
mass_sat_platform = 2.66; % [kg]
r_CoM_payload = 0.1; % [m]
r_CoM_sat_platform = -0.1; % [m] 
% center of mass of whole payload in the geometric reference frame
r_CoM_satellite = (r_CoM_payload*mass_payload + r_CoM_sat_platform*mass_sat_platform) / (mass_payload + mass_sat_platform); % [m]
% distance from satellite CoM to payload CoM
d_CS_CPL = r_CoM_payload - r_CoM_satellite; % [m]
% distance from satellite CoM to satellite platform CoM
d_CS_CSP = r_CoM_satellite - r_CoM_sat_platform; % [m]

% Intertia around satellite CoM
I_satellite = mass_payload*d_CS_CPL^2 + mass_sat_platform*d_CS_CSP^2; % [kg*m^2]

% Angular momentum provided by the reaction wheel
L_wheel = 0.001; % [kg*m^2/s] 
