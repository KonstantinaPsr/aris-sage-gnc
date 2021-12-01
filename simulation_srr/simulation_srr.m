% the x axis is parallel to long/easy axis of the satellite, the y and z
% axis are parallel to each corresponing short/hard axis of the satellite

% Await more detailed information from structures from weight distribution

unit = 3; % number of stacked cubesat units

% dimensions of satellite
height = unit*0.1; % points along x axis
length = 0.1; % points along y axis
width = 0.1; % points along z axis


volume = length*width*height; % [m^3]
% mass = unit*1.33; % [kg], 1.33kg is the max weight of a cubesat unit
% I_x = 1/12*mass*(width^2+length^2); % [kg*m^2], inertia around x-axis
% I_y = 1/12*mass*(width^2+height^2); % [kg*m^2], inertia around y-axis
% I_z = 1/12*mass*(length^2+height^2); % [kg*m^2], inertia around z-axis
% I = [I_x 0 0; 0 I_y 0; 0 0 I_z]; % [kg*m^2], inertia tensor
omega = [0; 0; 0]; % [1/s], rotational speed of satellite

%% Compute rotational speed and gravity acting on payload

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

% Angular momentum provided by the reaction wheel (provided from rocket
% lab)
L_wheel = 0.001; % [kg*m^2/s] 
% resulting rotational speed around the z-axis
omega_z = L_wheel/I_satellite; % [1/s]
disp(omega_z);
% velocity at the payload CoM
v_payload = omega_z*d_CS_CPL; % [m/s]
F_payload = mass_payload*v_payload^2/d_CS_CPL; % [N]
g_payload = F_payload/mass_payload; % [m/s^2]
disp(g_payload);



%% Compute needed Power
% Calculations based on the 

%% TODO
% Gravity to torque plot
% Power to torque plot
% Power to gravity plot
% Angular Momentum to gravity plot
% Actuator Comparison
