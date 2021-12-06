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
mass_payload = 1.33; % based on the maximum weight for a cubesat unit [kg]
mass_sat_platform = 2.66; % 2 cubesat units with the maximum weight [kg]
%r_CoM_payload = 0.05:0.005:0.14; % [m]
%r_CoM_sat_platform = -0.05:-0.005:-0.14; % [m] 
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
% L_wheel = 0.01; % [kg*m^2/s] 
% resulting rotational speed around the z-axis
% omega_z = L_wheel/I_satellite; % [1/s]
% disp(omega_z);
% velocity at the payload CoM
% v_payload = omega_z*d_CS_CPL; % [m/s]
% F_payload = mass_payload*v_payload^2/d_CS_CPL; % [N]
% g_payload = F_payload/mass_payload; % [m/s^2]
% disp(g_payload);

%% Effect of the angular momentum of the actuator on the rotational velocity and the artificial gravity
% The minimum and maximum angualr momentum values are based on the reaction
% wheel developed by rocketlab which make sense for our case and still fit
% inside our 3U CubeSat
% The lowest value is from the 3mNms RW-0.003 (33.5mm x 33.5mm x 17mm)
% The largest value is from the 30mNms RW-0.03 (50mm x 50 mm x 40 mm)

mass = 1.33*3; % weight of the satellite is based on the maximum weight typical for a satellite [kg]
L_wheel_range = 0.003:0.0001:0.03; % range for the angular momentum [Nms]
omega_z_range = L_wheel_range/I_satellite; % angular velocity around main axis [1/s]
% velocity at the payload CoM
v_payload = omega_z_range*d_CS_CPL; % [m/s]
% centripetal force at CoM of payload
F_payload = mass_payload*v_payload.^2/d_CS_CPL; % [N]
% aritficial gravity acting at CoM of payload
g_payload = F_payload/mass_payload; % [m/s^2]
%disp(g_payload);

figure(1);
t = tiledlayout(1,1);
ax1 = axes(t);
plot(ax1, omega_z_range, g_payload);
plot_title = title({'Artificial gravity for a fixed satellite weight of 4kg';'';''});
plot_title.FontSize = 14;
xlabel('Angular velocity around rotation axis [1/s]','FontSize',14);
ylabel('Acceleration at CoM of payload [m/s^2]','FontSize',14);
ax2 = axes(t);
plot(ax2, L_wheel_range, g_payload);
xlabel('Angular momentum provided by reaction wheel [Nms]','FontSize',14);
ax2.XAxisLocation = 'top';
x0=10;
y0=10;
width=825;
height=600;
set(gcf,'position',[x0,y0,width,height])

% t = tiledlayout(1,1);
% ax1 = axes(t);
% plot(ax1,x1,y1,'-r')
% ax1.XColor = 'r';
% ax1.YColor = 'r';
% ax2 = axes(t);
% plot(ax2,x2,y2,'-k')
% ax2.XAxisLocation = 'top';
% ax2.YAxisLocation = 'right';
% ax2.Color = 'none';
% ax1.Box = 'off';
% ax2.Box = 'off';

%% Weight relation between satellite platform and payload
% the geometrical reference frame is placed at the exact center of the cubesat
mass_payload = 0.5:0.025:2; % weight of payload [kg]
mass_sat_platform = 2; % weight of satellite platform [kg]
weight_ratio = mass_payload/mass_sat_platform;
%r_CoM_payload = 0.05:0.005:0.14; % [m]
%r_CoM_sat_platform = -0.05:-0.005:-0.14; % [m] 
r_CoM_payload = 0.1; % [m]
r_CoM_sat_platform = -0.1; % [m] 
% center of mass of whole payload in the geometric reference frame
r_CoM_satellite = (r_CoM_payload.*mass_payload + r_CoM_sat_platform.*mass_sat_platform) ./ (mass_payload + mass_sat_platform); % [m]
%disp(r_CoM_satellite);
% distance from satellite CoM to payload CoM
d_CS_CPL = r_CoM_payload - r_CoM_satellite; % [m]
% distance from satellite CoM to satellite platform CoM
d_CS_CSP = r_CoM_satellite - r_CoM_sat_platform; % [m]

% Intertia around satellite CoM
I_satellite = mass_payload.*d_CS_CPL.^2 + mass_sat_platform.*d_CS_CSP.^2; % [kg*m^2]

L_wheel_range = 0.01; % range for the angular momentum [Nms]
omega_z_range = L_wheel_range./I_satellite; % angular velocity around main axis [1/s]
% velocity at the payload CoM
v_payload = omega_z_range.*d_CS_CPL; % [m/s]
% centripetal force at CoM of payload
F_payload = mass_payload.*v_payload.^2./d_CS_CPL; % [N]
% aritficial gravity acting at CoM of payload
g_payload = F_payload./mass_payload; % [m/s^2]

figure(2);
t = tiledlayout(1,1);
ax1 = axes(t);
plot(ax1, mass_payload, g_payload, '-k');
ax1.XColor = 'k';
ax1.YColor = 'k';
plot_title = title({'Artificial gravity for a satellite platform with a weight of 2kg';'';''});
plot_title.FontSize = 14;
xlabel('Payload mass [kg]','FontSize',14);
y = ylabel('Acceleration at CoM of payload [m/s^2]','FontSize',14);
ax2 = axes(t);
plot(ax2, weight_ratio, omega_z_range, 'color', [0 0.4470 0.7410]);
ax2.XColor = [0 0.4470 0.7410];
ax2.YColor = [0 0.4470 0.7410];
xlabel('m_{payload}/m_{platform}','FontSize',14);
xlim([0.25, 1]);
ylabel('Angular velocity around rotation axis [1/s]','FontSize',14)
ax2.XAxisLocation = 'top';
ax2.YAxisLocation = 'right';
ax2.Color = 'none';
ax1.Box = 'off';
ax2.Box = 'off';
x0=10;
y0=10;
width=825;
height=600;
set(gcf,'position',[x0,y0,width,height])

% figure(3);
% plot(mass_payload, r_CoM_satellite);
% xlabel('Payload mass [kg]');
% ylabel('Position of satellite CoM [m]');
% 
% figure(4);
% plot(mass_payload, I_satellite);
% xlabel('Payload mass [kg]');
% ylabel('Intertia of satellite [kg*m^2]');
% 
% figure(5);
% plot(mass_payload, omega_z_range);
% xlabel('Payload mass [kg]');
% ylabel('Angular velocity [1/s]');
% 
% figure(6);
% plot(mass_payload, g_payload);
% xlabel('Payload mass [kg]');
% ylabel('Acceleration [m/s^2]');
% 
% disp('-----------------------------------------------------------------')
% disp(g_payload)
% disp(omega_z_range)
%% Compute needed Power
% The values are based on the three reaction wheel types which would fit
% into our satellite
L_wheels = [1 0.01 0.02]; % Angula momentum [Nms]
P_wheels = [0 1.05 1.8]; % Power consumption [W]