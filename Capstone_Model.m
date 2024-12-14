%Connor Willis Capstone Simulation
clc;clear;close all

%% Parameters

% Membrane Parameters
V0 = 0.01; % Resting volume of the membrane [m^3]
A_membrane = 0.01; % Area of the membrane [m^2]
Kpositive = 1000; % Membrane stiffness when Volume Increases [N/m^3]
Knegative = 1000; % Membrane stiffness when Volume Decreases [N/m^3]
C_membrane = 100; % Damping coefficient [N·s/m^3]

%Pump Parameters
P_pos = 20000; % Positive pump pressure [Pa]
P_neg = -10000; % Negative pump pressure [Pa]
Q_in = 0.001; % Inflow rate [m^3/s]
Q_out = 0.00066; % Outflow rate [m^3/s]
d_on = .4; % Duty Cycle



%% Simulation

% Simulation settings
sim_time = 10; % Simulation time [s]
dt = 0.001; % Time step [s]

% Run Simulink model
s= sim('Capstone_Sim', 'StopTime', num2str(sim_time), 'Solver', 'ode45', 'FixedStep', num2str(dt));

% Plot Membrane Volume
subplot(2,1,1);
plot(s.time, s.membraneVolume, 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Volume [m^3]');
title('Membrane Volume Over Time');
grid on;

% Plot Membrane Pressure
subplot(2,1,2);
plot(s.time, s.membranePressure, 'r', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Pressure [Pa]');
title('Membrane Pressure Over Time');
grid on;


