%Connor WIllis Capstone Project Simulation

%%
% This is version two of the simulation. This simulation assumes a constant
% flow from a pump towards a membrane then a fluid reservior on the opposite 
% side of a stopcock. This sim accounts for pressure drops using the
% Darcy-Weisbach Equation. It also includes a pressure drop due to the
% membrane and stopcock. This model also introduces fluid properties and 
% tube parameters.

clc; clear; close all;

%% Fluid Properties
rho = 1000; % Density of water [kg/m^3]
mu = 0.001; % Dynamic viscosity [Pa·s]

%% Tubing Parameters
D_tube = 0.00635; % Tube diameter (1/4-inch) [m]
L_membrane = 0.9144; % Length to membrane [m]
L_stopcock = 0.9144; % Length to stopcock [m]
A_tube=pi*(D_tube/2)^2; % Cross-sectional area [m^2]

%% Membrane Properties
A_membrane = 0.01; % Membrane surface area [m^2]
K_membrane = 1000; % Membrane stiffness [N/m^3]
C_membrane = 100; % Membrane damping coefficient [N·s/m^3]
V0 = 0.0005; % Initial membrane volume [m^3]

%% Pump Parameters
Q_pump_max = 0.000133; % Maximum pump flow rate [m^3/s]
bpm = 80; % Beats per minute
duty_cycle = 0.4; % Fraction of time pump is active

%% Reservoir Pressure
P_reservoir = 0; % Pressure at reservoir [Pa]

%% Run Simulink Model
sim_time = 10; % Simulation duration [s]
set_param('Capstone_SimV2_Fluid_Tube_Added', 'StopTime', num2str(sim_time));
s = sim('Capstone_SimV2_Fluid_Tube_Added.slx');

%% Plot Results
figure;
% Membrane Volume
subplot(2,1,1);
plot(s.t, s.V_membrane, 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Volume [m^3]');
title('Membrane Volume Over Time');
grid on;

% Membrane Pressure
subplot(2,1,2);
plot(s.t, s.P_membrane, 'r', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Pressure [Pa]');
title('Membrane Pressure Over Time');
grid on;
