% Rover_Open_Loop
%
% S. Shilliday
% Created: 2020/07/06
% Last Edited: 
%
% Designed for Rover_Rigid_Body_Model.m and Rover_Motor_Model.m,
%
% Simulation of a 4WD rover using open-loop control
%
% Inputs: Voltage values
% Outputs: Plots of actual current position
%
% Change log:-  
%
%%
clear; close all; clc;

% ----------------------------------------------------------------------
% Initialise variables
x_mod = zeros(12,1); % Current position and velocity variables

I_mod = zeros(1,4);     % Initialise electrical current vector
Omega_mod = zeros(1,4); % Intialise wheel rotational velocity vector

unmatched = zeros(3,1);

simtime = 0;                % Simulation timer, s
maxtime = 10;             % Upper simulation timer limit, s
dt = 0.001;                  % Simulation time segments, s
i = 2;

% ----------------------------------------------------------------------
% Initialise output matrices
Vout(1,:) = zeros(1,4);
time(1) = simtime;

torquesout_mod(1,:) = zeros(1,4);
xout_mod(1,:) = x_mod;
xdotout_mod(1,:) = zeros(12,1);
omegaout_mod(1,:) = zeros(1,4);
omegadotout_mod(1,:) = zeros(1,4);
Iout_mod(1,:) = zeros(1,4);
Idotout_mod(1,:) = zeros(1,4);

% -----------------------------------------------------------------------
%%

for simtime = dt:dt:maxtime
    
V = [12 12 12 12];  % Input voltage values

% =========================================================================
% Run the rover model
    
    % MODEL    
    % Run the motor model
    [torques_mod,Idot_mod,Omegadot_mod] = Rover_Motor_Model_v1(V,I_mod,Omega_mod);
    
    torquesout_mod(i,:) = torques_mod;
    
    % Run model for dynamic/kinematic response of rover system
    [xdot_mod, x_mod] = Rover_Rigid_Body_System_v1(x_mod,torques_mod,unmatched);
 
% =========================================================================
% Establish outputs

    % MODEL Output Matrices
    xout_mod(i,:) = x_mod;
    xdotout_mod(i,:) = xdot_mod;
    omegaout_mod(i,:) = Omega_mod;
    omegadotout_mod(i,:) = Omegadot_mod;
    Iout_mod(i,:) = I_mod;
    Idotout_mod(i,:) = Idot_mod;
    Vout(i,:) = V;
    time(i) = simtime;
    
% -------------------------------------------------------------------------
        i = i + 1;  % Output counter

% =========================================================================
% Establish new current variables
    x_mod = x_mod + xdot_mod*dt;
    I_mod = I_mod + Idot_mod*dt;
    Omega_mod = Omega_mod + Omegadot_mod.*dt;

end


%% Outputs

subplot(4,2,1)
plot(time,xout_mod(:,1))
xlabel('time [s]')
ylabel('surge [m/s]')
subplot(4,2,2)
plot(time,xout_mod(:,2))
xlabel('time [s]')
ylabel('sway [m/s]')
subplot(4,2,3)
plot(time,xout_mod(:,6)*180/pi)
xlabel('time [s]')
ylabel('yaw rate [m/s]')
subplot(4,2,4)
plot(time,xout_mod(:,12)*180/pi)
xlabel('time [s]')
ylabel('psi [m/s]')
subplot(4,4,9)
plot(time,Vout(:,1))
xlabel('time [s]')
ylabel('V1 [V]')
subplot(4,4,10)
plot(time,Vout(:,2))
xlabel('time [s]')
ylabel('V2 [V]')
subplot(4,4,13)
plot(time,Vout(:,3))
xlabel('time [s]')
ylabel('V3 [V]')
subplot(4,4,14)
plot(time,Vout(:,4))
xlabel('time [s]')
ylabel('V4 [V]')
subplot(2,2,4)
plot(xout_mod(:,8),xout_mod(:,7))
xlabel('y_pos [m]')
ylabel('x_pos [m]')
