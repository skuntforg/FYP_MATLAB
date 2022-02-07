% Rover_Motor_Model_v1.00
%
% S. Shilliday
% Created: 04/10/2018
% Last Edited: 08/10/2018
%
% This function script is designed to be used alongside 
% Rover_Rigid_Body_Model.m and Rover_4WD_State_Feedback_Controller.m
%
% Modelling the behaviour of the actuators in a Lynxmotion 4WD3 Rover for 
% given matrices of voltages, currents and rotational velocities
%
% Inputs: Matrix of Voltages, Currents, Rotational Velocities 
% Outputs: Torques generated, rate of change of current and rotational
% velocity
%
% Change log:-  **v1.00**   2020/04/28  -  Organised and standardised naming conventions       
%
% *************************************************************************

function [torques,idot,Omegadot] = Rover_Motor_Model_v1(V,I,Omega)

% Initialise relevant parameters
b = 0.008;          % Viscous torque [N m]
Jm = 0.005;         % Moment of inertia of motor [kg m^2]
Kt = 0.35;          % Torque constant [N m A^-1]
Ke = 0.35;          % EMF constant [V rad^-1 s^-1]
L = 0.1;            % Inductance of circuit [H]
R = 4;              % Resistance of circuit [ohms]
alpha = -0.133;     % Gradient for efficiency curve [A^-1]
gamma = 0.6;        % Offset for efficiency curve
zeta = 0.002;       % Base friction on wheel

idot = zeros(1,4);      % Initialise rate of change of current matrix
Omegadot = zeros(1,4);  % Initialise rate of change of rotational velocity matrix
eta = zeros(1,4);       % Initialise efficiency matrix
torques = zeros(1,4);   % Initialise torque matrix
 
for j = 1:4
   
    idot(1,j) = (V(1,j) - (R*I(1,j)) - (Ke*Omega(1,j)))/L;
    Omegadot(1,j) = ((Kt*I(1,j)) - (b*Omega(1,j)) - (zeta*Omega(1,j)))/Jm;
    
    eta(1,j) = (alpha*I(1,j)) + gamma;
    torques(1,j) = Kt * I(1,j) * eta(1,j);
    
end
