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
maxtime = 15;             % Upper simulation timer limit, s

dt = 0.001;                  % Simulation time segments, s=0.001
i = 2;
Kp = 2.3;                     %proportional gain    2.3
Ki = 2.5;                      %integral gain 2.5    
Kd = 0.001;                      %derivative gain 0.001
Kp2 = 2.1;     %2.1
Ki2 = 0.002 ;   %            0.002
Kd2 = 0;           %0
dV = 0;
dv = 0;
psid = 0; 
intd1=0;
intd2=0; 
derivative=0;
derivative2=0;
pdeltav=0;
deltav=0;
pdeltapsi = 0;
deltapsi = 0;
dvelocity = 0;
rho = 0;
resultant_v = 0;

%% Define the artificial potential field parameters
% Points of attraction
Xg = 20;
Yg = 15;
% Location of obstacles
nobs = 1;
xo = [5];
yo = [10];
% Size of obstacles (rho_0)
rho_0 = [5];
% Constants.
Ka = 0.01;  
Kr = [5];
m = 2.148;
traj = [];
vx = 0;
vy = 0;

% Store variables for later assessment
traj = [];

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
psid_mod(1,:) = zeros(1,1);
deltapsi_mod(1,:) = zeros(1,1);
deltav_mod(1,:) = zeros(1,1);
dvelocity_mod(1,:) = zeros(1,1);
rho_mod(1,:) = zeros(1,1);
resultant_v_mod(1,:) = zeros(1,1);

% -----------------------------------------------------------------------

%--------------------------------------------------------------------------

for simtime = dt:dt:maxtime 
    
    % INPUT FROM CONTROLLER
    V = [dV+dv dV+dv dV-dv dV-dv]; 
    % low = -12;
    % high = 12;
    % V = min(max(V, low), high);
   
    %% Potential Field Calculation
    % Attractive force (Fa)
    Fa = zeros(2,1);
    Fa(1,1) = -Ka*(x_mod(7)-Xg);
    Fa(2,1) = -Ka*(x_mod(8)-Yg);
    
    % Repulsive force (Fr)
    Fr = zeros(2,1);
    rho_r = sqrt((xo-x_mod(7))^2+(yo-x_mod(8))^2);
    disp(rho_r);
    if (rho_r <= 20)
         Fr(1,1) = Fr(1,1) - Kr*(rho_r-rho_0)*(x_mod(7)-xo)*(1/rho_r);
         Fr(2,1) = Fr(2,1) - Kr*(rho_r-rho_0)*(x_mod(8)-yo)*(1/rho_r);
    
% %     for i=1:length(xo)
% %          if rho_r <= 15
% %               %rho_r = sqrt((xo(i)-x)^2+(yo(i)-y)^2);
% %               Fr(1,1) = Fr(1,1) - Kr(i)*(rho_r-rho_0(i))*(x_mod(7)-xo(i))*(1/rho_r);
% %               Fr(2,1) = Fr(2,1) - Kr(i)*(rho_r-rho_0(i))*(x_mod(8)-yo(i))*(1/rho_r);
% %          end 
    end
    disp(Fr)
     % Total force (F)
    F = zeros(2,1);
    Fy = Fa(2,1) + sum(Fr(2,1));
    Fx = Fa(1,1) + sum(Fr(1,1));
    F = Fx + Fy;
    
    % Velocity and Position (vx, vy, x, y) through integrating the equations
    vx = vx + dt*(Fx/m);
    vy = vy + dt*(Fy/m);
    vr = sqrt(vx^2+vy^2); %resultant velocity vector 

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
    psid_mod(i,:) = psid;
    deltapsi_mod(i,:) = deltapsi;
    deltav_mod(i,:) = deltav;
    dvelocity_mod(i,:) = dvelocity;
    rho_mod(i,:) = rho;
    resultant_v_mod(i,:) = resultant_v;
   
% -------------------------------------------------------------------------
        i = i + 1;  % Output counter
%-----------------------------/*---------------------------------------------
% Controllers
     
    %SURGE controller
    %rho_r = sqrt((xo-x_mod(7))^2+(yo-x_mod(8))^2);
    rho = sqrt((Xg-x_mod(7))^2+(Yg-x_mod(8))^2);
    if rho <= 1
        dvelocity = 0;                             %desired surge velocity
    else
        dvelocity = vr;
        low = -1.5;
        high = 1.5;
        dvelocity = min(max(dvelocity, low), high);
    end
    %dsurge = 1;
    resultant_v = sqrt(x_mod(1)^2+x_mod(2)^2);
    deltav = dvelocity - resultant_v;              %error
    intd1 = intd1 + dt*deltav;              %integral term
    derivative = (deltav-pdeltav)/dt;   %derivative term
    dV = (Kp*deltav)+(Ki*intd1)+(Kd*derivative);   %controller
    pdeltav = deltav;                    %store old error value
    
    %YAW controller  
    %calculate desired heading angle geometrically
    psid = atan2((Yg-x_mod(8)),(Xg-x_mod(7)));
    %psid = 3;  %3.1398=179.9 0.785 ~ 45
    %calculate the difference between desired and actual heading angle
     deltapsi = psid - x_mod(12);  
     intd2 = intd2 + dt*deltapsi;
     derivative2 = (deltapsi-pdeltapsi)/dt;
     dv = (Kp2*deltapsi) + (Ki2*intd2) + (Kd2*derivative2);
     pdeltapsi = deltapsi;
           
% =========================================================================
% Establish new current variables
    x_mod = x_mod + xdot_mod*dt;
    I_mod = I_mod + Idot_mod*dt;
    Omega_mod = Omega_mod + Omegadot_mod.*dt;
    
    traj = [traj [x_mod(7);x_mod(8)]]; %plot the trajectory
    traj2 = [traj [
end


%% Outputs

figure(1);
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
ylabel('psi [degrees]') 
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

figure(2);
subplot(3,1,1)
plot(time, deltav_mod)
xlabel('time(s)')
ylabel('error(m/s)')
subplot(3,1,2)
plot(time, deltapsi_mod*180/pi)
xlabel('time(s)')
ylabel('error(degrees)')
subplot(3,1,3)
plot(time, resultant_v_mod)
xlabel('time(s)')
ylabel('resultant velocity m/s')

%% Display the results
% Plot the trajectory
figure(3);
% Now the attractive potential contour
xmin = -50; xmax = 50;
ymin = -50; ymax = 50;
[Y,X] = meshgrid(xmin:1:xmax,ymin:1:ymax);
Z = 1.0*(0.5*Ka*sqrt((X-Xg).^2+(Y-Yg).^2));
Zmax = max(max(Z));
Z = 10*Z/Zmax;
contour(Y,X,Z,50);hold on; 
plot(traj(2,1),traj(1,1),'bo','LineWidth',6);
plot(Yg,Xg,'bx','LineWidth',10);
xlabel('y(m)');
ylabel('x(m)');
axis('equal');
% Now the repulsive potential contour
........
% The obstacle contours
for kk = 1:nobs
    r = 0:0.01:rho_0(kk);    %0.01
    tht = 0:0.1:2*pi+0.1;    %0.1
    [THT,R] = meshgrid(tht,r);
    Z = 0.5*Kr(kk)*((sqrt((R.*cos(THT)).^2+(R.*sin(THT)).^2))-(rho_0(kk))).^2;
    Zmax = max(max(Z));
    Z = 10*Z/Zmax;
    contour(xo(kk)+R.*cos(THT),yo(kk)+R.*sin(THT),Z,50);
end
% Draw the obstacle boundaries
for jj = 1:nobs
    tht = 0:0.01:2*pi;
    xobs = xo(jj)+rho_0(jj)*cos(tht);
    yobs = yo(jj)+rho_0(jj)*sin(tht);
    plot(xobs,yobs,':r','LineWidth',2.0);
end
%% Finally, plot the trajectory
plot(traj(2,:),traj(1,:),'g','LineWidth',1.5);
hold off;
axis equal;
axis tight; 

% figure 
% p = 1.0*(0.5*Ka*sqrt((X-Xg).^2+(Y-Yg).^2));
% q= -1.0*(0.5*Kr*((sqrt((X-xo).^2+(Y-yo).^2))-rho_0)^2);
% mesh(p)
% figure
% mesh(q)



 