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
x_mod2 = zeros(12,1); % Current position and velocity variables
x_mod3 = zeros(12,1);
 
I_mod = zeros(1,4);     % Initialise electrical current vector
Omega_mod = zeros(1,4); % Intialise wheel rotational velocity vector
I_mod2 = zeros(1,4);     % Initialise electrical current vector
Omega_mod2 = zeros(1,4); % Intialise wheel rotational velocity vector
I_mod3 = zeros(1,4);     % Initialise electrical current vector
Omega_mod3 = zeros(1,4); % Intialise wheel rotational velocity vector

unmatched = zeros(3,1);

simtime = 0;                % Simulation timer, s
maxtime = 10;             % Upper simulation timer limit, s

dt = 0.001;                  % Simulation time segments, s=0.001
i = 2;
Kp = 2.3;                     %proportional gain    2.3
Ki = 2.5;                      %integral gain 2.5    
Kd = 0.001;                      %derivative gain 0.001
Kp2 = 2.1;     %2.1
Ki2 = 0.002 ;   %            0.002
Kd2 = 0;           %0
dV = [0 0 0];
dv = [0 0 0];
psid = [0 0 0]; 
intd1=[0 0 0];
intd2=[0 0 0]; 
derivative=[0 0 0];
derivative2=[0 0 0];
pdeltav= [0 0 0];
deltav= [0 0 0];
pdeltapsi = [0 0 0];
deltapsi = [0 0 0];
dvelocity = [0 0 0];
rho = [0 0 0];
resultant_v = [0 0 0];

%% Define the artificial potential field parameters
% number of rovers
nor = 3;
% Points of attraction
Xg = 20;  %10,1
Yg = 10;
% Location of obstacles
noo = 1;
xo = [10];
yo = [10];
%Radius of obstacles (rho_0)
rho_0 = [1];
% Constants.
Ka = 3; 
Kr = [2000 2000 3000]
Kdamp = 50;
m = 2.148;
vx = [0 0 0];
vy = [0 0 0];
vr = [0 0 0];
xt = [0 -2 1];
yt = [0 -1.5 2];

% Store variables for later assessment
traj = [];
traj2 = [];
traj3 = [];

% ----------------------------------------------------------------------
% Initialise output matrices
Vout(1,:) = zeros(1,4); 
V2out(1,:) = zeros(1,4);
V3out(1,:) = zeros(1,4);
time(1) = simtime;

% torquesout_mod(1,:) = zeros(1,4); 
xout_mod(1,:) = x_mod;
xout_mod2(1,:) = x_mod2;
xout_mod3(1,:) = x_mod3;
xdotout_mod(1,:) = zeros(12,1);
xdotout_mod2(1,:) = zeros(12,1);
xdotout_mod3(1,:) = zeros(12,1);
omegaout_mod(1,:) = zeros(1,4);
omegaout_mod2(1,:) = zeros(1,4);
omegaout_mod3(1,:) = zeros(1,4);
omegadotout_mod(1,:) = zeros(1,4);    
omegadotout_mod2(1,:) = zeros(1,4); 
omegadotout_mod3(1,:) = zeros(1,4); 
Iout_mod(1,:) = zeros(1,4);
Iout_mod2(1,:) = zeros(1,4);
Iout_mod3(1,:) = zeros(1,4);
Idotout_mod(1,:) = zeros(1,4);
Idotout_mod2(1,:) = zeros(1,4);
Idotout_mod3(1,:) = zeros(1,4);
psid_mod(1,:) = zeros(3,1);
deltapsi_mod(1,:) = zeros(3,1);
deltav_mod(1,:) = zeros(3,1);
dvelocity_mod(1,:) = zeros(3,1);
rho_mod(1,:) = rho;
rhor_mod(1,:) = zeros(3,1);
resultant_v_mod(1,:) = zeros(3,1);
xt_mod(1,:) = xt; % xt_mod(1,:) = zeros(1,1); 
yt_mod(1,:) = yt; % yt_mod(1,:) = zeros(1,1);
vx_mod(1,:) = vx;
%vt_mod(1,:) = zeros(1,1);
vy_mod(1,:) = vy;
Fa_mod(1,:) = zeros(2,1);
Fr_mod(1,:) = zeros(2,1);
Fy_mod(1,:) = zeros(2,1);
Fx_mod(1,:) = zeros(2,1);
Fa_mod2(1,:) = zeros(2,1);
Fr_mod2(1,:) = zeros(2,1);
Fy_mod2(1,:) = zeros(2,1);
Fx_mod2(1,:) = zeros(2,1);
Fa_mod3(1,:) = zeros(2,1);
Fr_mod3(1,:) = zeros(2,1);
Fy_mod3(1,:) = zeros(2,1);
Fx_mod3(1,:) = zeros(2,1)
vr_mod(1,:) = vr;

% INITIALISE POSITION IF NOT ZERO`
 x_mod(7) = 0;
 x_mod(8) = 0;
 x_mod2(7) = -2; %ensure same value in xt and yt for initial position!
 x_mod2(8) = -1.5;
 x_mod3(7) = 1;
 x_mod3(8) = 2;

%--------------------------------------------------------------------------
 
for simtime = 0:dt:maxtime 
    
    % INPUT FROM CONTROLLER
%     x = dV(1,1) + dv(1,1);
%     y = dV(1,1) - dv(1,1);
    V = [dV(1,1)+dv(1,1) dV(1,1)+dv(1,1) dV(1,1)-dv(1,1) dV(1,1)-dv(1,1)]; 
    V2 = [dV(1,2)+dv(1,2) dV(1,2)+dv(1,2) dV(1,2)-dv(1,2) dV(1,2)-dv(1,2)];
    V3 = [dV(1,3)+dv(1,3) dV(1,3)+dv(1,3) dV(1,3)-dv(1,3) dV(1,3)-dv(1,3)];
   
 %% Calculate the conservative forces
 
    [xt(1,1), yt(1,1), vx(1,1), vy(1,1), vr(1,1), Fa, Fr, Fd, Fx, Fy, rho_r(1,1)] = apf(xt(1,1), yt(1,1), vx(1,1), vy(1,1), Xg, Yg, xo, yo, noo, Ka, Kr, Kdamp, m, rho_0, dt);
    
    [xt(1,2), yt(1,2), vx(1,2), vy(1,2), vr(1,2), Fa2, Fr2, Fd2, Fx2, Fy2, rho_r(1,2)] = apffollower(xt(1,2), yt(1,2), vx(1,2), vy(1,2), Xg, Yg, xo, yo, noo, Ka, Kr, Kdamp, m, rho_0, dt, xt(1,1), yt(1,1));
    
    [xt(1,3), yt(1,3), vx(1,3), vy(1,3), vr(1,3), Fa3, Fr3, Fd3, Fx3, Fy3, rho_r(1,3)] = apf(xt(1,3), yt(1,3), vx(1,3), vy(1,3), Xg, Yg, xo, yo, noo, Ka, Kr, Kdamp, m, rho_0, dt);
% =========================================================================
% Run the rover model
    
    % MODEL    
    % Run the motor model
    [torques_mod,Idot_mod,Omegadot_mod] = Rover_Motor_Model_v1(V,I_mod,Omega_mod);
    [torques_mod2,Idot_mod2,Omegadot_mod2] = Rover_Motor_Model_v1(V2,I_mod2,Omega_mod2);
    [torques_mod3,Idot_mod3,Omegadot_mod3] = Rover_Motor_Model_v1(V3,I_mod3,Omega_mod3);
    
    torquesout_mod(i,:) = torques_mod;
    torquesout_mod2(i,:) = torques_mod2;
    torquesout_mod3(i,:) = torques_mod3;
     
    % Run model for dynamic/kinematic response of rover system
    [xdot_mod, x_mod] = Rover_Rigid_Body_System_v1(x_mod,torques_mod,unmatched);
    [xdot_mod2, x_mod2] = Rover_Rigid_Body_System_v1(x_mod2,torques_mod2,unmatched);
    [xdot_mod3, x_mod3] = Rover_Rigid_Body_System_v1(x_mod3,torques_mod3,unmatched);
 
% =========================================================================
% Establish outputs

    % MODEL Output Matrices
    xout_mod(i,:) = x_mod;
    xout_mod2(i,:) = x_mod2;
    xout_mod3(i,:) = x_mod3;
    xdotout_mod(i,:) = xdot_mod;
    xdotout_mod2(i,:) = xdot_mod2;
    xdotout_mod3(i,:) = xdot_mod3;
    omegaout_mod(i,:) = Omega_mod;
    omegaout_mod2(i,:) = Omega_mod2;
    omegaout_mod3(i,:) = Omega_mod3;
    omegadotout_mod(i,:) = Omegadot_mod;    
    omegadotout_mod2(i,:) = Omegadot_mod2;
    omegadotout_mod3(i,:) = Omegadot_mod3; 
    Iout_mod(i,:) = I_mod;
    Iout_mod2(i,:) = I_mod2;
    Iout_mod3(i,:) = I_mod3;
    Idotout_mod(i,:) = Idot_mod;
    Idotout_mod2(i,:) = Idot_mod2;
    Idotout_mod3(i,:) = Idot_mod3;
    Vout(i,:) = V;
    Vout2(i,:) = V2;
    Vout3(i,:) = V3;
    time(i) = simtime;
    psid_mod(i,:) = psid;
    deltapsi_mod(i,:) = deltapsi;
    deltav_mod(i,:) = deltav;
    dvelocity_mod(i,:) = dvelocity;
    rho_mod(i,:) = rho;
    resultant_v_mod(i,:) = resultant_v;
    xt_mod(i,:) = xt;
    yt_mod(i,:) = yt;
    vx_mod(i,:) = vx;
    vy_mod(i,:) = vy;
    Fa_mod(i,:) = Fa;
    Fr_mod(i,:) = Fr;
    Fx_mod(i,:) = Fx;
    Fy_mod(i,:) = Fy;
    vr_mod(i,:) = vr;
    rhor_mod(i,:) = rho_r;
    Fa2_mod(i,:) = Fa2;
    Fr2_mod(i,:) = Fr2;
    Fx2_mod(i,:) = Fx2;
    Fy2_mod(i,:) = Fy2;
    Fa3_mod(i,:) = Fa3;
    Fr3_mod(i,:) = Fr3;
    Fx3_mod(i,:) = Fx3;
    Fy3_mod(i,:) = Fy3;


% -------------------------------------------------------------------------
    i = i + 1;  % Output counter
%-----------------------------/*---------------------------------------------
% Controllers
     
    %VELOCITY controller - resultant
    rho(1,1) = sqrt((Xg-x_mod(7))^2+(Yg-x_mod(8))^2);
    if rho(1,1) <= 0.2
        dvelocity(1,1) = 0;                             %desired surge velocity
    else
        dvelocity(1,1) = vr(1,1);
%         low = -1.0;
%         high = 1.0;         dvelocity = min(max(dvelocity, low), high);
    end
    resultant_v(1,1) = sqrt(x_mod(1)^2+x_mod(2)^2);
    deltav(1,1) = dvelocity(1,1) - resultant_v(1,1);              %error
    intd1(1,1) = intd1(1,1) + dt*deltav(1,1);              %integral term
    derivative(1,1) = (deltav(1,1)-pdeltav(1,1))/dt;   %derivative term
    dV(1,1) = (Kp*deltav(1,1))+(Ki*intd1(1,1))+(Kd*derivative(1,1));   %controller
    pdeltav(1,1) = deltav(1,1);                    %store old error value
    
    %VELOCITY controller - rover 2
    rho(1,2) = sqrt((Xg-x_mod2(7))^2+(Yg-x_mod2(8))^2);
    if rho(1,2) <= 0.2
        dvelocity(1,2) = 0;                             %desired surge velocity
    else
        dvelocity(1,2) = vr(1,2);
    end
    resultant_v(1,2) = sqrt(x_mod2(1)^2+x_mod2(2)^2);
    deltav(1,2) = dvelocity(1,2) - resultant_v(1,2);              %error
    intd1(1,2) = intd1(1,2) + dt*deltav(1,2);              %integral term
    derivative(1,2) = (deltav(1,2)-pdeltav(1,2))/dt;   %derivative term
    dV(1,2) = (Kp*deltav(1,2))+(Ki*intd1(1,2))+(Kd*derivative(1,2));   %controller
    pdeltav(1,2) = deltav(1,2);                    %store old error value
    
    %VELOCITY controller - rover 3
    rho(1,3) = sqrt((Xg-x_mod3(7))^2+(Yg-x_mod3(8))^2);
    if rho(1,3) <= 0.2
        dvelocity(1,3) = 0;                             %desired surge velocity
    else
        dvelocity(1,3) = vr(1,3);
    end
    resultant_v(1,3) = sqrt(x_mod3(1)^2+x_mod3(2)^2);
    deltav(1,3) = dvelocity(1,3) - resultant_v(1,3);              %error
    intd1(1,3) = intd1(1,3) + dt*deltav(1,3);              %integral term
    derivative(1,3) = (deltav(1,3)-pdeltav(1,3))/dt;   %derivative term
    dV(1,3) = (Kp*deltav(1,3))+(Ki*intd1(1,3))+(Kd*derivative(1,3));   %controller
    pdeltav(1,3) = deltav(1,3);                    %store old error value
    
    %YAW controller  
    %calculate desired heading angle geometrically
    psid(1,1) = atan2((yt(1,1)-x_mod(8)),(xt(1,1)-x_mod(7)));
    %psid = -3;  %3.1398=179.9 0.785 ~ 45
    %calculate the difference between desired and actual heading angle
    deltapsi(1,1) = psid(1,1) - x_mod(12);  
    intd2(1,1) = intd2(1,1) + dt*deltapsi(1,1);
    derivative2(1,1) = (deltapsi(1,1)-pdeltapsi(1,1))/dt;
    dv(1,1) = (Kp2*deltapsi(1,1)) + (Ki2*intd2(1,1)) + (Kd2*derivative2(1,1));
    pdeltapsi(1,1) = deltapsi(1,1);
    
    %YAW controller r2
    %calculate desired heading angle geometrically
    psid(1,2) = atan2((yt(1,2)-x_mod2(8)),(xt(1,2)-x_mod2(7)));
    %calculate the difference between desired and actual heading angle
    deltapsi(1,2) = psid(1,2) - x_mod2(12);  
    intd2(1,2) = intd2(1,2) + dt*deltapsi(1,2);
    derivative2(1,2) = (deltapsi(1,2)-pdeltapsi(1,2))/dt;
    dv(1,2) = (Kp2*deltapsi(1,2)) + (Ki2*intd2(1,2)) + (Kd2*derivative2(1,2));
    pdeltapsi(1,2) = deltapsi(1,2);
    
    %YAW controller r3
    %calculate desired heading angle geometrically
    psid(1,3) = atan2((yt(1,3)-x_mod3(8)),(xt(1,3)-x_mod3(7)));
    %calculate the difference between desired and actual heading angle
    deltapsi(1,3) = psid(1,3) - x_mod3(12);  
    intd2(1,3) = intd2(1,3) + dt*deltapsi(1,3);
    derivative2(1,3) = (deltapsi(1,3)-pdeltapsi(1,3))/dt;
    dv(1,3) = (Kp2*deltapsi(1,3)) + (Ki2*intd2(1,3)) + (Kd2*derivative2(1,3));
    pdeltapsi(1,3) = deltapsi(1,3);
           
% =========================================================================
% Establish new current variables
    x_mod = x_mod + xdot_mod*dt;
    x_mod2 = x_mod2 + xdot_mod2*dt;
    x_mod3 = x_mod3 + xdot_mod3*dt;
    I_mod = I_mod + Idot_mod*dt;
    I_mod2 = I_mod2 + Idot_mod2*dt;
    I_mod3 = I_mod3 + Idot_mod3*dt;
    Omega_mod = Omega_mod + Omegadot_mod.*dt;
    Omega_mod2 = Omega_mod2 + Omegadot_mod2.*dt;
    Omega_mod3 = Omega_mod3 + Omegadot_mod3.*dt;
    
    traj = [traj [x_mod(7);x_mod(8)]]; %plot the trajectory
    traj2 = [traj2 [x_mod2(7);x_mod2(8)]]; %plot the trajectory
    traj3 = [traj3 [x_mod3(7);x_mod3(8)]]; %plot the trajectory


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

figure(3)
subplot(5,1,1)
plot(time, Fa_mod)
xlabel('time (s)')
ylabel('Magnitude Fa')

subplot(5,1,2)
plot(time, Fr_mod)
xlabel('Time (s)')
ylabel('Magnitude Fr')
subplot(5,1,3)
plot(time, xout_mod(:,7))
hold on
plot(time, xout_mod(:,8))
xlabel('time (s)')
ylabel('co-ordinate (m)')
legend('X', 'Y')
subplot(5,1,4)
plot(time, Fx_mod)
xlabel('time (s)')
ylabel('Magnitude Fx')
subplot(5,1,5)
plot(time, Fy_mod)
xlabel('time (s)')
ylabel('Magnitude Fy')

%% Display the results
% Plot the trajectory
figure(4);
% Now the attractive potential contour
xmin = -5; xmax = 20;
ymin = -5; ymax = 15;
[Y,X] = meshgrid(xmin:1:xmax,ymin:1:ymax);
Z = 1.0*(0.5*Ka*sqrt((X-Xg).^2+(Y-Yg).^2));
Zmax = max(max(Z));
Z = 10*Z/Zmax;
contour(Y,X,Z,50);hold on; 
plot(traj(2,1),traj(1,1),'bo','LineWidth',6);
plot(traj2(2,1),traj2(1,1),'bo','LineWidth',6);
plot(traj3(2,1),traj3(1,1),'bo','LineWidth',6);
plot(Yg,Xg,'bx','LineWidth',10);
xlabel('y(m)');
ylabel('x(m)');
axis('equal');
% Now the repulsive potential contour
........
% The obstacle contours
for kk = 1:noo
    r = 0:0.01:rho_0(kk);
    tht = 0:0.1:2*pi+0.1;
    [THT,R] = meshgrid(tht,r);
    Z = 0.5*Kr(kk)*((sqrt((R.*cos(THT)).^2+(R.*sin(THT)).^2))-(rho_0(kk))).^2;
    Zmax = max(max(Z));
    Z = 10*Z/Zmax;
    contour(yo(kk)+R.*cos(THT),xo(kk)+R.*sin(THT),Z,50);
end
% Draw the obstacle boundaries
for jj = 1:noo
    tht = 0:0.01:2*pi;
    xobs = yo(jj)+rho_0(jj)*cos(tht);
    yobs = xo(jj)+rho_0(jj)*sin(tht);
    plot(xobs,yobs,':r','LineWidth',2.0);
end
%% Finally, plot the trajectory
plot(traj(2,:),traj(1,:),'g','LineWidth',1.5);
hold on;
plot(traj2(2,:),traj2(1,:),'r','LineWidth',1.5);
hold on;
plot(traj3(2,:),traj3(1,:),'b','LineWidth',1.5)
hold off;
axis equal;
axis tight; 

% Plot the theoretical trajectory
figure
subplot(2,1,1)
plot(yt_mod(:,1), xt_mod(:,1))
xlabel('yt (m)')
ylabel('xt (m)')
title('rover 1 theoretical traj')
subplot(2,1,2)
plot(yt_mod(:,2), xt_mod(:,2))
xlabel('yt (m)')
ylabel('xt (m)')
title('rover 2 theoretical traj')

figure
plot(time, vx_mod(:,1))
hold on
plot(time, vy_mod(:,1))
xlabel('Time (s)')
ylabel('Theoretical Velocity (m/s)')
legend('vx', 'vy')
% 
% figure
% plot(xt_mod(:,1))
% legend('xt')
% 
% figure
% plot(yt_mod(:,1))
% legend('yt')

% figure 
% p = 0.5*Ka*((X-Xg).^2+(Y-Yg).^2);
%  rho_r = sqrt((X-xo).^2+(Y-yo).^2);
%  q= 0.5*Kr*((1./rho_r-1./rho_0).^2);  %CHANGE THIS BACK TO THE RIGHT FIELD AS IN FR CALC
% mesh(X,Y,p)
%  figure
%  mesh(X,Y,q)
%  figure
%  mesh(X,Y,p+q)


