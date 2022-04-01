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
maxtime = 40;             % Upper simulation timer limit, s

dt = 0.01;                  % Simulation time segments, s=0.001
i = 2;
Kp = 2.3;                     %proportional gain    2.3
Ki = 3.5;                      %integral gain 2.5    
Kd = 0.001;                      %derivative gain 0.001
Kp2 = 2.1;     %2.1
Ki2 = 0.05 ;   %            0.002
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
q = 0;
rho_r = 0;
%% Define the artificial potential field parameters
% Points of attraction
Xg = 10;  %10,1
Yg = 10;
% Location of obstacles
nobs = 3;
% nobs = 5;
% xo = [10 20 30 40 50];
% yo = [1 1 1 1 1];
% xo = [3 5.5 8];
% yo = [-1 0 -2];
xo = [4.1 4.1 7]
yo = [4 5.8 8.5]
%Radius of obstacles (rho_0)
% rho_0 = [7 5 8 4 5];
% rho_0 = [2 0.5 2];
rho_0 = [1.5 1.5 0.9]
% Constants.
Ka = 2; 
Kr = [10000 4000 1200]
Kdamp = 50;
m = 2.148;
s = 0.1;
vx = 0;
vy = 0;
vr = 0;
xt = 0;
yt = 0;

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
rhor_mod(1,:) = zeros(1,1);
resultant_v_mod(1,:) = zeros(1,1);
xt_mod(1,:) = zeros(1,1);
yt_mod(1,:) = zeros(1,1);
vx_mod(1,:) = zeros(1,1);
vt_mod(1,:) = zeros(1,1);
vy_mod(1,:) = zeros(1,1);
Fa_mod(1,:) = zeros(2,1);
Fr_mod(1,:) = zeros(2,1);
Fy_mod(1,:) = zeros(2,1);
Fx_mod(1,:) = zeros(2,1);
vr_mod(1,:) = zeros(1,1);

% -----------------------------------------------------------------------

%--------------------------------------------------------------------------
Fr = zeros(2,1);
for simtime = 0:dt:maxtime 
    
    % INPUT FROM CONTROLLER
    V = [dV+dv dV+dv dV-dv dV-dv]; 
    %V = [dV dV dV dV];
    %V = [5.5+dv 5.5+dv 5.5-dv 5.5-dv];
    % V = [12 12 12 12];
   
 %% Calculate the conservative forces
    % Attractive force (Fa)
    Fa = zeros(2,1);
    Fa(1,1) = -Ka*(xt-Xg);
    Fa(2,1) = -Ka*(yt-Yg);
    fprintf('Fa\n')
    disp(Fa)
    % Repulsive force (Fr)
    Fr = zeros(2,1);
    for j=1:nobs
        
         rho_r = sqrt((xo(j)-xt)^2+(yo(j)-yt)^2);
        
         if (rho_r <= rho_0(j))
              %rho_r = sqrt((xo(i)-x)^2+(yo(i)-y)^2);
              Fr(1,1) = Fr(1,1) - Kr(j)*(rho_r-rho_0(j))*(xt-xo(j))*(1/rho_r);
              Fr(2,1) = Fr(2,1) - Kr(j)*(rho_r-rho_0(j))*(yt-yo(j))*(1/rho_r);
        end 
    end 
    fprintf('Fr\n')
    disp(Fr)
    % Damping force (Fd)
    Fd = zeros(2,1);
    Fd(1,1) = -Kdamp*vx;
    Fd(2,1) = -Kdamp*vy;
    fprintf('Fd\n')
    disp(Fd)
    % Added Force
    Fadd = zeros(2,1);
    rho_g = sqrt((xt-Xg)^2+(yt-Yg)^2);
    if vx < 0.0001 && rho_g > 0.2
         Fadd(1,1) = 3*0.3*(xt-Xg);
    end 
    if vy < 0.0001 && rho_g > 0.2
         Fadd(2,1) = 3*0.3*(yt-Yg);
    end
    % Total force (F)
    Fy = Fa(2,1) + sum(Fr(2,1)) + Fd(2,1) + Fadd(2,1);
    Fx = Fa(1,1) + sum(Fr(1,1)) + Fd(1,1) + Fadd(1,1);
    F(1,1) = Fx;
    F(2,1) = Fy;
    fprintf('Fx\n')
    disp(Fx)
    fprintf('Fy\n')
    disp(Fy)
    % Velocity and Position (vx, vy, x, y) through integrating the equations
    rho_g = sqrt((xt-Xg)^2+(yt-Yg)^2); %distance to goal
    vx = vx + dt*(Fx/m);
%     if vx < 0.001 % && rho_g > 0.2
%         vx = vx + 0.05
%     end
    fprintf('vx\n')
    disp(vx)
    vy = vy + dt*(Fy/m);
%     if vy < 0.001 % && rho_g > 0.2
%         vy = vy + 0.05
%     end
    fprintf('vy\n')
    disp(vy)
    xt = xt + dt*vx;
    fprintf('x\n')
    disp(xt)
    yt = yt + dt*vy;
    fprintf('y\n')
    disp(yt)
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
% -------------------------------------------------------------------------
    i = i + 1;  % Output counter
%-----------------------------/*---------------------------------------------
% Controllers
     
    %VELOCITY controller - resultant
  
    rho = sqrt((Xg-x_mod(7))^2+(Yg-x_mod(8))^2);
    if rho <= 0.2
        dvelocity = 0;                             %des ired surge velocity
    else
        dvelocity = vr;
%         if vx && vy < 0
%             dvelocity = -vr;
       %end
%         low = -1.4;
%         high = 1.4;
%         dvelocity = min(max(dvelocity, low), high);
    end
    dvelocity = vr;
    resultant_v = sqrt(x_mod(1)^2+x_mod(2)^2);
    deltav = dvelocity - resultant_v;              %error
    intd1 = intd1 + dt*deltav;              %integral term
    derivative = (deltav-pdeltav)/dt;   %derivative term
    dV = (Kp*deltav)+(Ki*intd1)+(Kd*derivative);   %controller
    pdeltav = deltav;                    %store old error value
    
    %SURGE controller - for testing 
%     dsurge = 1;
%     deltav = dsurge - x_mod(1);              %error
%     intd1 = intd1 + dt*deltav;              %integral term
%     derivative = (deltav-pdeltav)/dt;   %derivative term
%     dV = (Kp*deltav)+(Ki*intd1)+(Kd*derivative);   %controller
%     pdeltav = deltav;                    %store old error value
    
    
    %YAW controller  
    %calculate desired heading angle geometrically
    psid = atan2((yt-x_mod(8)),(xt-x_mod(7)));
    %psid = 3.07;  %3.1398=179.9 0.785 ~ 45
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
    traj2 = [traj [xt;yt]];
    
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
set(gca,'FontSize',14)


figure(2);
% subplot(3,1,1)
plot(time, deltav_mod, 'LineWidth', 2.5)
xlabel('time(s)', 'FontSize', 16)
ylabel('error(m/s)', 'FontSize', 16)
set(gca,'FontSize',14)


figure(6)
plot(time, deltapsi_mod*180/pi, 'LineWidth', 2.5)
 xlabel('time(s)', 'FontSize', 16)
 ylabel('error(degrees)', 'FontSize', 16)
set(gca,'FontSize',14)


figure(3)
plot(time, resultant_v_mod, 'LineWidth', 2.5)
xlabel('time [s]', 'FontSize', 16)
ylabel('resultant velocity [m/s]', 'FontSize', 16)
set(gca,'FontSize',14)

figure(4)
plot(time,Vout(:,1), 'LineWidth', 2.5)
xlabel('time [s]', 'FontSize', 16)
hold on
plot(time,Vout(:,2), 'LineWidth', 2.5)
hold on
plot(time,Vout(:,3), 'LineWidth', 2.5)
hold on
plot(time,Vout(:,4), 'LineWidth', 2.5)
ylabel('Voltage of Motors [V]', 'FontSize', 16)
legend('V1', 'V2', 'V3', 'V4')
set(gca,'FontSize',14)

figure(5)
plot(xout_mod(:,8),xout_mod(:,7), 'LineWidth', 2.5)
xlabel('y_pos [m]', 'FontSize', 16)
ylabel('x_pos [m]', 'FontSize', 16)
set(gca,'FontSize',14)

% subplot(3,1,2)
% plot(time, deltapsi_mod*180/pi)
% xlabel('time(s)')
% ylabel('error(degrees)')
% subplot(3,1,3)
% plot(time, resultant_v_mod)
% xlabel('time(s)')
% ylabel('resultant velocity m/s')
% 
% % figure(3)
% % subplot(5,1,1)
% % plot(time, Fa_mod)
% % xlabel('time (s)')
% % ylabel('Magnitude Fa')
% % 
% % subplot(5,1,2)
% % plot(time, Fr_mod)
% % xlabel('Time (s)')
% % ylabel('Magnitude Fr')
% % subplot(5,1,3)
% % plot(time, xout_mod(:,7))
% % hold on
% % plot(time, xout_mod(:,8))
% % xlabel('time (s)')
% % ylabel('co-ordinate (m)')
% % legend('X', 'Y')
% % subplot(5,1,4)
% % plot(time, Fx_mod)
% % xlabel('time (s)')
% % ylabel('Magnitude Fx')
% % subplot(5,1,5)
% % plot(time, Fy_mod)
% % xlabel('time (s)')
% % ylabel('Magnitude Fy')
% 
%% Display the results
% Plot the trajectory
figure(18);
% Now the attractive potential contour
xmin = -10; xmax = 35;
ymin = -10; ymax = 35;
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
set(gca,'FontSize',14)
% Now the repulsive potential contour
........
% The obstacle contours
for kk = 1:nobs
    r = 0:0.01:rho_0(kk);
    tht = 0:0.1:2*pi+0.1;
    [THT,R] = meshgrid(tht,r);
    Z = 0.5*Kr(kk)*((sqrt((R.*cos(THT)).^2+(R.*sin(THT)).^2))-(rho_0(kk))).^2;
    Zmax = max(max(Z));
    Z = 10*Z/Zmax;
    contour(yo(kk)+R.*cos(THT),xo(kk)+R.*sin(THT),Z,50);
end
% Draw the obstacle boundaries
for jj = 1:nobs
    tht = 0:0.01:2*pi;
    xobs = yo(jj)+rho_0(jj)*cos(tht);
    yobs = xo(jj)+rho_0(jj)*sin(tht);
    plot(xobs,yobs,':r','LineWidth',2.0);
end
%% Finally, plot the trajectory
plot(traj(2,:),traj(1,:),'g','LineWidth',1.5);
hold off;
axis equal;
axis tight; 

% Plot the theoretical trajectory
figure(15);
plot(yt_mod, xt_mod)
xlabel('yt (m)')
ylabel('xt (m)')

figure(16)
plot(time, vx_mod)
hold on
plot(time, vy_mod)
xlabel('Time (s)')
ylabel('Theoretical Velocity (m/s)')
legend('vx', 'vy')

figure(17)
plot(xt_mod)
legend('xt')

figure(8)
plot(yt_mod)
legend('yt')

% 
% 
figure 
p = 0.5*Ka*((X-Xg).^2+(Y-Yg).^2);
mesh(X,Y,p);

for n=1:nobs
    
    rho_r = sqrt((X-xo(n)).^2+(Y-yo(n)).^2);
    q = q + 0.5*Kr(n)*((1./rho_r-1./rho_0(n)).^2);  %CHANGE THIS BACK TO THE RIGHT FIELD AS IN FR CALC
    
end 
% 
% s = 2;
% ming = 5;
% r = s*((X-Xg).^2+(Y-Yg).^2)+ ming;
% 
%  
%  figure
%  mesh(X,Y,q)
%  figure
%  mesh(X,Y,r)
%  figure
%  mesh(X,Y,p+q)
% 
% 
%  