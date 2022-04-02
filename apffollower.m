%% POTENTIAL FIELDS SCRIPT

function [xt, yt, vx, vy, vr, Fa, Fr, Fd, Fx, Fy, rho_r, Far, Frr1, Frr2, rho_rr] = apffollower(xt, yt, vx, vy, Xg, Yg, xo, yo, noo, Ka, Kr, Kdamp, m, rho_0, dt, xt2, yt2, dx, dy, xt3, yt3)

Kar = 50;
% Calculate the conservative forces
    % Attractive force (Fa)
    Fa = zeros(2,1);
%     Fa(1,1) = -Ka*(xt-Xg);
%     Fa(2,1) = -Ka*(yt-Yg);
%     fprintf('Fa\n')
%     disp(Fa)
    % Repulsive force (Fr)
    Fr = zeros(2,1);
    if noo > 0
        for j=1:noo
        
            rho_r = sqrt((xo(j)-xt)^2+(yo(j)-yt)^2);
        
            if (rho_r <= rho_0(j))
              %rho_r = sqrt((xo(i)-x)^2+(yo(i)-y)^2);
                 Fr(1,1) = Fr(1,1) - Kr(j)*(rho_r-rho_0(j))*(xt-xo(j))*(1/rho_r);
                 Fr(2,1) = Fr(2,1) - Kr(j)*(rho_r-rho_0(j))*(yt-yo(j))*(1/rho_r);
            end     
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
    % Attractive force to other robots
    Far = zeros(2,1);
    Far(1,1) = -Kar*((xt-xt2)/(abs(xt-xt2))*(1-dx/(abs(xt-xt2))));
    Far(2,1) = -Kar*((yt-yt2)/(abs(yt-yt2))*(1-dy/(abs(yt-yt2))));
    % Repulsive force to other robots
    Frr1 = zeros(2,1);
     Krr = 300;
     rho_rr = sqrt((xt2-xt)^2+(yt2-yt)^2);
     z1 = 0.5;  %should be same as dx and dy!
     z2 = 0.5;
     if (rho_rr <= z1)
        Frr1(1,1) = Frr1(1,1) - Krr*(rho_rr-z1)*(xt-xt2)*(1/rho_rr);
        Frr1(2,1) = Frr1(2,1) - Krr*(rho_rr-z1)*(yt-yt2)*(1/rho_rr);
     end
     Frr2 = zeros(2,1);
     rho_rr = sqrt((xt3-xt)^2+(yt3-yt)^2);
     Krr = 300;
     z2 = 0.5;
     z2 = 0.5;
     if (rho_rr <= z2)
        Frr2(1,1) = Frr2(1,1) - Krr*(rho_rr-z2)*(xt-xt3)*(1/rho_rr);
        Frr2(2,1) = Frr2(2,1) - Krr*(rho_rr-z2)*(yt-yt3)*(1/rho_rr);
     end
    % Added Force
    Fadd = zeros(2,1);
    rho_g = sqrt((xt3-Xg)^2+(yt3-Yg)^2);
    if vx < 0.0001 && rho_g > 0.2
         Fadd(1,1) = 3*0.3*(xt-Xg);
    end 
    if vy < 0.0001 && rho_g > 0.2
         Fadd(2,1) = 3*0.3*(yt-Yg);
    end
    % Total force (F)
    Fy = Fa(2,1) + sum(Fr(2,1)) + Fd(2,1) + Far(2,1) + Fadd(2,1) + Frr1(2,1) + Frr2(2,1);
    Fx = Fa(1,1) + sum(Fr(1,1)) + Fd(1,1) + Far(1,1) + Fadd(1,1) + Frr1(1,1) + Frr2(1,1);
    F(1,1) = Fx;
    F(2,1) = Fy;
    fprintf('Fx\n')
    disp(Fx)
    fprintf('Fy\n')
    disp(Fy)
    % Velocity and Position (vx, vy, x, y) through integrating the equations
    vx = vx + dt*(Fx/m);
%     if vx <= 0.0001
%         vx = vx + 0.1
%     end
    fprintf('vx\n')
    disp(vx)
    vy = vy + dt*(Fy/m);
%     if vy <= 0.0001
%         vy = vy + 0.1
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
    