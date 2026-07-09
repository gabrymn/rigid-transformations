% 2D transforms for the odometry of a differential kinematics mobile robot                        

figure; hold on; grid on; axis equal;

ssx=ones(1,4)*0.2; sdx=ones(1,4)*0.2;
ssx=[ssx ones(1,4)*0.4]; sdx=[sdx ones(1,4)*0.4];
ssx=[ssx ones(1,4)*0.6]; sdx=[sdx ones(1,4)*0.6];
ssx=[ssx ones(1,4)*0.8]; sdx=[sdx ones(1,4)*0.8];
ssx=[ssx ones(1,4)*1.0]; sdx=[sdx ones(1,4)*1.0];
ssx=[ssx ones(1,4)*1.2]; sdx=[sdx ones(1,4)*1.2];
ssx=[ssx ones(1,4)*1.0]; sdx=[sdx ones(1,4)*1.0];
ssx=[ssx ones(1,4)*0.8]; sdx=[sdx ones(1,4)*0.8];
ssx=[ssx ones(1,4)*0.6]; sdx=[sdx ones(1,4)*0.6];
ssx=[ssx ones(1,4)*0.4]; sdx=[sdx ones(1,4)*0.4];
ssx=[ssx ones(1,4)*0.2]; sdx=[sdx ones(1,4)*0.2];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.19];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.18];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.17];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.16];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.14];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.13];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.12];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.11];
ssx=[ssx ones(1,1)*0.2]; sdx=[sdx ones(1,1)*0.1];
ssx=[ssx ones(1,4)*0.2]; sdx=[sdx ones(1,4)*0.1];
ssx=[ssx ones(1,1)*0.19]; sdx=[sdx ones(1,1)*0.11];
ssx=[ssx ones(1,1)*0.18]; sdx=[sdx ones(1,1)*0.12];
ssx=[ssx ones(1,1)*0.17]; sdx=[sdx ones(1,1)*0.13];
ssx=[ssx ones(1,1)*0.16]; sdx=[sdx ones(1,1)*0.14];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,20)*0.15]; sdx=[sdx ones(1,20)*0.15];
ssx=[ssx ones(1,2)*0.15]; sdx=[sdx ones(1,2)*0.14];
ssx=[ssx ones(1,2)*0.15]; sdx=[sdx ones(1,2)*0.13];
ssx=[ssx ones(1,8)*0.15]; sdx=[sdx ones(1,8)*0.12];
ssx=[ssx ones(1,2)*0.15]; sdx=[sdx ones(1,2)*0.13];
ssx=[ssx ones(1,2)*0.15]; sdx=[sdx ones(1,2)*0.14];
ssx=[ssx ones(1,10)*0.15]; sdx=[sdx ones(1,10)*0.15];
ssx=[ssx ones(1,60)*0.15]; sdx=[sdx ones(1,60)*0.14];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.13];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.12];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.11];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.10];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.09];
ssx=[ssx ones(1,15)*0.15]; sdx=[sdx ones(1,15)*0.08];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.09];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.10];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.11];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.12];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.13];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.14];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,5)*0.15]; sdx=[sdx ones(1,5)*0.15];
ssx=[ssx ones(1,10)*0.15]; sdx=[sdx ones(1,10)*0.14];
ssx=[ssx ones(1,1)*0.15]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,1)*0.14]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,1)*0.13]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,30)*0.12]; sdx=[sdx ones(1,30)*0.15];
ssx=[ssx ones(1,1)*0.13]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,1)*0.14]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,10)*0.15]; sdx=[sdx ones(1,10)*0.15];
ssx=[ssx ones(1,1)*0.14]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,5)*0.13]; sdx=[sdx ones(1,5)*0.15];
ssx=[ssx ones(1,1)*0.14]; sdx=[sdx ones(1,1)*0.15];
ssx=[ssx ones(1,15)*0.15]; sdx=[sdx ones(1,15)*0.15];
ssx=[ssx ones(1,7)*0.14]; sdx=[sdx ones(1,7)*0.15];
ssx=[ssx ones(1,10)*0.15]; sdx=[sdx ones(1,10)*0.15];

arcs_length = length(ssx);

a1=0.3; a2=0.1; a3=0.2; a4=0.05;
%                    A      B      C      D      E
xBodyPoints = [    -a1    -a1     a1  a1+a2     a1];
yBodyPoints = [     a1    -a1    -a1      0     a1];
%                         F      G      H      I
xLeftWheelPoints = [    -a3    -a3     a3     a3];
yLeftWheelPoints = [  a1+a4     a1     a1  a1+a4];
%                          L      M      N      P
xRightWheelPoints = [    -a3    -a3     a3     a3];
yRightWheelPoints = [    -a1 -a1-a4 -a1-a4    -a1];

body_hc = [xBodyPoints; yBodyPoints; [1 1 1 1 1]];
leftwheel_hc = [xLeftWheelPoints; yLeftWheelPoints; [1 1 1 1]];
rightwheel_hc = [xRightWheelPoints; yRightWheelPoints; [1 1 1 1]];

% Baseline
b = 0.6;    

% pose iniziale (x=0 y=0 theta=0)
T_world = [cos(0) -sin(0) 0; 
           sin(0) cos(0)  0; 
           0      0       1];

for k = 1:arcs_length

    dsx = ssx(k);
    ddx = sdx(k);
    
    if abs(dsx - ddx) < 0.0001 

        T_rel = [1, 0, dsx; % o ddx
                 0, 1, 0; 
                 0, 0, 1];
        
    else
        
        dtheta = (dsx - ddx) / b;
        d = ddx / dtheta; 
        d_halfb = d + b/2;


        T_robot_to_CIR = [1, 0, 0; 
                          0, 1, d_halfb; 
                          0, 0, 1];
        
        R_CIR = [cos(-dtheta), sin(dtheta),  0; 
                 sin(-dtheta), cos(-dtheta), 0; 
                 0,            0,            1];
        
        T_CIR_to_robot = [1, 0, 0; 
                          0, 1, -d_halfb; 
                          0, 0, 1];
        

        T_rel = T_CIR_to_robot * R_CIR * T_robot_to_CIR;

    end

    T_world = T_world * T_rel;

    curr_body_hc = T_world * body_hc; 
    curr_leftwheel_hc = T_world * leftwheel_hc;
    curr_rightwheel_hc = T_world * rightwheel_hc;
    
    patch(curr_body_hc(1,:), curr_body_hc(2,:), 'r');
    patch(curr_leftwheel_hc(1,:), curr_leftwheel_hc(2,:),'c');
    patch(curr_rightwheel_hc(1,:), curr_rightwheel_hc(2,:), 'c');
    
    pause(0.02);
    
end

hold off;

% pose finale
x_f = T_world(1,3); 
y_f = T_world(2,3); 
theta_f = atan2(T_world(2,1), T_world(1,1));

disp("Pose finale (x_f, y_f, θ_f)");
disp([x_f y_f theta_f]);

