function T = wheelTransformation(wheel_frame, rocker, bogie, steer)

%inputs: wheel_frame ('FL','FR','CL','CR','RL','RR'), rocker angle, bogie angle,
%steering angle (all in radians).
%example for front left frame: wheelTransformation('FL', 0.0873, 0.0349,
%0.1222)
%output: T_z_wheelframe, the transformation from the given wheel reference frame
%located at the geometric center of the wheel and the IMU reference frame

%note: The function assumes the bogie and/or steer value coressponds to
%the given wheel_frame (ie the rear right wheel will assume bogie = right
%bogie angle, steer = steering angle of rear right wheel. For wheels that
%don't have a bogie or steering value, whatever is entered will be ignored.


%translations from frame A to frame B, in frame A
%eg. p_frameB_frameA
p_RightRocker_IMU = [0.110;0.370;0.031];
p_LeftRocker_IMU = [0.110;-0.406;0.031];

p_FrontLeftSteer_LeftRocker = [0.750;0.022;-0.298];
p_FrontRightSteer_RightRocker = [0.750;-0.022;-0.298];
p_LeftBogie_LeftRocker = [-0.375; 0.022; -0.260];
p_RightBogie_RightRocker = [-0.375; -0.022; -0.260];

p_CenterLeftWheel_LeftBogie = [0.375;-0.002;-0.192];
p_RearLeftSteer_LeftBogie = [-0.375;0;-0.037];
p_CenterRightWheel_RightBogie = [0.375;0.002;-0.192];
p_RearRightSteer_RightBogie = [-0.375;0;-0.037];

p_FrontLeftWheel_FrontLeftSteer = [0.000;-0.002;-0.155];
p_RearLeftWheel_RearLeftSteer = [0.000;-0.002;-0.155];
p_FrontRightWheel_FrontRightSteer = [0.000;0.002;-0.155];
p_RearRightWheel_RearRightSteer = [0.000;0.002;-0.155];

yaw180 = [-1,0,0;0,-1,0;0,0,1];

%transformations

%Is it on the left side?
if isequal(wheel_frame,'FL') || isequal(wheel_frame,'CL') || isequal(wheel_frame,'RL')
    T_IMU_LeftRocker = [yaw180*rpy_R(0,rocker,0) p_LeftRocker_IMU; zeros(1,3), 1];
    %is it the FL wheel?
    if isequal(wheel_frame,'FL')
        T_LeftRocker_FrontLeftSteer = [rpy_R(0,0,steer) p_FrontLeftSteer_LeftRocker; zeros(1,3), 1];
        T_FrontLeftSteer_FrontLeftWheel = [rpy_R(0,0,0) p_FrontLeftWheel_FrontLeftSteer; zeros(1,3), 1];
        T = T_IMU_LeftRocker*T_LeftRocker_FrontLeftSteer*T_FrontLeftSteer_FrontLeftWheel;
    else
        T_LeftRocker_LeftBogie = [rpy_R(0,bogie,0) p_LeftBogie_LeftRocker; zeros(1,3), 1];
        %Is it the Center Left Wheel?
        if isequal(wheel_frame,'CL')
            T_LeftBogie_CenterLeftWheel = [rpy_R(0,0,0) p_CenterLeftWheel_LeftBogie; zeros(1,3), 1];
            T = T_IMU_LeftRocker*T_LeftRocker_LeftBogie*T_LeftBogie_CenterLeftWheel;
        else
            T_LeftBogie_RearLeftSteer = [rpy_R(0,0,steer) p_RearLeftSteer_LeftBogie; zeros(1,3), 1];
            T_RearLeftSteer_RearLeftWheel = [rpy_R(0,0,0) p_RearLeftWheel_RearLeftSteer; zeros(1,3), 1];
            T = T_IMU_LeftRocker*T_LeftRocker_LeftBogie*T_LeftBogie_RearLeftSteer*T_RearLeftSteer_RearLeftWheel;
        end
    end       
elseif isequal(wheel_frame,'FR') || isequal(wheel_frame,'CR') || isequal(wheel_frame,'RR')
    T_IMU_RightRocker = [yaw180*rpy_R(0,-rocker,0) p_RightRocker_IMU; zeros(1,3), 1];
    %is it the Front Right wheel?
    if isequal(wheel_frame,'FR')
        T_RightRocker_FrontRightSteer = [rpy_R(0,0,steer) p_FrontRightSteer_RightRocker; zeros(1,3), 1];
        T_FrontRightSteer_FrontRightWheel = [rpy_R(0,0,0) p_FrontRightWheel_FrontRightSteer; zeros(1,3), 1];
        T = T_IMU_RightRocker*T_RightRocker_FrontRightSteer*T_FrontRightSteer_FrontRightWheel;
    else
        T_RightRocker_RightBogie = [rpy_R(0,bogie,0) p_RightBogie_RightRocker; zeros(1,3), 1];
        %Is it the Center Right Wheel?
        if isequal(wheel_frame,'CR')
            T_RightBogie_CenterRightWheel = [rpy_R(0,0,0) p_CenterRightWheel_RightBogie; zeros(1,3), 1];
            T = T_IMU_RightRocker*T_RightRocker_RightBogie*T_RightBogie_CenterRightWheel;
        else
            T_RightBogie_RearRightSteer = [rpy_R(0,0,steer) p_RearRightSteer_RightBogie; zeros(1,3), 1];
            T_RearRightSteer_RearRightWheel = [rpy_R(0,0,0) p_RearRightWheel_RearRightSteer; zeros(1,3), 1];
            T = T_IMU_RightRocker*T_RightRocker_RightBogie*T_RightBogie_RearRightSteer*T_RearRightSteer_RearRightWheel;
        end
    end   
else
    disp('frame doesn''t exist');
end 

function R = rpy_R(phi,theta,psi)

% R = rpy_R(r,p,y)
%
% Computes the rotation matrix for a RPY rotation, R.

[Rr,Rp,Ry] = rpy_rot_set(phi,theta,psi);

R = Ry*Rp*Rr;


function [R_phi,R_theta,R_psi] = rpy_rot_set(phi,theta,psi)

% [Rr, Rp, Ry] = rpy_rot_set(r,p,y)
%
% Computes the individual rotation matrices for separate roll, pitch, and
% yaw rotations, Rr, Rp, and Ry.

R_phi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
R_psi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];