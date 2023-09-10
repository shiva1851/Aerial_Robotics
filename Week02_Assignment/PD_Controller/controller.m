function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;
e = s_des - s; %error

% FILL IN YOUR  CODE HERE
kp=120;
kv=250;
%kp=150;
%kv=200;
%Tune your PD Controller

a_z_des =0;
u= params.mass*((a_z_des)+kp*e(1) +kv*e(2) +params.gravity);
% thrust equation here
%Implement PD Control

if (u<params.u_min) 
    u=params.u_min; 
end
if (u>params.u_max)
    u=params.u_max; 
end

end