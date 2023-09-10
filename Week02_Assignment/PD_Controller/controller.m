function [ u ] = pd_controller(~, s, s_des, params)


u = 0;
e = s_des - s; %error

kp=120;
kv=250;
%kp=150;
%kv=200;

a_z_des =0;
u= params.mass*((a_z_des)+kp*e(1) +kv*e(2) +params.gravity);

if (u<params.u_min) 
    u=params.u_min; 
end
if (u>params.u_max)
    u=params.u_max; 
end

end
