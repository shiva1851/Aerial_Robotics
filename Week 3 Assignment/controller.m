function [u1, u2] = controller(~, state, des_state, params)

    % Constants for PD controller gains
    k_pz = 800;  % Proportional gain for altitude control
    k_dz = 800;  % Derivative gain for altitude control
    
    k_pphi = 2000;  % Proportional gain for roll control
    k_dphi = 2000;  % Derivative gain for roll control
    
    k_py = 150;    % Proportional gain for lateral control
    k_dy = 150;    % Derivative gain for lateral control
    
    % Initialize control inputs
    u1 = 0;
    u2 = 0;

    % Calculate control inputs using PD control
    error_z = des_state.pos(2) - state.pos(2);
    error_phi = des_state.pos(1) - state.pos(1);

    % Proportional and derivative terms for altitude control
    u1 = params.mass * (params.gravity + k_pz * error_z + k_dz * (des_state.vel(2) - state.vel(2)));

    % Proportional and derivative terms for roll control
    phic = -1 / params.gravity * (des_state.acc(1) + k_py * error_phi+k_dy*(des_state.vel(1) - state.vel(1)));
    phic_dot = 0;  % Derivative of the desired roll angle
    u2 = params.Ixx * (k_pphi * (phic - state.rot) + k_dphi * (phic_dot - state.omega));

end
