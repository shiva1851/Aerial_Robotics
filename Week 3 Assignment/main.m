function main()
    % Define parameters
    params.gravity = 9.81;
    params.mass = 1;
    params.Ixx = 0.1;
    params.arm_length = 0.5;
    params.minF = 0;
    params.maxF = 10;

    % Define initial state
    state.pos = [0; 0];
    state.vel = [0; 0];
    state.rot = 0;
    state.omega = 0;

    % Define initial desired state
    des_state = traj_diamond(0, state);

    % Simulation time parameters
    dt = 0.01;  % Time step
    t_end = 16; % Simulation duration

    % Simulation loop
    for t = 0:dt:t_end
        % Compute control inputs
        [u1, u2] = controller(t, state, des_state, params);

        % Simulate dynamics
        state = simulate_dynamics(state, u1, u2, params, dt);

        % Update desired state
        des_state = traj_diamond(t, state);

        % Visualize the drone (optional)
        visualize_drone(state);

        % Pause to visualize the animation (optional)
        pause(dt);
    end
end

function state_next = simulate_dynamics(state, u1, u2, params, dt)
    % Simulate the dynamics of the quadrotor

    % Extract states
    yz = state.pos;
    yz_dot = state.vel;
    phi = state.rot;
    phi_dot = state.omega;

    % Extract parameters
    g = params.gravity;
    m = params.mass;
    Ixx = params.Ixx;
    l = params.arm_length;

    % Compute accelerations
    accel_y = (u1 / m) * sin(phi);
    accel_z = (u1 / m) * cos(phi) - g;

    % Update states
    yz_next = yz + yz_dot * dt;
    yz_dot_next = yz_dot + [accel_y; accel_z] * dt;
    phi_next = phi + phi_dot * dt;
    phi_dot_next = phi_dot + (u2 / Ixx) * dt;

    % Update state struct
    state_next.pos = yz_next;
    state_next.vel = yz_dot_next;
    state_next.rot = phi_next;
    state_next.omega = phi_dot_next;
end

function visualize_drone(state)
    % Function to visualize the drone (optional)
    % Implement visualization code here
end

% Run the main script
main();