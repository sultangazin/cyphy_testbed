% Nx - dimension of the state
% dt - sampling time
% p_s - start the trajectory
% p_f - end the trajectory

function [time, trj_pos, trj_vel, trj_acc, trj_jerk] = CreateTrajectoryNew(xx0, xxe, Ttrj_end, Nx, poly_order, dt)

trajectory = TrajectoryClass(poly_order);
% Generate the polynomial that performs the interpolation.
trajectory = trajectory.generate(xx0, xxe, Ttrj_end);

Nsim = Ttrj_end/dt;             % simulation steps
trj_pos = zeros(3, Nsim);
trj_vel = zeros(3, Nsim);
trj_acc = zeros(3, Nsim);
trj_jerk = zeros(3, Nsim);
for step = 1 : Nsim
    % Get trajectory:
    trj_pos(:, step) = trajectory.trj_eval(step * dt, 0);
    trj_vel(:, step) = trajectory.trj_eval(step * dt, 1);
    trj_acc(:, step) = trajectory.trj_eval(step * dt, 2);
    trj_jerk(:, step) = trajectory.trj_eval(step * dt, 3);
end
time = [0:(Nsim-1)] * dt;
end