%% Clean the workspace
clear all;
close all;%% Model

%% Include necessary functions and constants
addpath('./Functions');
addpath('./Functions/Quaternion');

G_acc = 9.81;
Mass = 0.032;

InertiaMatrix = [
    16.571, 0.830, 0.718;
    0.831, 16.656, 1.800;
    0.718, 1.800, 29.262];
InertiaMatrix = InertiaMatrix * 1e-6;

Inv_InertiaMatrix = inv(InertiaMatrix);

body_arm = 0.04;

q0 = [1, 0, 0, 0]';

%% Geometric Controller from Faessler et al 2015

Kp_xy_mll = 7.0;    
Kd_xy_mll = 5.0;  
Ki_xy_mll = 0.0;   
Irange_xy_mll = 0.2; 
    
Kp_z_mll = 16.5;    
Kd_z_mll = 3.4;     
Ki_z_mll = 15.5;     
Irange_z_mll = 0.4;

Kpq = 6;      
Kr = 2;       

Mellinger_dt = 0.001;

%% Internal controller
Kw_xy = 20000;   
Kw_z = 12000;    
Kd_w_rp = 200;   

%% Cmd_vel_controller
magic = 40500/9.81;
Thr_upper = 60000;
Thr_lower = 0;

M_upper = 32000;
M_lower = -32000;

%% Simulate the expert
close all;

% Start
q0 = [1;0;0;0];

% Initial states for each of experiments/expert demonstrations
initials = eye(9);

% Increase the height
initials(3,3) = 0.7;
initials(3,6) = 0.7;
initials(3,9) = 0.7;

% Non-zero height to move sideways
initials(3,1) = 0.3;
initials(3,2) = 0.3;
initials(3,4) = 0.3;
initials(3,5) = 0.3;
initials(3,7) = 0.3;
initials(3,8) = 0.3;

% To be away from the origin when moving towards it
initials(1,4) = 1;
initials(2,5) = 1;
initials(1,7) = 1;
initials(2,8) = 1;

% size of an operational envelope
x_factor = 0.1;
initials = x_factor * initials;

% duration of each demonstration
T = 5;
T_stop = 10;
simT = T + T_stop;
% sampling time
Ts = 0.001;

for i = 1:9
    % Store a waypoint
    intermediate_z = initials(:,i);
    nx = 9;
    n_order = 7;
    % Create a trajectory from the origin to a waypoint
    [time_1,polys_1, polys_v_1, polys_a_1, polys_j_1] = CreateTrajectoryNew(zeros(9,1),intermediate_z,T,nx,n_order,Ts); 
    
    % Concatenate the position trajectory with a waypoint to the origin
    polys = [zeros(1,3); polys_1'; zeros(T_stop/Ts,3)];
    % Append the time vector
    polys = [(0:Ts:simT)', polys];
    % Concatenate the velocity trajectory with a waypoint to zero velocity
    polys_v = [zeros(1,3); polys_v_1'; zeros(T_stop/Ts,3)];
    polys_v = [(0:Ts:simT)', polys_v];
    % Concatenate the acc trajectory with a waypoint to zero acc
    polys_a = [zeros(1,3); polys_a_1'; zeros(T_stop/Ts,3)];
    polys_a = [(0:Ts:simT)', polys_a];

    out(i) = sim('sim_expert');
end

%% Construct and plot the demonstrations

% construct demonstration matrices (start time from the waypoint)
idx_begin = length(polys_1) + 1;
time = out(1).z.time(idx_begin:end) - min(out(1).z.time(idx_begin:end));
Z = zeros(9,9,size(time,1));    % trajectories
V = zeros(3,9,size(time,1));    % inputs

% store the demonstrations
for i = 1:9
    Z(:,i,:) = out(i).z.signals.values(idx_begin:end,:)';
    V(:,i,:) = out(i).v.signals.values(idx_begin:end,:)';
end

% Construct the "instanteneous" controller
K = zeros(3,9,size(time,1));
for i = 1:length(time)
   K(:,:,i) = V(:,:,i)/Z(:,:,i);
end

% Filter the gains to avoid discontinuities
windowSize = 100;
b = (1/windowSize) * ones(1, windowSize);
a = 1;

K = filter(b,a,K,[],3);
%% Plot weights
norm_K_sim = [];
for i = 1:size(K,3)
    norm_K_sim = [norm_K_sim; norm(K(:,:,i),'fro')];
end
figure;
plot(time, norm_K_sim);

Kjz = [];
for i = 1:size(K,3) 
    Kjz = [Kjz; K(3,:,i)];
end
figure;
plot(time, Kjz);
legend('x','y','z','vx','vy','vz','ax','ay','az');
axis([-Inf Inf -1000 1000]);
Kjx = [];
for i = 1:size(K,3)
    Kjx = [Kjx; K(1,:,i)];
end
figure;
plot(time, Kjx);
legend('x','y','z','vx','vy','vz','ax','ay','az');
axis([-Inf Inf -1000 1000]);
Kjy = [];
for i = 1:size(K,3)
    Kjy = [Kjy; K(2,:,i)];
end
figure;
plot(time, Kjy);
legend('x','y','z','vx','vy','vz','ax','ay','az');
axis([-Inf Inf -1000 1000]);

% %% (Debug) Plot the experiments
% figure;
% for i = 1:9
%     subplot(3,3,i);
%     plot(time, out(i).z.signals.values(idx_begin:end,:));
% end
% figure;
% for i = 1:9
%     subplot(3,3,i);
%     plot(time, out(i).v.signals.values(idx_begin:end,:));
% end

%% Export the gains
exportK = [];
idx_start = 3001;
idx_end = 5001;
ratio = norm(Z(:,:,idx_end)*inv(Z(:,:,idx_start)));
fprintf("The sufficient condition ||Z(T)Z^{-1}(0)}}: %3.3f\n", ratio);
for i=idx_start:idx_end
    exportK = [exportK; K(:,:,i)];
end
writematrix(exportK,'K.csv');