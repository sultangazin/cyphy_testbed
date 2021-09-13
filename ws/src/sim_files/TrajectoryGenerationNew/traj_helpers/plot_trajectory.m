function [] = plot_trajectory(trajectory, dt, der)
    Tend = trajectory.Tfinal;
    Nsteps = Tend / dt;
    trj_pos = zeros(3, Nsteps);
    for (step = 1 : Nsteps)
        trj_pos(:, step) = trajectory.trj_eval(step * dt, der);
    end
    
    time = [0:Nsteps-1] * dt;
    
    figure;
    subplot(3,1,1);
    plot(time, trj_pos(1,:))
    subplot(3,1,2);
    plot(time, trj_pos(2,:))
    subplot(3,1,3);
    plot(time, trj_pos(3,:))
end