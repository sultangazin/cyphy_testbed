function f = cmd2f(cmd)
    % Function mapping the motor command to the generated force
    f = 2.130295e-11 .* cmd.^2 + 1.032633e-6 .* cmd + 5.48456e-4;
end