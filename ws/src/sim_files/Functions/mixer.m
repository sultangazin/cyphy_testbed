function [thrust, torques] = mixer(m_thrusts, m_torques, arm)
%
%    +    -
% -  4    1
% +  3    2
%
    N_motors = length(m_thrusts);
    Trq = zeros(3, 1);
    Thr = 0.0;
    
    m_thrusts = reshape(m_thrusts, N_motors, 1);
    
    % Mixing matrix for generating the torques
    %           m1  m2 m3 m4
    %   roll    -   -  +  + 
    %   pitch   -   +  +  -
    %   yaw     +   -  +  -
    Mix = [
        -1, -1, 1,  1;
        -1,  1, 1, -1;
         1, -1, 1, -1];
    
    if (size(Mix, 2) ~= N_motors) 
        error('Mixer matrix: Size mismatch!');
    else
        Trq(1:2) = Mix(1:2, :) * m_thrusts * arm;
        Trq(3) = dot(Mix(3, :), m_torques); 
        Thr = sum(m_thrusts);
    end
    
    thrust = Thr;
    torques = Trq;
end