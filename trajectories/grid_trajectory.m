function [x] = grid_trajectory(t, u, l, d, psi0, n0, e0, z0)
% [x] = grid_trajectory(t, u, l, d, psi0, n0, e0, z0)
% Calculate the instantaneous reference state of the vehicle at time t
% for a reference trajectory that "mows the lawn" over a rectangular area
% at a constant velocity
%
% Inputs:
%   t       Time to calculate trajectory at
%   u       Forward velocity
%   l       Line length (m)
%   d       line separation (m)
%   psi0    Initial line direction
%   n0      Northing of start point    
%   e0      Easting of start point
%   z0;     Depth to run at
%
% Outputs:
%   x       State from trajectory at time t:
%           [n e z phi theta psi n_dot e_dot z_dot phi_dot theta_dot psi_dot]'

% Derived parameters
t_l = l / u;                % Time to complete a line
t_lc = (pi * d) / (2 * u);  % Time to complete a line change
rot_rate = 180 / t_lc;      % Rotation rate during line changes
p = t_l + t_lc;             % Line "period"
%t_total = t_l + (n - 1) * p;% Total time to complete the grid 

% Determine parameters of the line we're currently on
% Determine which line we're on
line = floor(t / p);

% Direction of travel for the line
line_dir = psi0 + 180 * mod(line, 2);  

% Direction of rotation for the line change: +1 = CW, -1 = CCW
change_dir = 1 - 2 * mod(line, 2);

% Calculate line start and end points
a = [e0 n0] + d * line * [sind(psi0 + 90) cosd(psi0 + 90)];
b = a + l * [sind(psi0) cosd(psi0)];
if mod(line, 2) == 0
    first = a;
    second = b;
else
    first = b;
    second = a;
end

% Determine if we're in the line or the line change
line_time = t - line * p;
if line_time <= t_l
    % line
    dist = u * line_time;
    pos = first + dist * [sind(line_dir) cosd(line_dir)];
    n = pos(2);
    e = pos(1);
    z = z0;
    phi = 0;
    theta = 0;
    psi = line_dir;
    n_dot = u * cosd(psi);
    e_dot = u * sind(psi);
    z_dot = 0;
    phi_dot = 0;
    theta_dot = 0;
    psi_dot = 0;
    
    %plot(e, n, 'b*')
    %hold on    
else
    % line change: semicircle connecting the lines
    change_time = line_time - t_l;
    
    % Center point of the semicircle
    ctr = second + d/2 * [sind(psi0 + 90) cosd(psi0 + 90)];
    
    % Bearing from center to current point on line
    brng = (psi0 - 90) + rot_rate * change_time * change_dir;
    
    % Location
    pos = ctr + d/2 * [sind(brng) cosd(brng)];
    n = pos(2);
    e = pos(1);
    z = z0;
    phi = 0;
    theta = 0;
    psi = brng + change_dir * 90;

    n_dot = u * cosd(psi);
    e_dot = u * sind(psi);
    z_dot = 0;
    phi_dot = 0;
    psi_dot = rot_rate * change_dir;
    theta_dot = 0;
    
    %h_ctr = plot(ctr(1), ctr(2), 'r*');
    %hold on
    %plot(e, n, 'b*')    
    %delete(h_ctr);
end

x = [n e z phi theta psi n_dot e_dot z_dot phi_dot theta_dot psi_dot]';

end
