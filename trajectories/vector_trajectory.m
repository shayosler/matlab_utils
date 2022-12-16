function [s, xte, r] = vector_trajectory(x, s, u, l, d, psi0, n0, e0, z0, obs, d_max)
% [s, xte, r] = vector_trajectory(x, s, u, l, d, psi0, n0, e0, z0)
% Based on the current vehicle position, calculate the desired states to 
% guide it along a reference trajectory comprised of a series of either
% straight or semicircular line segments that "mows the lawn" over a 
% rectangular area at a constant velocity.
%
% Inputs:
%   x       Current vehicle state:
%           [n e z theta psi n_dot e_dot z_dot theta_dot psi_dot]'
%   s       Current trajectory segment
%   u       Forward velocity
%   l       Line length (m)
%   d       line separation (m)
%   psi0    Initial line direction
%   n0      Northing of start point    
%   e0      Easting of start point
%   z0      Depth to run at
%   obs     Matrix of obstacle descriptions, one per row [N E radius]
%   d_max   Maximum distance at which obstacles should influence path
%
% Outputs:
%   s       New trajectory segment
%   xte     Cross track error
%   r       State from trajectory at time t:
%           [n e z theta psi n_dot e_dot z_dot theta_dot psi_dot]'

%% Calculate vector component for trajectory following
% Determine which "line" we're on. Each "line" is comprised of a straight
% segment and then a semicircular line change segment
line = floor(s / 2);

% Direction of travel for the line
line_dir = psi0 + 180 * mod(line, 2);  

% Direction of rotation for the line change: +1 = CW, -1 = CCW
change_dir = 1 - 2 * mod(line, 2);

% Calculate line start and end points
a = [n0 e0] + d * line * [cosd(psi0 + 90) sind(psi0 + 90)];
b = a + l * [cosd(psi0) sind(psi0)];
if mod(line, 2) == 0
    first = a;
    second = b;
else
    first = b;
    second = a;
end

% Determine if we're in the line or the line change
if mod(s, 2) == 0
    % line
    % Position relative to first point on the line
    pos = [x(1) x(2)] - first;
    % Calculate cross track and along track distance
    R = [cosd(line_dir) -sind(line_dir);
        sind(line_dir) cosd(line_dir)];
    track_dist = R' * (pos)';
    atd = track_dist(1); % Along track distance
    xte = track_dist(2); % Cross track error
    
    % Calculate desired heading at the current point
    psi_inf = 60;   % Line relative heading at xte=inf
    K = .5;          % Gain on xte
    psi = line_dir - psi_inf * (2 / pi) * atan(K * xte);
    
    % Alternate calculation
    % unit vector in line direction
    ul = (second - first) / norm(second - first);
    proj = ul * dot(ul, pos); % Projection of current position onto line
    perp = pos - proj;
    dist = norm(perp);
    
    n = proj(1);
    e = proj(2);
    
   
    % Determine if segment needs to be incremented. Increment if our along
    % track distance puts us past the end of the line
    if atd > l
        s = s + 1;
    end
else   
    % Center point of the semicircle
    ctr = second + d/2 * [cosd(psi0 + 90) sind(psi0 + 90)];
    
    % Position relative to the center
    pos = [x(1) x(2)] - ctr;
    
    % Bearing from center to current location relative to line direction
    brng = atan2d(pos(2), pos(1)) + line_dir;
    
    % Cross track error
    xte = d/2 - norm(pos);
    xte = xte * change_dir;
    
    % Calculate desired heading at the current point
    psi_inf = 90;   % Line relative heading at xte=inf
    K = .75;          % Gain on xte
    psi = (brng + 90) - psi_inf * (2 / pi) * atan(K * xte);
    
    % Project current location onto arc
    proj = ctr + d/2 * [cosd(brng) sind(brng)];
    n = proj(1);
    e = proj(2);
    
    % Determine if segment needs to be incremented. Increment if the angle
    % between current location and center of line change circle is greater
    % than 90
    if brng > change_dir * 90
        s = s + 1;
    end
end
z = z0;
theta = 0;
n_dot = u * cosd(psi);
e_dot = u * sind(psi);
z_dot = 0;
theta_dot = 0;
psi_dot = 0;

%% Calculate vector component d/t obstacles
% pos = [x(1) x(2)];
% 
% n_obs = 0; % northing component from obstacle
% e_obs = 0; % easting component from obstacle
% in_obs = false;
% for k = 1:size(obs, 1)
%     % Calculate distance to closest point on obstac
%     pos_o = [obs(k, 1) obs(k, 2)];
%     rad = obs(k, 3);
%     dist = norm(pos - pos_o) - rad; % distance to closest point on obstacle
%     if norm(pos - pos_o) < rad
%         in_obs = true;
%         break;
%     elseif dist < d_max
%        % Calculate unit vector in direction of repulsive "force" (i.e.
%        % directly away from obstacle
%        u_rep = (pos - pos_o) / norm(pos - pos_o);
%        
%        % bearing relative to line
%        brng = atan2d(u_rep(2), u_rep(1)) - line_dir;
%        brng = polar_correct(brng, -180, 180);
%        if brng < 0
%            brng = brng + 85;
%        else
%            brng = brng - 85;
%        end
%        u_rep = [cosd(brng) sind(brng)];
%        
%        % Calculate magnitude of repulsive "force"
%        C = 10;
%        f_rep = -C * (1/d_max - 1/dist) * (1/(dist^2)) * u_rep;
%        n_obs = n_obs + f_rep(1);
%        e_obs = e_obs + f_rep(2); 
%     end
% end
% 
% if in_obs
%     n_dot = 0;
%     e_dot = 0;
% else
%     % add trajectory and obstacle components
%     n_dot = n_dot + n_obs;
%     e_dot = e_dot + e_obs;
%    
%     % normalize so total vector has magnitude u
%     vel = u * ([n_dot e_dot] / norm([n_dot e_dot]));
%     n_dot = vel(1);
%     e_dot = vel(2);
%     
%     % Recalculate desired heading
%     psi = atan2(e_dot, n_dot);
% end

r = [n e z theta psi n_dot e_dot z_dot theta_dot psi_dot]';

end