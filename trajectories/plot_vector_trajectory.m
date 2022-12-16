function [h] = plot_vector_trajectory(N, E, s, u, l, d, psi0, n0, e0, z0, obs, d_max, varargin)
% [h] = plot_vector_trajectory(N, E, s, u, l, d, psi0, n0, e0, z0)
% Plot the vector field guiding a vehicle alone a reference trajectory 
% comprised of a series of either straight or semicircular line segments 
% that "mows the lawn" over a rectangular area at a constant velocity.
%
% Inputs:
%   N       Northing positions to plot, matrix generated with meshgrid
%   E       Easting positions to plot
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

E_dot = zeros(size(E));
N_dot = zeros(size(N));
for i = 1:size(E, 1)
    for j = 1:size(E, 2)
        e_ij = E(i, j);
        n_ij = N(i, j);
        x = [n_ij e_ij 0 0 0 0 0 0 0 0];
        [~, ~, r] = vector_trajectory(x, s, u, l, d, psi0, n0, e0, z0, obs, d_max);
        n_dot_ij = r(6);
        e_dot_ij = r(7);
        N_dot(i, j) = n_dot_ij;
        E_dot(i, j) = e_dot_ij;
    end 
end

quiver(E, N, E_dot, N_dot, varargin{:})

end