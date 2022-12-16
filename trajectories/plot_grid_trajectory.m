function [h] = plot_grid_trajectory(t, u, l, d, psi0, n0, e0, z0, varargin)
% [h] = plot_grid_trajectory(t, u, l, d, psi0, n0, e0, z0, varargin)
% Plot the trajectory at the times in t for a reference trajectory "mows 
% the lawn" over a rectangular area at a constant velocity
%
% Inputs:
%   t       Times to calculate trajectory at
%   u       Forward velocity
%   l       Line length (m)
%   d       line separation (m)
%   psi0    Initial line direction
%   n0      Northing of start point    
%   e0      Easting of start point
%   z0      Depth to run at
%
% Outputs:
%   h       Vector of chart line objects comprising the plot

xd = zeros(10, length(t));
for j = 1:length(t)
    xd(:, j) = grid_trajectory(t(j), u, l, d, psi0, n0, e0, z0);
end
xd(5, :) = polar_correct(xd(5, :), 0, 360);
h = plot(xd(2, :), xd(1, :), varargin{:});

end