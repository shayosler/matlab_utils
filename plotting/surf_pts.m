function h_out = surf_pts(x, y, z, c, h)
%Interpolate mesh over points and plot
if nargin < 5
    h = gca;
end

%% Determine points for interpolation
x_min = min(x);
x_max = max(x);
y_min = min(y);
y_max = max(y);
step = 1;
[xp, yp] = meshgrid(x_min:step:x_max, y_min:step:y_max);

%% Interpolate grid
zp = griddata(x, y, z, xp, yp);
if(nargin < 4)
    c = zp; 
end
h_out = surf(h, xp, yp, zp, c, 'FaceAlpha', 0.75, 'EdgeColor', 'none');
colorbar


end