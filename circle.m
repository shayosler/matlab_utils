function h = circle(x, y, rad, varargin)
%CIRCLE   Draw a (filled) circle
%

if nargin >= 4
  edge_color = varargin{4};
else
  edge_color = 'b';
end

if nargin >= 5
  fill_color = varargin{5};
else
  fill_color = 'w';
end

%% Get rectangle params
%lower left corner
ll_x = x - rad;
ll_y = y - rad;
width = 2 * rad;
height = 2 * rad;

h = rectangle('Position', [ll_x ll_y width height], 'Curvature', [1 1],'EdgeColor', edge_color, 'FaceColor', fill_color);

end
