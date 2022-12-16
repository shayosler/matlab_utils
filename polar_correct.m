function corrected = polar_correct(vals, min, max)
% corrected = polar_correct(vals, min, max)
%   Polar correct the values in vals to the range (min, max]
%   Maps real valued scalars to rotations in SO(1)

upper = max;
lower = min;
if max < min
    upper = min;
    lower = max;
end

period = abs(upper - lower);
gap = zeros(size(vals));
gap(vals >= upper) = vals(vals >= upper) - upper;
gap(vals < upper) = abs(lower - vals(vals < upper));
phase = mod(gap, period);

corrected = vals;
corrected(vals >= upper) = lower + phase(vals >= upper);
corrected(vals < lower & phase == 0) = lower;
corrected(vals < lower & phase ~= 0) = upper - phase(vals < lower & phase ~= 0);

end
