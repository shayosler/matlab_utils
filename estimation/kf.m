function [xhat, P, nu, xhat_k1k, P_k1k] = kf(sys, x, P, Q, z, H, R, K)
%  [xhat, P] = kf(sys, x, P, Q, z, H, R)
% Calculate a state estimate and error covariance. Assumes 
% that a single time step has occured since the last time the
% estimate was calculated
% Inputs:
%   sys         Discrete time system 
%   x           Previous state estimate, Nx1
%   P           Previous estimate covariance, NxN
%   Q           Process noise covairance, NxN
%   z           Measurement, Mx1
%   H           Measurement model MxN
%   R           Measurement covariance MxM
%   K           Kalman gain to use, optional
%
% Outputs:
%   xhat        New state estimate
%   P           Covariance
%   nu          Innovation

% Get system model
F = sys.A;
G = sys.B;

% Predict
xhat_k1k = F*x;
P_k1k = F*P*F' + G*Q*G';

% Kalman Gain
if nargin < 8
    K = P_k1k * (H' / (H * P_k1k * H' + R));
end

% Innovation
nu = z - H * xhat_k1k;

% Update
xhat = xhat_k1k + K * nu;
I = eye(size(F));
P = (I - K * H) * P_k1k * (I - K * H)' + K * R * K';

end

function msq_err = mean_sq_err(xtrue, xhat)
% msq_err = mean_sq_err(xtrue, xhat)
% Calculate the mean squared error
N = size(xhat, 2);
err = xtrue - xhat;
errsq = err .^ 2;
msq_err = sum(errsq, 2) ./ N;

end
