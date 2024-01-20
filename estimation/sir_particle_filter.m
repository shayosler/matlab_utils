function [x_hat, sigma, pk1] = sir_particle_filter(pk, uk, f, Qsq, zk1, h, L)
% Estimate the state of a system using an SIR particle filter
% Assumes measurements are independent
% Inputs:
%   pk      Set of particles at time k, n x n_particles
%   uk      Control inputs at time k, mx1
%   Qsq     Square root of the process" noise covariance matrix, nxn
%   f       Discrete state transition function: x[k+1] = f(x[k], u[k])
%   zk1     Measurement for time k+1
%   h       Function mapping state to measurements: zk = h(xk)
%   L       Distributions representing sensor noise models for each
%           measurement, mx1
%
% Outputs:
%   x_hat   State estimate for time k+1
%   sigma   Estimate standard deviation for time k+1
%   pk1     Particles for time k+1
%   wk1     Weights for time k+1

%% For each particle, propagate it forward, then calculate the likelihood
% TODO: can vectorize this more if f() and h() are set up to handle vectors
n = size(pk, 1);            % Dimensionality of state vector
n_particles = size(pk, 2);  % Number of particles
pk1 = zeros(size(pk));
l = ones(n_particles, 1);   % likelihood for each particle
for i = 1:n_particles
    % Propagate particles forward, with some noise
    w = Qsq * randn(n, 1);
    pk1(:, i) = f(pk(:, i), uk) + w;

    % Calculate innovation for particle i
    innov = zk1 - h(pk1(:, i));

    % Calculate likelihood for particle i given zkp1
    % Independent measurements, total likelihood is product of
    % likelihoods for each measurement
    for d = 1:length(zk1)
        l(i) = l(i) * pdf(L(d), innov(d));
    end
end

% Update weights then normalize
wk1 = l ./ sum(l);

%% Resample
resample_idxs = randsample(n_particles, n_particles, true, wk1);
pk1 = pk1(:, resample_idxs);
% TODO: resample with added noise?

%% Calculate new estimate
x_hat = mean(pk1, 2);
sigma = std(pk1')';

end