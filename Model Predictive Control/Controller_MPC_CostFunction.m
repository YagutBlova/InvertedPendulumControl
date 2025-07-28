function [H, F] = CostFunctionMPC(A, B, np, Q, R, P)
[nx, nu] = size(B);         % Get system dimensions


phi = zeros(np*nx, nx);       % Preallocate matrix Phi
for i = 1:np                  % Create matrix M using powers of matrix A
    phi(i*nx-nx+1:i*nx, :) = A^i; 
end

% Get prediction matrix N
gamma = zeros(nx*np, nu*np);    % Preallocate matrix Gamma
NN = zeros(nx*np, nu);          % Preallocate auxiliary matrix NN with zeros
for i = 1:np                    % Calculate auxiliary matrix NN
    NN(i*nx-nx+1:i*nx, :) = A^(i - 1) * B; 
end
for i = 1:np                % By shifting NN matrix create matrix Gamma
    gamma(i*nx-nx+1:end, i*nu-nu+1:i*nu) = NN(1:np*nx-(i - 1)*nx, :); 
end

% Get cost function matrices H,F
psi = kron(eye(np), R);

omega = zeros(np*nx, np*nx);  % Preallocate matrix Omega
for i = 1:np-1                % Create matrix Omega
    omega(i*nx-nx+1:i*nx, i*nx-nx+1:i*nx) = Q; 
end
omega(np*nx-nx+1:np*nx, np*nx-nx+1:np*nx) = P; % Add last diagonal element

% H = zeros(np*nu, np*nu);    % Initialise Hessian matrix with zeros
% F = zeros(np, nx);          % Initialise F matrix with matrix

H=2*(psi+gamma'*omega*gamma);
F=2*gamma'*omega*phi;
end