%Admittance controller

% Apparent Inertia
m_ad = 0.9; %[kg]
M = eye(3)*1;

% Desired Damping 
d_ad = 0.9; % Damping Coefficient
Damp = eye(3)*d_ad;

% Desired Stiffness
stiffness_coeff = 0.9;
K = stiffness_coeff*eye(3);

A_ad = [zeros(3,3), eye(3); 
        -inv(M)*K, -inv(M)*Damp];
    
B_ad = [zeros(3,3), zeros(3,3);
        inv(M)*K, inv(M)];
    
C_ad = [eye(3), zeros(3,3)];

D_ad = zeros(3,6);


