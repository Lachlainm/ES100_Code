clear all;

%distance in meters from geometric center to CoG

I = [0.42 0 0; 0 0.42 0; 0 0 0.642];
invI = I^-1;


zero_m = zeros(3,3);
Ident = eye(3,3); 

A = [zero_m zero_m Ident zero_m;
     zero_m zero_m zero_m Ident;
     zero_m zero_m zero_m zero_m; 
     zero_m zero_m zero_m zero_m]; 


g = 9.806; % force of gravity
m = 7.4;   % Mass of the quadcopter
z_hat = [0; 0; 1];
l = 0.053336;

g_term = invI*eye(3,3)*cross([0 0 -l]', [0 0 g]');

G = [0;0;0;0;0;0;0;0;g;g_term];

% Rotation matrix for the 45-degree rotation around the z-axis
R_45 = [cosd(45) -sind(45) 0; 
        sind(45)  cosd(45) 0; 
        0         0        1];

% Modified B matrix with the 45-degree rotation
B = [zero_m zero_m;
    zero_m zero_m;
    (1/m)*Ident zero_m;
     zero_m inv(I)]; 

C_m = [eye(6,6), zeros(6,12)]; 

D = 0; 

% Augment A matrix with integrator dynamics
A_augmented = [A, zeros(12, 6);
               C_m];

% Augment B matrix
B_augmented = [B; eye(6,6)];

At = [0 1 0;
     0 0 0;
     1 0 0];

Bt = [0;
      1/m; 
      0];

Brx = [0;
       invI(1,1);
       0];

Bry = [0;
        invI(2,2);
        0];

Brz = [0;
        invI(3,3);
        0];

B = [zeros(3) zeros(3);
     zeros(3) zeros(3);
     1/m * eye(3) zeros(3);
     zeros(3) inv(I)];


% dest_real  = 1.27278;
dest_real  = -0.5;
dest_imag  = 0;
desr_real  = -2;
desr_imag  = 0;
dest_er = -0.2;
desr_er = -1;
desired_polest = [dest_real + dest_imag*1i, dest_real - dest_imag*1i, dest_er];
desired_polesr = [desr_real + desr_imag*1i, desr_real - desr_imag*1i, desr_er];


Qt = [1/0.3^2, 0; 0, 1/0.3^2];
K_HLtx = acker(At,Bt, desired_polest);
K_HLty = acker(At,Bt, desired_polest);
K_HLtz = acker(At,Bt, desired_polest);


K_HLrx = acker(At,Brx, desired_polesr);
K_HLry = acker(At,Bry, desired_polesr);
K_HLrz = acker(At,Brz, desired_polesr);

K_HL_FF = [K_HLtx(1,1), zeros(1,5), K_HLtx(1,2), zeros(1,5), K_HLtx(1,3), zeros(1,5);
        0, K_HLty(1,1), zeros(1,5), K_HLty(1,2), zeros(1,5), K_HLty(1,3), zeros(1,4);
        0, 0, K_HLtz(1,1), zeros(1,5), K_HLtz(1,2), zeros(1,5), K_HLtz(1,3), zeros(1,3);
        zeros(1,3), K_HLrx(1,1), zeros(1,5), K_HLrx(1,2), zeros(1,5), K_HLrx(1,3), zeros(1,2);
        zeros(1,4), K_HLry(1,1), zeros(1,5), K_HLry(1,2), zeros(1,5), K_HLry(1,3), zeros(1,1);
        zeros(1,5), K_HLrz(1,1), zeros(1,5), K_HLrz(1,2), zeros(1,5), K_HLrz(1,3)];



max_pos_error_XY = 0.1;
max_pos_error_Z = 0.1;
max_angle_error_PR = deg2rad(100);
max_angle_error_Y = pi;
max_vel_error = 4;
max_angle_vel_error_PR = 0.12;
max_angle_vel_error_Y = 4;
max_pos_int_error = 8;
max_angle_int_error_PR = 2;
max_angle_int_error_Y = max_angle_error_Y;

Q = diag([ones(1,2)*1/max_pos_error_XY^2, 1/max_pos_error_Z^2, ones(1,2)*1/max_angle_error_PR^2, ...
    1/max_angle_error_Y^2, ones(1,3)*1/max_vel_error^2, ...
    ones(1,2)*1/max_angle_vel_error_PR^2, 1/max_angle_vel_error_Y^2, ...
    ones(1,3)*1/max_pos_int_error^2, ones(1,2)*1/max_angle_int_error_PR^2, ...
    1/max_angle_int_error_Y^2]);

max_force  = 0.4;
max_moment = 0.5;

% weight matrix for the inputs: prioritize less that state
R = diag([1/max_force^2*ones(1,3),1/max_moment^2*ones(1,3)]);

% Compute the LQR gain K
% K_HL = inv(R) * B_augmented' * P;
K_HL_manual = lqr(A_augmented, B_augmented, Q, R);

K_HL_manual = round(K_HL_manual, 6);

sig_delay = 1; 

sample_rate = 0.0025;

B_t = pinv(B);
sys = ss(A_augmented, B_augmented, eye(18), D);



 %Autonomous
max_pos_error_XY = 0.5;
max_pos_error_Z = 0.1;
max_angle_error_PR = 100*pi/180;
max_angle_error_Y = pi;
max_vel_error = 5;
max_angle_vel_error_PR = 0.12;
max_angle_vel_error_Y = 4;
max_pos_int_error = 8;
max_angle_int_error_PR = 2;
max_angle_int_error_Y = max_angle_error_Y;

Q = diag([ones(1,2)*1/max_pos_error_XY^2, 1/max_pos_error_Z^2, ones(1,2)*1/max_angle_error_PR^2, ...
    1/max_angle_error_Y^2, ones(1,3)*1/max_vel_error^2, ...
    ones(1,2)*1/max_angle_vel_error_PR^2, 1/max_angle_vel_error_Y^2, ...
    ones(1,3)*1/max_pos_int_error^2, ones(1,2)*1/max_angle_int_error_PR^2, ...
    1/max_angle_int_error_Y^2]);

max_force  = 4;
max_moment = 0.5;

% weight matrix for the inputs: prioritize less that state
R = diag([1/max_force^2*ones(1,3),1/max_moment^2*ones(1,3)]);

% Solve the Riccati equation for P using the 'care' function

% Compute the LQR gain K
% K_HL = inv(R) * B_augmented' * P;
K_HL = lqr(A_augmented, B_augmented, Q, R);

K_HL = round(K_HL, 6);
% A_single = [0 1 0;
%              0 0 0;
%              1 0 0];
% 
% B_single = [0;1/m;0];
% C_single = eye(3);
% D_single = 0;
% 
% [~, L_Kalman, ~] = kalman(ss(A_single, B_single, C_single, D_single), 0, 0.00000784*eye(3));


function K = acker(A, B, poles)
    char_poly = eye(3); 
    for i=1:length(poles)
        char_poly = char_poly*(A-poles(i)*eye(3));
    end
    P = [B, A*B, A^2*B];
    K = [0 0 1]*inv(P)*char_poly;
end

