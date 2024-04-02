dt = 0.1; %sec
end_time = 70; % sec
t_vec = 0:dt:end_time;

backup  = 0.30;     %m
FFforce = 4.5;      %N
FFforceZ = 0.0;
wall    = 2.495;     %m

% X Y Z, R P Y, U V W
% prod_waypoints = [
%     ... % takeoff
%     0 0 0 0 0 0 0 0 0; 0 0 0.5 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0; 0 0 1.5 0 0 0 0 0 0; 0 0 2 0 0 0 0 0 0;
%     ... % turn towards wall;
%     0 0 2 0 0 0 0 0 0; 0 0 2 0 0 10 0 0 0; 0 0 2 0 0 20 0 0 0; 0 0 2 0 0 30 0 0 0; 0 0 2 0 0 40 0 0 0; 
%     ...
%     0 0 2 0 0 50 0 0 0; 0 0 2 0 0 60 0 0 0; 0 0 2 0 0 70 0 0 0; 0 0 2 0 0 80 0 0 0; 0 0 2 0 0 90 0 0 0;
%     ... % move horizontally to start point
%     1 0 2 0 0 90 0 0 0; 2 0 2 0 0 90 0 0 0;
%     ... % approach wall
%     2 wall*0.25 2 0 0 90 0 0 0; 2 wall*0.5 2 0 0 90 0 0 0; 2 wall*0.75 2 0 0 90 0 0 0; 2 wall 2 0 0 90 0 0 0;
%     ... % P
%     2 wall 2 0 0 90 0 FFforce 0; 2 wall 2.5 0 0 90 0 FFforce 0; 1.65 wall 2.5 0 0 90 0 FFforce 0; 1.65 wall 2.25 0 0 90 0 FFforce 0; 2 wall 2.25 0 0 90 0 FFforce 0; 2 wall 2 0 0 90 0 FFforce 0; 2 wall 2 0 0 90 0 FFforce 0; 2 wall 2 0 0 90 0 0 0; 
%     ... % space
%     2 wall - backup 2 0 0 90 0 0 0; 1.5 wall - backup 2 0 0 90 0 0 0; 
%     ... % R
%     1.5 wall 2 0 0 90 0 0 0; 1.5 wall 2 0 0 90 0 FFforce 0; 1.5 wall 2.5 0 0 90 0 FFforce 0; 1.15 wall 2.5 0 0 90 0 FFforce 0; 1.15 wall 2.25 0 0 90 0 FFforce 0; 1.5 wall 2.25 0 0 90 0 FFforce 0; 1.15 wall 2 0 0 90 0 FFforce 0; 1.15 wall 2 0 0 90 0 FFforce 0; 1.15 wall 2 0 0 90 0 0 0;
%     ... % space
%     1.15 wall - backup 2 0 0 90 0 0 0; 1 wall - backup 2 0 0 90 0 0 0;
%     ... % O
%     1 wall 2 0 0 90 0 0 0; 1 wall 2 0 0 90 0 FFforce 0; 1 wall 2.5 0 0 90 0 FFforce 0; 0.65 wall 2.5 0 0 90 0 FFforce 0; 0.65 wall 2 0 0 90 0 FFforce 0; 1 wall 2 0 0 90 0 FFforce 0; 1 wall 2 0 0 90 0 FFforce 0; 1 wall 2 0 0 90 0 0 0;
%     ... % space
%     1.0 wall - backup 2 0 0 90 0 0 0; 0.5 wall - backup 2 0 0 90 0 0 0;
%     ... % D
%     0.5 wall 2 0 0 90 0 0 0; 0.5 wall 2 0 0 90 0 FFforce 0; 0.5 wall 2.5 0 0 90 0 FFforce 0; 0.15 wall 2.25 0 0 90 0 FFforce 0; 0.5 wall 2 0 0 90 0 FFforce 0; 0.5 wall 2 0 0 90 0 FFforce 0;  0.5 wall 2 0 0 90 0 0 0;
%     ... % go home
%     0.5 wall-backup 2 0 0 90 0 0 0; 0 wall - backup 2 0 0 90 0 0 0;
%     ... % back away from wall
%     0 wall*0.75 2 0 0 90 0 0 0; 0 wall*0.5 2 0 0 90 0 0 0; 0 wall*0.25 2 0 0 90 0 0 0; 0 0 2 0 0 90 0 0 0;
%     ... % land
%     0 0 1.75 0 0 90 0 0 0; 0 0 1.5 0 0 90 0 0 0; 0 0 1.25 0 0 90 0 0 0; 0 0 1.0 0 0 90 0 0 0; 0 0 0.75 0 0 90 0 0 0; 0 0 0.5 0 0 90 0 0 0; 0 0 0.25 0 0 90 0 0 0; 0 0 0 0 0 90 0 0 0;
% ];

% prod_waypoints = [
%      ... % takeoff
%      0 0 0 0 0 0 0 0 0;  0 0 1 0 0 0 0 0 0;
%      ... % approach wall
%      0 wall*0.95 1 0 0 0 0 0 0;
%      ... % apply force
%      0 wall 1 0 0 0 0 FFforce FFforceZ; 0 wall 1 0 0 0 0 FFforce FFforceZ;
%      ... % translate
%      -0.4 wall 1 0 0 0 0 FFforce FFforceZ;
%      ... % back up away from wall
%      -0.5 wall*0.5 1 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
%      ... % land
%      0 0 0 0 0 0 0 0 0;
% ];

% prod_waypoints = [
%      ... % takeoff
%      0 0 0 0 0 0 0 0 0;  0 0 1 0 0 0 0 0 0;
%      ... % approach wall
%      0 wall*0.5 1 0 0 0 0 0 0; 0 wall*0.95 1 0 0 0 0 0 0;
%      ... % apply force
%      0 wall 1 0 0 0 0 FFforce FFforceZ; 0 wall 1 0 0 0 0 FFforce FFforceZ;
%      % ... % up
%      % 0 wall 1.15 0 0 0 0 FFforce FFforceZ;
%      % ... % right
%      % -0.2 wall 1.15 0 0 0 0 FFforce FFforceZ;
%      ... % back up away from wall
%      % -0.2 wall*0.5 1.15 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
%      0.0 wall*0.5 1.0 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
%      ... % land
%      0 0 0 0 0 0 0 0 0;
% ];

prod_waypoints = [
     ... % takeoff
     0 0 0 0 0 0 0 0 0;  0 0 1 0 0 0 0 0 0;
     ... % hover change
     .5 0 1 0 0 0 0 0 0; 1 0 1 0 0 0 0 0 0; 1 0.5 1 0 0 0 0 0 0;
     ... % change back
     1 1 1 0 0 0 0 0 0; 0.75 0.75 1 0 0 0 0 0 0; 0.5 0.5 1 0 0 0 0 0 0; 0 0 1 0 0 0 0 0 0;
     ... % land
     0 0 0 0 0 0 0 0 0;
];

% Scale the message to fit in the X-Z plane
scale_factor = 1.0;
prod_waypoints = prod_waypoints * scale_factor;
start_time = 0;
% Interpolate the waypoints for the full trajectory

time_points = [0; 0.1*end_time; 0.4*end_time; 0.415*end_time; 0.5*end_time; 0.6*end_time; 0.7*end_time; 0.8*end_time; 0.9*end_time; end_time];

interp_traj = interp1(linspace(0, end_time, size(prod_waypoints,1)), prod_waypoints, t_vec(t_vec < end_time), 'linear', 'extrap');

traj = zeros(9, length(t_vec));
for i = 1:length(t_vec(1:length(t_vec)-1))
    t = t_vec(i);
    y = interp_traj(i, 2); % Fixed Y-coordinate as the message is in X-Z plane
    z = interp_traj(i, 3);
    x = interp_traj(i, 1); 

    rol = interp_traj(i, 4); % Fixed Y-coordinate as the message is in X-Z plane
    pit = interp_traj(i, 5);
    yaw = interp_traj(i, 6); 

    fx = interp_traj(i, 7); 
    fy = interp_traj(i, 8); 
    fz = interp_traj(i, 9); 

    traj(:,i) = [x; y; z; rol; pit; yaw; fx; fy; fz]; 
   
end

traj_ts = timeseries(traj, t_vec);
csvwrite('trans_demo.csv', traj');
