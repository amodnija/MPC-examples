% first casadi test for mpc fpr mobile robots
clear all
close all
clc

% CasADi v3.4.5
% addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.4.5')
% CasADi v3.5.5
addpath('/home/amit/Dropbox/Documents/casadi-linux-matlabR2014b-v3.5.5')

import casadi.*

T = 0.2; %[s]
N = 14; % prediction horizon
% rob_diam = 0.178; % for turtlebot3 
rob_diam = 0.72; % for innok

v_max = 0.2; v_min = -v_max;
omega_max = 2.84; omega_min = -omega_max;

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta');
states = [x;y;theta]; n_states = length(states);

v = SX.sym('v'); omega = SX.sym('omega');
controls = [v;omega]; n_controls = length(controls);
rhs = [v*cos(theta);v*sin(theta);omega]; % system r.h.s

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u)
U = SX.sym('U',n_controls,N); % Decision variables (controls)
P = SX.sym('P',n_states + n_states);
% parameters (which include at the initial state of the robot and the reference state)

X = SX.sym('X',n_states,(N+1));
% A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

st  = X(:,1); % initial state
g = [g;st-P(1:3)]; % initial condition constraints
for k = 1:N
    st = X(:,k);  con = U(:,k);
    obj = obj+(st-P(4:6))'*Q*(st-P(4:6)) + con'*R*con; % calculate obj
    st_next = X(:,k+1);
    f_value = f(st,con);
    st_next_euler = st+ (T*f_value);
    g = [g;st_next-st_next_euler]; % compute constraints
end
% Add constraints for collision avoidance
obs_x = 1; % meters
obs_y = 1.4; % meters
obs_diam = 0.3; % meters
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; -sqrt((X(1,k)-obs_x)^2+(X(2,k)-obs_y)^2) + (rob_diam/2 + obs_diam/2)];
end
% make the decision variable one column  vector
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
args.lbg(1:3*(N+1)) = 0; % equality constraints
args.ubg(1:3*(N+1)) = 0; % equality constraints

args.lbg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = -inf; % inequality constraints
args.ubg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = 0; % inequality constraints

args.lbx(1:3:3*(N+1),1) = -2; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 5; %state x upper bound
args.lbx(2:3:3*(N+1),1) = -2; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 5; %state y upper bound
args.lbx(3:3:3*(N+1),1) = -inf; %state theta lower bound
args.ubx(3:3:3*(N+1),1) = inf; %state theta upper bound

args.lbx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_min; %v lower bound
args.ubx(3*(N+1)+1:2:3*(N+1)+2*N,1) = v_max; %v upper bound
args.lbx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_min; %omega lower bound
args.ubx(3*(N+1)+2:2:3*(N+1)+2*N,1) = omega_max; %omega upper bound
%----------------------------------------------
% ALL OF THE ABOVE IS JUST A PROBLEM SET UP


% THE SIMULATION LOOP SHOULD START FROM HERE
%-------------------------------------------
node = ros.Node('/test0');
pub_vel = ros.Publisher(node,'/cmd_vel','geometry_msgs/Twist','DataFormat','struct');
sub_pos = ros.Subscriber(node,'/odom','nav_msgs/Odometry','DataFormat','struct');
velAndOhmega = rosmessage(pub_vel);

t0 = 0;
x0 = [0 ; 0 ; 0.0];    % initial condition.
xs = [4 ; 4 ; 0.0]; % Reference posture.

xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs for each robot
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 20; % Maximum simulation time

% Start MPC
mpciter = 0;
xx1 = [];
u_cl=[];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
tic
while(norm((x0-xs),2) > 5e-2 && mpciter < 100000)
    args.p   = [x0;xs]; % set the values of the parameters vector
    % initial value of the optimization variables
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)];
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    a = 5 + 5;
    quat1 = sub_pos.LatestMessage.Pose.Pose.Orientation.X;
    quat = [sub_pos.LatestMessage.Pose.Pose.Orientation.W sub_pos.LatestMessage.Pose.Pose.Orientation.X sub_pos.LatestMessage.Pose.Pose.Orientation.Y sub_pos.LatestMessage.Pose.Pose.Orientation.Z];
    eulYaw = quat2eul(quat);
    x0 = [sub_pos.LatestMessage.Pose.Pose.Position.X; sub_pos.LatestMessage.Pose.Pose.Position.Y; eulYaw(1)];

    u_cl= [u_cl ; u(1,:)];
    t(mpciter+1) = t0;
    y = 1;
    
    
    % Apply the control and shift the solution
    [t0, x0, u0] = shift(T, t0, x0, u,f);
    
    vel = u0(1,1);
    ohm = u0(1, 2);
    velAndOhmega.Linear.X = vel;
    velAndOhmega.Angular.Z = ohm;
    send(pub_vel, velAndOhmega)
    
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY
    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter
    mpciter = mpciter + 1;
end;
toc
%

ss_error = norm((x0-xs),2)
Draw_MPC_PS_Obstacles (t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam)