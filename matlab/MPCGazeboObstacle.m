% All units are according to the SI unit convention (meter, second, radian)

clear all
close all
clc

% CasADi v3.4.5
% addpath('C:\Users\mehre\OneDrive\Desktop\CasADi\casadi-windows-matlabR2016a-v3.4.5')
% CasADi v3.5.5
addpath('/home/vedant/casadi-3.6.0-linux64-matlab2018b/')

import casadi.*


T = 0.2; % sampling time 
N = 100; % prediction horizon
% rob_diam = 0.178; % for turtlebot3 
rob_diam = 0.178; % robot diameter
v_max = 0.2; v_min = 0; % max, min velocity
omega_max = 0.75; omega_min = -0.75; % min, max angular velocity.

x = SX.sym('x'); y = SX.sym('y'); theta = SX.sym('theta'); % definition of SX variables (casadi's symbolic variables) for states
states = [x;y;theta]; n_states = length(states); % created a vector for states, and n_states to maintain the number of state variables (3 here)

v = SX.sym('v'); omega = SX.sym('omega'); % SX definitions for v and omega, i.e, our conrtols
controls = [v;omega]; n_controls = length(controls); % created a controls vector, n_controls is the number of controls (2 here)
rhs = [v*cos(theta);v*sin(theta);omega]; % rhs of the kinematic model (velocity x component, velocity y component, angular velocity)

f = Function('f',{states,controls},{rhs}); % nonlinear mapping function f(x,u), takes states and controls as input and gives the system rhs as output
U = SX.sym('U',n_controls,N); % Decision variables (controls) (2xN)
P = SX.sym('P',n_states + n_states); % parameters (which include at the initial state of the robot and the reference state)

X = SX.sym('X',n_states,(N+1)); % A vector that represents the states over the optimization problem.

obj = 0; % Objective function
g = [];  % constraints vector

Q = zeros(3,3); Q(1,1) = 1;Q(2,2) = 5;Q(3,3) = 0.1; % weighing matrices (states)
R = zeros(2,2); R(1,1) = 0.5; R(2,2) = 0.05; % weighing matrices (controls)

% Build objective function and fill up the g (constraints) vector. The objective function will be given to the NLP solver which uses it to 
% calculate numerical solutions to the differential equation model of the system.

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

% Add constraints for collision avoidance (configure obstacle)
obs_x = 0.5; 
obs_y = 0.25; 
obs_diam = 0.05; 

for k = 1:N+1   % calculate constraints for preventing collisions
    g = [g ; -sqrt((X(1,k)-obs_x)^2+(X(2,k)-obs_y)^2) + (rob_diam/2 + obs_diam/2)];
end

% make the decision variable one column vector for passing to the nlp solver. Optimization variables include both x and u as we are using the multiple shooting method.
OPT_variables = [reshape(X,3*(N+1),1);reshape(U,2*N,1)];

nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

% configure the nlp solver

opts = struct;
opts.ipopt.max_iter = 2000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

% create the solver object

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

% setup constraints of the problem

args = struct;
args.lbg(1:3*(N+1)) = 0; % equality constraints
args.ubg(1:3*(N+1)) = 0; % equality constraints

args.lbg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = -inf; % inequality constraints
args.ubg(3*(N+1)+1 : 3*(N+1)+ (N+1)) = 0; % inequality constraints

args.lbx(1:3:3*(N+1),1) = -2; %state x lower bound
args.ubx(1:3:3*(N+1),1) = 5; %state x upper bound
args.lbx(2:3:3*(N+1),1) = -2; %state y lower bound
args.ubx(2:3:3*(N+1),1) = 0.6; %state y upper bound
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

node = ros.Node('/test1'); % create ros node 
pub_vel = ros.Publisher(node,'/cmd_vel','geometry_msgs/Twist','DataFormat','struct'); % create publisher (trainsmitter) object in ros
sub_pos = ros.Subscriber(node,'/odom','nav_msgs/Odometry','DataFormat','struct'); % create subscriber (reciever) object in ros
velAndOhmega = rosmessage(pub_vel); % set up ros message for transmission

t0 = 0;
x0 = [0 ; 0 ; 0.0];    % initial condition, we keep updating x0 in the MPC loop to store the current state
xs = [1 ; 0.5 ; 0.0]; % Reference posture.


xx(:,1) = x0; % xx contains the history of states
t(1) = t0;

u0 = zeros(N,2);        % two control inputs, N times, thus a Nx2 vector
X0 = repmat(x0,1,N+1)'; % initialization of the states decision variables

sim_tim = 20; % Maximum simulation time

% Start MPC
% initialize counter loop and vectors to store states and control actions
mpciter = 0; 
xx1 = []; 
u_cl=[];

% the main simulaton loop... it works as long as the error is greater
% than 10^-6 and the number of mpc steps is less than its maximum
% value.
tic
% run while loop till current state is sufficiently close to reference state or maximimum iteration limit is reached
% the proximity of the current state and the simulation state is calculated by calculating the error between x0 and xs using the norm function
while(norm((x0-xs),2) > 5e-2 && mpciter < 100000) 

    args.p   = [x0;xs]; % set the values of the parameters vector (start and end condition for our solver)
    args.x0  = [reshape(X0',3*(N+1),1);reshape(u0',2*N,1)]; % initial value of the optimization variables (we reshape u0 into a 1 dimensional vector so as to pass it to the solver)
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
        'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);  
    u = reshape(full(sol.x(3*(N+1)+1:end))',2,N)'; % get controls only from the solution. sol.x contains the optimal solution for the described control problem. 
    % Since we redefine sol with a new x0 value every iteration, our optimal control vector gets changed every interation
    xx1(:,1:3,mpciter+1)= reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY

    a = 5 + 5;

    pause(0.5) % pause program, giving time for the turtlebot to send its location and orientation to our program using sub_pos
    quat = [sub_pos.LatestMessage.Pose.Pose.Orientation.W sub_pos.LatestMessage.Pose.Pose.Orientation.X sub_pos.LatestMessage.Pose.Pose.Orientation.Y sub_pos.LatestMessage.Pose.Pose.Orientation.Z];
    eulYaw = quat2eul(quat);

    % update our current state using the feedback recieved from the bot regarding its location and orientation

    x0 = [sub_pos.LatestMessage.Pose.Pose.Position.X; sub_pos.LatestMessage.Pose.Pose.Position.Y; eulYaw(1)];

    u_cl= [u_cl ; u(1,:)]; % store the optimal control actions for this timestep (1st row of our u vector)

    t(mpciter+1) = t0;
    y = 1;
    
    
    % Apply the control and shift the solution

    % One thing to note here is the difference between u and u0. When we defined our solver, we used u0 as the initialization matrix for our controls.
    % Since we have already made an optimal solution prediction which is stored in u, we store that prediction in u0 so as to initialize our solver with the new
    % optimal controls in every iteration. In short, we use u0 to propogate the predicted optimal control to the next iteration of MPC. 

    % The shift function will apply the first row of controls that have been predicted, and initalize u0 with the most optimal solution for the current state.
    % Specifics of the shift function can be found in the shift.m file

    [t0, x0, u0] = shift(T, t0, x0, u,f);

    % since u0 contains the optimal prediction for the current state, we extract the two controls in the first row of u0 and use it to initialize velAndOhmega,
    % which is our publisher's message.
    
    vel = u0(1,1);
    ohm = u0(1, 2);
    velAndOhmega.Linear.X = vel;
    velAndOhmega.Angular.Z = ohm;

    % After initializing velAndOhmega, we use the send function to transmit this message to the bot.

    send(pub_vel, velAndOhmega)
    
    xx(:,mpciter+2) = x0;
    X0 = reshape(full(sol.x(1:3*(N+1)))',3,N+1)'; % get solution TRAJECTORY

    % Shift trajectory to initialize the next step
    X0 = [X0(2:end,:);X0(end,:)];
    mpciter
    mpciter = mpciter + 1;
end;
toc
% Once our objectiive is reached, or once our simulation has timed out, we send a message to the bot to stop moving, but setting v and omega as 0.
velAndOhmega.Linear.X = 0;
velAndOhmega.Angular.Z = 0;
send(pub_vel, velAndOhmega)

ss_error = norm((x0-xs),2)

% clear the ros workspace so as to create fresh nodes, subscribers and publishers everytime the program is run.

clear('pub_vel','sub_pos','node')

Draw_MPC_PS_Obstacles (t,xx,xx1,u_cl,xs,N,rob_diam,obs_x,obs_y,obs_diam)