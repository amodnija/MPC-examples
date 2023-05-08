from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import matplotlib.pyplot as plt
from simulation_code import simulate
from math import *

# setting matrix_weights' variables (in cost function, against reference values)
Q_x = 1
Q_y = 5
Q_theta = 0.1

R1 = 0.5              #weights for U
R2 = 0.05

step_horizon = 0.2  # time between steps in seconds (sampling time)
N = 14              # number of look ahead steps (prediction horizon)
rob_diam = 0.3      # diameter of the robot
wheel_radius = 1    # wheel radius
Lx = 0.3            # L in J Matrix (half robot x-axis length)
Ly = 0.3            # l in J Matrix (half robot y-axis length)
sim_time = 20       # simulation time

# specs-> init and target states
x_init = 0
y_init = 0
theta_init = 0.0
x_target = 1.5
y_target = 1.5
theta_target = 0.0

#velocity constraints (theta is already bound)
v_max = 0.6
v_min = -0.6
omega_min = -pi/4
omega_max = pi/4
def shift_timestep(step_horizon, t0, state_init, u, f):
    f_value = f(state_init, u[:, 0])
    next_state = ca.DM.full(state_init + (step_horizon * f_value))

    t0 = t0 + step_horizon
    u0 = ca.horzcat(
        u[:, 1:],
        ca.reshape(u[:, -1], -1, 1)
    )

    return t0, next_state, u0


def DM2Arr(dm):
    return np.array(dm.full())


# state symbolic variables
x = ca.SX.sym('x')
y = ca.SX.sym('y')
theta = ca.SX.sym('theta')
states = ca.vertcat(
    x,
    y,
    theta
)
n_states = states.numel()

# control symbolic variables
vel = ca.SX.sym('vel')
omega = ca.SX.sym('omega')
controls = ca.vertcat(
    vel,
    omega,
)
n_controls = controls.numel()

# matrix containing all states over all time steps +1 (each column is a state vector)
X = ca.SX.sym('X', n_states, N + 1) #sym returns a static instance of SX (symbolic exp matrix)

# matrix containing all control actions over all time steps (each column is an action vector)
U = ca.SX.sym('U', n_controls, N) #optimization variables 

# column vector for storing initial state and target state
P = ca.SX.sym('P', n_states + n_states)

# state weights matrix (Q_X, Q_Y, Q_THETA)
Q = ca.diagcat(Q_x, Q_y, Q_theta)

# controls weights matrix
R = ca.diagcat(R1, R2)

# # discretization model (e.g. x2 = f(x1, v, t) = x1 + v * dt)
# rot_3d_z = ca.vertcat(
#     ca.horzcat(cos(theta), -sin(theta), 0),
#     ca.horzcat(sin(theta),  cos(theta), 0),
#     ca.horzcat(         0,           0, 1)
# )
# # Mecanum wheel transfer function which can be found here: 
# # https://www.researchgate.net/publication/334319114_Model_Predictive_Control_for_a_Mecanum-wheeled_robot_in_Dynamical_Environments
# J = (wheel_radius/4) * ca.DM([
#     [         1,         1,          1,         1],
#     [        -1,         1,          1,        -1],
#     [-1/(Lx+Ly), 1/(Lx+Ly), -1/(Lx+Ly), 1/(Lx+Ly)]
# ])
# RHS = states + J @ controls * step_horizon  # Euler discretization
RHS = ca.vertcat(
    vel*cos(theta),
    vel*sin(theta),
    omega
)  
#@ is matrix mul, @x represents sym consts

# maps controls from [va, vb, vc, vd].T to [vx, vy, omega].T (maps states and controls to RHS)
f = ca.Function('f', [states, controls], [RHS])


cost_fn = 0  # cost function
g = X[:, 0] - P[:n_states]  # constraints in the equation (initial condition setting using P)


# runge kutta
for k in range(N):
    st = X[:, k]
    con = U[:, k]
    #cost function
    cost_fn = cost_fn \
        + (st - P[n_states:]).T @ Q @ (st - P[n_states:]) \
        + con.T @ R @ con
    st_next = X[:, k+1]
    k1 = f(st, con)
    # k2 = f(st + step_horizon/2*k1, con)
    # k3 = f(st + step_horizon/2*k2, con)
    # k4 = f(st + step_horizon * k3, con)
    # st_next_RK4 = st + (step_horizon / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    st_next_euler = st + step_horizon*k1
    g = ca.vertcat(g, st_next - st_next_euler)

obs_x = 0.5
obs_y = 0.5
obs_diam = 0.3

for k in range(N+1):
    tmp = -ca.sqrt((X[0,k]-obs_x)**2 + (X[1,k]-obs_y)**2)+(rob_diam/2 + obs_diam/2)
    g = ca.vertcat(g, tmp)

OPT_variables = ca.vertcat(
    X.reshape((-1, 1)),   # Example: 3x11 ---> 33x1 where 3=states, 11=N+1
    U.reshape((-1, 1))
)

nlp_prob = {
    'f': cost_fn,
    'x': OPT_variables,
    'g': g,
    'p': P
}

opts = {
    'ipopt': {
        'max_iter': 100,
        'print_level': 0,
        'acceptable_tol': 1e-8,
        'acceptable_obj_change_tol': 1e-6
    },
    'print_time': 0
}

solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

lbx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
ubx = ca.DM.zeros((n_states*(N+1) + n_controls*N, 1))
lbg = ca.DM.zeros((n_states*(N+1)+(N+1), 1))
ubg = ca.DM.zeros((n_states*(N+1)+(N+1), 1))

lbg[n_states*(N+1):] = -inf
ubg[n_states*(N+1):] = 0
lbx[0: n_states*(N+1): n_states] = -2     # X lower bound [0,0+3,0+6,...till 0+3*(N+1)]
lbx[1: n_states*(N+1): n_states] = -2     # Y lower bound
lbx[2: n_states*(N+1): n_states] = -ca.inf     # theta lower bound

ubx[0: n_states*(N+1): n_states] = 2      # X upper bound
ubx[1: n_states*(N+1): n_states] = 2      # Y upper bound
ubx[2: n_states*(N+1): n_states] = ca.inf      # theta upper bound

lbx[n_states*(N+1)::n_states] = v_min                  # v lower bound for all V
ubx[n_states*(N+1)::n_states] = v_max                  # v upper bound for all V
lbx[n_states*(N+1)+1::n_states] = omega_min                  # v lower bound for all V
ubx[n_states*(N+1)+1::n_states] = omega_max 

args = {
    'lbg': lbg,
    'ubg': ubg,
    'lbx': lbx,
    'ubx': ubx
}

t0 = 0
state_init = ca.DM([x_init, y_init, theta_init])        # initial state
state_target = ca.DM([x_target, y_target, theta_target])  # target state

# xx = DM(state_init)
t = ca.DM(t0)

u0 = ca.DM.zeros((n_controls, N))  # initial control
X0 = ca.repmat(state_init, 1, N+1)         # initial state full


mpc_iter = 0
cat_states = DM2Arr(X0)
cat_controls = DM2Arr(u0[:, 0])
times = np.array([[0]])


###############################################################################

if __name__ == '__main__':
    main_loop = time()  # return time in sec
    while (ca.norm_2(state_init - state_target) > 1e-2) and (mpc_iter * step_horizon < sim_time):
        t1 = time()
        args['p'] = ca.vertcat(
            state_init,    # current state
            state_target   # target state
        )
        # optimization variable current state
        args['x0'] = ca.vertcat(
            ca.reshape(X0, n_states*(N+1), 1),
            ca.reshape(u0, n_controls*N, 1)
        )

        sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

        u = ca.reshape(sol['x'][n_states * (N + 1):], n_controls, N)
        X0 = ca.reshape(sol['x'][: n_states * (N+1)], n_states, N+1)

        cat_states = np.dstack((
            cat_states,
            DM2Arr(X0)
        ))

        cat_controls = np.vstack((
            cat_controls,
            DM2Arr(u[:, 0])
        ))
        t = np.vstack((
            t,
            t0
        ))

        t0, state_init, u0 = shift_timestep(step_horizon, t0, state_init, u, f)

        # print(X0)
        X0 = ca.horzcat(
            X0[:, 1:],
            ca.reshape(X0[:, -1], -1, 1)
        )

        # xx ...
        t2 = time()
        print(mpc_iter)
        print(t2-t1)
        times = np.vstack((
            times,
            t2-t1
        ))

        mpc_iter = mpc_iter + 1

    main_loop_time = time()
    ss_error = ca.norm_2(state_init - state_target)

    print('\n\n')
    print('Total time: ', main_loop_time - main_loop)
    print('avg iteration time: ', np.array(times).mean() * 1000, 'ms')
    print('final error: ', ss_error)

    # simulate
    simulate(cat_states, cat_controls, times, step_horizon, N,
             np.array([x_init, y_init, theta_init, x_target, y_target, theta_target]), save=False)







