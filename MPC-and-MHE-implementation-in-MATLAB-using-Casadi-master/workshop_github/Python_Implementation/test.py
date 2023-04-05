import casadi as ca
from casadi import *

x = ca.SX.sym('w')
obj = 10 - ca.sqrt(x) 
OPT_variables = x
g = []
p = []
nlp_prob = {
    'f': obj,
    'x': OPT_variables,
    'g': g,
    'p': p
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

args = {
    'lbg': -ca.inf,  # constraints lower bound
    'ubg': ca.inf,  # constraints upper bound
    'lbx': 0,
    'ubx': ca.inf,
    'p': [],
    'x0': 0.5
}

sol = solver(
            x0=args['x0'],
            lbx=args['lbx'],
            ubx=args['ubx'],
            lbg=args['lbg'],
            ubg=args['ubg'],
            p=args['p']
        )

print("x value: " + str(sol['x']) + "\n" + "function value: " + str(sol['f']))

