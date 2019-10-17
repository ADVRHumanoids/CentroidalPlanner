#!/usr/bin/env python
from casadi import *
import matlogger2.matlogger as matl
import centroidal_planner.pycpl_casadi as cpl_cas
import rospy
import xbot_interface.config_options as cfg
import xbot_interface.xbot_interface as xbot

logger = matl.MatLogger2('/tmp/centauro_full_jumping_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

# get cartesio ros client
opt = cfg.ConfigOptions()
model = xbot.ModelInterface(opt)

urdf = rospy.get_param('robot_description')

fk_waist = cpl_cas.generate_forward_kin(urdf, 'pelvis')
FK_waist = Function.deserialize(fk_waist)

fk1 = cpl_cas.generate_forward_kin(urdf, 'wheel_1')
FK1 = Function.deserialize(fk1)

fk2 = cpl_cas.generate_forward_kin(urdf, 'wheel_2')
FK2 = Function.deserialize(fk2)

fk3 = cpl_cas.generate_forward_kin(urdf, 'wheel_3')
FK3 = Function.deserialize(fk3)

fk4 = cpl_cas.generate_forward_kin(urdf, 'wheel_4')
FK4 = Function.deserialize(fk4)

id_string = cpl_cas.generate_inv_dyn(urdf)
ID = Function.deserialize(id_string)

jac_waist = cpl_cas.generate_jacobian(urdf, 'pelvis')
Jac_waist = Function.deserialize(jac_waist)

jac_C1 = cpl_cas.generate_jacobian(urdf, 'wheel_1')
Jac_C1 = Function.deserialize(jac_C1)

jac_C2 = cpl_cas.generate_jacobian(urdf, 'wheel_2')
Jac_C2 = Function.deserialize(jac_C2)

jac_C3 = cpl_cas.generate_jacobian(urdf, 'wheel_3')
Jac_C3 = Function.deserialize(jac_C3)

jac_C4 = cpl_cas.generate_jacobian(urdf, 'wheel_4')
Jac_C4 = Function.deserialize(jac_C4)

tf = 1.  # Normalized time horizon
ns = 30  # number of shooting nodes

nc = 4  # number of contacts

# REDUCED model
DoF = 12
nq = DoF + 7  # number of DoFs - NB: 7 DoFs floating base (quaternions)
nv = nq - 1
nf = 3*nc

# FULL model
DoF_full = 35
nq_full = DoF_full + 7
nv_full = nq_full - 1

# Variables
q = SX.sym('q', nq)
qdot = SX.sym('qdot', nv)
qddot = SX.sym('qddot', nv)
f = SX.sym('f', nf)

# Bounds and initial guess  # TODO: get from robot
q_min = np.array([-1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -2.00, -2.07, -2.61799388, -2.48, -2.06, -2.61799388, -2.48, -2.06, -2.61799388, -2.00, -2.07, -2.61799388])
q_max = np.array([1.0,  1.0,  1.0,  1.0,  1.0,  1.0,  1.0, 2.48, 2.07, 2.61799388, 2.00, 2.09439510, 2.61799388, 2.00, 2.09439510, 2.61799388, 2.48, 2.07, 2.61799388])
q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -0.746874, -1.25409, -1.55576, 0.746874, 1.25409, 1.55576, 0.746874, 1.25409, 1.55576, -0.746874, -1.25409, -1.55576])

qdot_min = np.full((1, nv), -1000.)
qdot_max = np.full((1, nv), 1000.)
qdot_init = np.zeros_like(qdot_min)

qddot_min = np.full((1, nv), -1000.)
qddot_max = np.full((1, nv), 1000.)
qddot_init = np.zeros_like(qddot_min)

f_min = np.tile(np.array([-1000., -1000., -1000.]), 4)
f_max = np.tile(np.array([1000., 1000., 1000.]), 4)
f_init = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

x_min = np.append(q_min, qdot_min)
x_max = np.append(q_max, qdot_max)

x_init = np.append(q_init, qdot_init)

x0_min = x_init
x0_max = x_init

xf_min = np.append(q_min, np.zeros_like(qdot_min))
xf_max = np.append(q_max, np.zeros_like(qdot_min))

t_min = 0.05
t_max = 0.15

# Model equations
S = SX.zeros(3, 3)
S[0, 1] = -q[5]
S[0, 2] = q[4]
S[1, 0] = q[5]
S[1, 2] = -q[3]
S[2, 0] = -q[4]
S[2, 1] = q[3]

tmp1 = casadi.mtimes(0.5*(q[6]*SX.eye(3) - S), qdot[3:6])
tmp2 = -0.5*casadi.mtimes(q[3:6].T, qdot[3:6])

x = vertcat(q, qdot)
xdot = vertcat(qdot[0:3], tmp1, tmp2, qdot[6:nv], qddot)

nx = x.size1()

# Objective term
L = 0.01*dot(qddot, qddot)

# Runge-Kutta 4 integrator
f_RK = Function('f_RK', [x, qddot], [xdot, L])
X0 = MX.sym('X0', nx)
U = MX.sym('U', nv)
Time = MX.sym('Time', 1)
DT = Time
X = X0
Q = 0

k1, k1_q = f_RK(X, U)
k2, k2_q = f_RK(X + 0.5*DT*k1, U)
k3, k3_q = f_RK(X + DT / 2 * k2, U)
k4, k4_q = f_RK(X + DT * k3, U)
X = X + DT / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
Q = Q + DT / 6 * (k1_q + 2 * k2_q + 2 * k3_q + k4_q)

F_RK = Function('F_RK', [X0, U, Time], [X, Q], ['x0', 'p', 'time'], ['xf', 'qf'])

q_red = MX.sym('q_red', nq)
q_full = MX(Sparsity.dense(nq_full, 1))
q_full[0:7, 0] = q_red[0:7]
q_full[7:10, 0] = q_red[7:10]
q_full[10:12, 0] = [-0.301666, 0.746874]
q_full[12:15, 0] = q_red[10:13]
q_full[15:17, 0] = [0.301666, -0.746874]
q_full[17:20, 0] = q_red[13:16]
q_full[20:22, 0] = [0.301666, -0.746874]
q_full[22:25, 0] = q_red[16:19]
q_full[25:27, 0] = [-0.301666, 0.746874]
q_full[27, 0] = [3.56617e-13]
q_full[28:35, 0] = [0.520149, 0.320865, 0.274669, -2.23604, 0.0500815, -0.781461, -0.0567608]
q_full[35:42, 0] = [0.520149, -0.320865, -0.274669, -2.23604, -0.0500815, -0.781461, 0.0567608]

qdot_red = MX.sym('qdot_red', nv)
qdot_full = MX(Sparsity.dense(nv_full, 1))
qdot_full[0:6, 0] = qdot_red[0:6]
qdot_full[6:9, 0] = qdot_red[6:9]
qdot_full[9:11, 0] = [0.0, 0.0]
qdot_full[11:14, 0] = qdot_red[9:12]
qdot_full[14:16, 0] = [0.0, 0.0]
qdot_full[16:19, 0] = qdot_red[12:15]
qdot_full[19:21, 0] = [0.0, 0.0]
qdot_full[21:24, 0] = qdot_red[15:18]
qdot_full[24:26, 0] = [0.0, 0.0]
qdot_full[26, 0] = [0.0]
qdot_full[27:34, 0] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qdot_full[34:41, 0] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

qddot_red = MX.sym('qddot_red', nv)
qddot_full = MX(Sparsity.dense(nv_full, 1))
qddot_full[0:6, 0] = qddot_red[0:6]
qddot_full[6:9, 0] = qddot_red[6:9]
qddot_full[9:11, 0] = [0.0, 0.0]
qddot_full[11:14, 0] = qddot_red[9:12]
qddot_full[14:16, 0] = [0.0, 0.0]
qddot_full[16:19, 0] = qddot_red[12:15]
qddot_full[19:21, 0] = [0.0, 0.0]
qddot_full[21:24, 0] = qddot_red[15:18]
qddot_full[24:26, 0] = [0.0, 0.0]
qddot_full[26, 0] = [0.0]
qddot_full[27:34, 0] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qddot_full[34:41, 0] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

State_Extension = Function('State_Extension', [q_red, qdot_red, qddot_red], [q_full, qdot_full, qddot_full],
                           ['q_red', 'qdot_red', 'qddot_red'], ['q_full', 'qdot_full', 'qddot_full'])


qddot_red = MX.sym('qddot_red', nv)
qddot_full = MX(Sparsity.dense(nv_full, 1))
qddot_full[0:6, 0] = qddot_red[0:6]
qddot_full[6:9, 0] = qddot_red[6:9]
qddot_full[9:11, 0] = [0.0, 0.0]
qddot_full[11:14, 0] = qddot_red[9:12]
qddot_full[14:16, 0] = [0.0, 0.0]
qddot_full[16:19, 0] = qddot_red[12:15]
qddot_full[19:21, 0] = [0.0, 0.0]
qddot_full[21:24, 0] = qddot_red[15:18]
qddot_full[24:26, 0] = [0.0, 0.0]
qddot_full[26, 0] = [0.0]
qddot_full[27:34, 0] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
qddot_full[34:41, 0] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

tau_full = MX.sym('tau_full', nv_full)
tau_red = MX(Sparsity.dense(nv, 1))
tau_red[0:6, 0] = tau_full[0:6]
tau_red[6:9, 0] = tau_full[6:9]
tau_red[9:12, 0] = tau_full[11:14]
tau_red[12:15, 0] = tau_full[16:19]
tau_red[15:18, 0] = tau_full[21:24]

Torque_Reduction = Function('Torque_Reduction', [tau_full], [tau_red], ['tau_full'], ['tau_red'])

q_full_init = State_Extension(q_red=q_init, qdot_red=qdot_init, qddot_red=qddot_init)['q_full']

C1_pos_ground = FK1(q=q_full_init)['ee_pos']
C2_pos_ground = FK2(q=q_full_init)['ee_pos']
C3_pos_ground = FK3(q=q_full_init)['ee_pos']
C4_pos_ground = FK4(q=q_full_init)['ee_pos']

Waist_pos_init = FK_waist(q=q_full_init)['ee_pos']

Waist_pos_jump = FK_waist(q=q_full_init)['ee_pos']
Waist_pos_jump[2] += 0.3

lift_node = 10
touch_down_node = 20

# Start with an empty NLP
NV = nx*(ns+1) + (nv + nf)*ns + ns
V = MX.sym('V', NV)

# NLP vars bounds and init guess
v_min = []
v_max = []
v_init = []
g_min = []
g_max = []

# offset in v
offset = 0

# "Lift" initial conditions
X = []
Qddot = []
Force = []
Time = []

# Formulate the NLP
for k in range(ns):

    # State at k-th node
    X.append(V[offset:offset+nx])

    if k == 0:
        v_min += x0_min.tolist()
        v_max += x0_max.tolist()
    else:
        v_min += x_min.tolist()
        v_max += x_max.tolist()

    v_init += x_init.tolist()

    offset += nx

    # Control at k-th node
    Qddot.append(V[offset:offset+nv])

    v_min += qddot_min.tolist()
    v_max += qddot_max.tolist()

    v_init += qddot_init.tolist()

    offset += nv

    Force.append(V[offset:offset+nf])

    v_min += f_min.tolist()
    v_max += f_max.tolist()

    v_init += f_init.tolist()

    offset += nf

    Time.append(V[offset:offset+1])
    v_min += np.array([t_min]).tolist()
    v_max += np.array([t_max]).tolist()
    v_init += np.array([t_min]).tolist()

    offset += 1

# Final state
X.append(V[offset:offset+nx])

v_min += xf_min.tolist()
v_max += xf_max.tolist()

v_init += x_init.tolist()

offset += nx

assert offset == NV

# Create NLP
J = MX([0])
g = []

Waist_pos_hist = MX(Sparsity.dense(3, ns))
Waist_vel_hist = MX(Sparsity.dense(6, ns))
C1_history = MX(Sparsity.dense(3, ns))
C2_history = MX(Sparsity.dense(3, ns))
C3_history = MX(Sparsity.dense(3, ns))
C4_history = MX(Sparsity.dense(3, ns))
C1_vel_history = MX(Sparsity.dense(6, ns))
C2_vel_history = MX(Sparsity.dense(6, ns))
C3_vel_history = MX(Sparsity.dense(6, ns))
C4_vel_history = MX(Sparsity.dense(6, ns))
Fc1_history = MX(Sparsity.dense(3, ns))
Fc2_history = MX(Sparsity.dense(3, ns))
Fc3_history = MX(Sparsity.dense(3, ns))
Fc4_history = MX(Sparsity.dense(3, ns))
tau_u_history = MX(Sparsity.dense(6, ns))
tau_a_history = MX(Sparsity.dense(DoF, ns))
q_history = MX(Sparsity.dense(nq, ns))
qdot_history = MX(Sparsity.dense(nv, ns))
qddot_history = MX(Sparsity.dense(nv, ns))
h_history = MX(Sparsity.dense(1, ns))

Waist_pos = None
Waist_vel = None
C1_pos = None
C2_pos = None
C3_pos = None
C4_pos = None

C1_vel = None
C2_vel = None
C3_vel = None
C4_vel = None

mu = 1.0
mu_lin = mu/2.0*sqrt(2.0)

A_fr = np.zeros([5, 3])
A_fr[0, 0] = 1.0
A_fr[0, 2] = -mu_lin
A_fr[1, 0] = -1.0
A_fr[1, 2] = -mu_lin
A_fr[2, 1] = 1.0
A_fr[2, 2] = -mu_lin
A_fr[3, 1] = -1.0
A_fr[3, 2] = -mu_lin
A_fr[4, 2] = -1.0

R_wall = np.zeros([3, 3])
R_wall[0, 1] = -1.0
R_wall[1, 2] = -1.0
R_wall[2, 0] = 1.0

A_fr_R = mtimes(A_fr, R_wall)

for k in range(ns):

    integrator_out = F_RK(x0=X[k], p=Qddot[k], time=Time[k])

    Q_k = X[k][0:nq]
    Qdot_k = X[k][nq:nq + nv]

    Q_full = State_Extension(q_red=Q_k, qdot_red=Qdot_k, qddot_red=Qddot[k])['q_full']
    Qdot_full = State_Extension(q_red=Q_k, qdot_red=Qdot_k, qddot_red=Qddot[k])['qdot_full']
    Qddot_full = State_Extension(q_red=Q_k, qdot_red=Qdot_k, qddot_red=Qddot[k])['qddot_full']

    Waist_pos = FK_waist(q=Q_full)['ee_pos']
    C1_pos = FK1(q=Q_full)['ee_pos']
    C2_pos = FK2(q=Q_full)['ee_pos']
    C3_pos = FK3(q=Q_full)['ee_pos']
    C4_pos = FK4(q=Q_full)['ee_pos']

    Waist_jac = Jac_waist(q=Q_full)['J']
    C1_jac = Jac_C1(q=Q_full)['J']
    C2_jac = Jac_C2(q=Q_full)['J']
    C3_jac = Jac_C3(q=Q_full)['J']
    C4_jac = Jac_C4(q=Q_full)['J']

    Waist_vel = mtimes(Waist_jac, Qdot_full)
    C1_vel = mtimes(C1_jac, Qdot_full)
    C2_vel = mtimes(C2_jac, Qdot_full)
    C3_vel = mtimes(C3_jac, Qdot_full)
    C4_vel = mtimes(C4_jac, Qdot_full)

    JtF_k = mtimes(C1_jac.T, vertcat(Force[k][0:3], MX.zeros(3, 1))) + \
            mtimes(C2_jac.T, vertcat(Force[k][3:6], MX.zeros(3, 1))) + \
            mtimes(C3_jac.T, vertcat(Force[k][6:9], MX.zeros(3, 1))) + \
            mtimes(C4_jac.T, vertcat(Force[k][9:12], MX.zeros(3, 1)))

    if lift_node <= k < touch_down_node:
        Tau_k = ID(q=Q_full, qdot=Qdot_full, qddot=Qddot_full)['tau']

    if k < lift_node or k >= touch_down_node:
        Tau_k = ID(q=Q_full, qdot=Qdot_full, qddot=Qddot_full)['tau'] - JtF_k

    Tau_red = Torque_Reduction(tau_full=Tau_k)['tau_red']

    J += 100.*Time[k]
    J += 1000.*dot(Q_k[3:7] - MX([0., 0., 0., 1.]), Q_k[3:7] - MX([0., 0., 0., 1.]))

    # if k <= lift_node or k >= touch_down_node:
    J += 100.*dot(Qdot_k, Qdot_k)

    if lift_node <= k < touch_down_node:
        J += 1000.*dot(Waist_pos - Waist_pos_jump, Waist_pos - Waist_pos_jump)

    # if k <= lift_node:
    #     J += 1000.*dot(Waist_vel, Waist_vel)

    g += [integrator_out['xf'] - X[k+1]]
    g_min += [0] * X[k + 1].size1()
    g_max += [0] * X[k + 1].size1()

    g += [Tau_red[0:6]]
    g_min += np.zeros((6, 1)).tolist()
    g_max += np.zeros((6, 1)).tolist()

    # g += [Tau_red]
    # g_min += np.append(np.zeros((6, 1)), np.full((DoF, 1), -400.)).tolist()
    # g_max += np.append(np.zeros((6, 1)), np.full((DoF, 1), 400.)).tolist()

    if lift_node <= k < touch_down_node:
        g += [Force[k]]
        g_min += np.zeros((nf, 1)).tolist()
        g_max += np.zeros((nf, 1)).tolist()

    if k <= lift_node:
        g += [C1_pos, C2_pos, C3_pos, C4_pos]
        g_min += [C1_pos_ground, C2_pos_ground, C3_pos_ground, C4_pos_ground]
        g_max += [C1_pos_ground, C2_pos_ground, C3_pos_ground, C4_pos_ground]

    if k >= touch_down_node:
        g += [C1_pos, C2_pos, C3_pos, C4_pos]
        g_min += [C1_pos_ground, C2_pos_ground, C3_pos_ground, C4_pos_ground]
        g_max += [C1_pos_ground, C2_pos_ground, C3_pos_ground, C4_pos_ground]

    if k >= 24:
        g += [Waist_pos]
        g_min += [Waist_pos_init]
        g_max += [Waist_pos_init]

    if k >= 24:
        g += [Waist_vel]
        g_min += np.zeros((6, 1)).tolist()
        g_max += np.zeros((6, 1)).tolist()

    # Linearized friction cones
    g += [mtimes(A_fr, Force[k][0:3]), mtimes(A_fr, Force[k][3:6]),
          mtimes(A_fr, Force[k][6:9]), mtimes(A_fr, Force[k][9:12])]
    g_max += np.zeros((20, 1)).tolist()
    g_min += np.full((20, 1), -inf).tolist()

    Waist_pos_hist[0:3, k] = Waist_pos
    Waist_vel_hist[0:6, k] = Waist_vel
    C1_history[0:3, k] = C1_pos
    C2_history[0:3, k] = C2_pos
    C3_history[0:3, k] = C3_pos
    C4_history[0:3, k] = C4_pos
    C1_vel_history[0:3, k] = C1_vel[0:3]
    C2_vel_history[0:3, k] = C2_vel[0:3]
    C3_vel_history[0:3, k] = C3_vel[0:3]
    C4_vel_history[0:3, k] = C4_vel[0:3]
    Fc1_history[0:3, k] = Force[k][0:3]
    Fc2_history[0:3, k] = Force[k][3:6]
    Fc3_history[0:3, k] = Force[k][6:9]
    Fc4_history[0:3, k] = Force[k][9:12]
    tau_u_history[0:6, k] = Tau_red[0:6]
    tau_a_history[0:DoF, k] = Tau_red[6:nv]
    q_history[0:nq, k] = Q_k
    qdot_history[0:nv, k] = Qdot_k
    qddot_history[0:nv, k] = Qddot[k]
    h_history[0, k] = Time[k]

g = vertcat(*g)
v_init = vertcat(*v_init)
g_min = vertcat(*g_min)
g_max = vertcat(*g_max)
v_min = vertcat(*v_min)
v_max = vertcat(*v_max)


# Create an NLP solver
prob = {'f': J, 'x': V, 'g': g}
opts = {'ipopt.tol': 1e-3,
        'ipopt.constr_viol_tol': 1e-2,
        'ipopt.max_iter': 2000,
        'ipopt.linear_solver': 'ma57'}
solver = nlpsol('solver', 'ipopt', prob, opts)

# Solve the NLP
sol = solver(x0=v_init, lbx=v_min, ubx=v_max, lbg=g_min, ubg=g_max)
w_opt = sol['x'].full().flatten()
lam_w_opt = sol['lam_x']
lam_g_opt = sol['lam_g']

# Plot the solution
tau_u_hist = Function("tau_u_hist", [V], [tau_u_history])
tau_u_hist_value = tau_u_hist(w_opt).full()
tau_a_hist = Function("tau_a_hist", [V], [tau_a_history])
tau_a_hist_value = tau_a_hist(w_opt).full()

Waist_vel = Function("Waist_vel", [V], [Waist_vel_hist])
Waist_vel_value = Waist_vel(w_opt).full()

Waist_pos = Function("Waist_vel", [V], [Waist_pos_hist])
Waist_pos_value = Waist_pos(w_opt).full()

Fc1_hist = Function("Fc1_hist", [V], [Fc1_history])
Fc1_hist_value = Fc1_hist(w_opt).full()
Fc2_hist = Function("Fc2_hist", [V], [Fc2_history])
Fc2_hist_value = Fc2_hist(w_opt).full()
Fc3_hist = Function("Fc3_hist", [V], [Fc3_history])
Fc3_hist_value = Fc3_hist(w_opt).full()
Fc4_hist = Function("Fc4_hist", [V], [Fc4_history])
Fc4_hist_value = Fc4_hist(w_opt).full()

C1_hist = Function("C1_hist", [V], [C1_history])
C1_hist_value = C1_hist(w_opt).full()
C2_hist = Function("C2_hist", [V], [C2_history])
C2_hist_value = C2_hist(w_opt).full()
C3_hist = Function("C3_hist", [V], [C3_history])
C3_hist_value = C3_hist(w_opt).full()
C4_hist = Function("C4_hist", [V], [C4_history])
C4_hist_value = C4_hist(w_opt).full()

C1_vel_hist = Function("C1_vel_hist", [V], [C1_vel_history])
C1_vel_hist_value = C1_vel_hist(w_opt).full()
C2_vel_hist = Function("C2_vel_hist", [V], [C2_vel_history])
C2_vel_hist_value = C2_vel_hist(w_opt).full()
C3_vel_hist = Function("C3_vel_hist", [V], [C3_vel_history])
C3_vel_hist_value = C3_vel_hist(w_opt).full()
C4_vel_hist = Function("C4_vel_hist", [V], [C4_vel_history])
C4_vel_hist_value = C4_vel_hist(w_opt).full()

q_hist = Function("q_hist", [V], [q_history])
q_hist_value = q_hist(w_opt).full()
qdot_hist = Function("qdot_hist", [V], [qdot_history])
qdot_hist_value = qdot_hist(w_opt).full()
qddot_hist = Function("qddot_hist", [V], [qddot_history])
qddot_hist_value = qddot_hist(w_opt).full()

h_hist = Function("h_hist", [V], [h_history])
h_hist_value = h_hist(w_opt).full()

logger.add('q', q_hist_value)
logger.add('qdot', qdot_hist_value)
logger.add('qddot', qddot_hist_value)
logger.add('tau_u', tau_u_hist_value)
logger.add('tau_a', tau_a_hist_value)
logger.add('Waist_twist', Waist_vel_value)
logger.add('Waist_pos', Waist_pos_value)
logger.add('Fc1', Fc1_hist_value)
logger.add('Fc2', Fc2_hist_value)
logger.add('Fc3', Fc3_hist_value)
logger.add('Fc4', Fc4_hist_value)
logger.add('C1', C1_hist_value)
logger.add('C2', C2_hist_value)
logger.add('C3', C3_hist_value)
logger.add('C4', C4_hist_value)
logger.add('C1_vel', C1_vel_hist_value)
logger.add('C2_vel', C2_vel_hist_value)
logger.add('C3_vel', C3_vel_hist_value)
logger.add('C4_vel', C4_vel_hist_value)
logger.add('h', h_hist_value)
logger.add('ns', ns)


# Resampler
dt = 0.001

# Formulate discrete time dynamics
dae = {'x': x, 'p': qddot, 'ode': xdot, 'quad': []}
opts = {'tf': dt}
F_int = integrator('F_int', 'cvodes', dae, opts)

Tf = 0.0
T_i = {}

for k in range(ns):
    if k == 0:
        T_i[k] = 0.0
    else:
        T_i[k] = T_i[k-1] + h_hist_value[0, k-1]

    Tf += h_hist_value[0, k]

n_res = int(round(Tf/dt))

q_res = MX(Sparsity.dense(nq, n_res))
qdot_res = MX(Sparsity.dense(nv, n_res))
qddot_res = MX(Sparsity.dense(nv, n_res))
F_res = MX(Sparsity.dense(nf, n_res))
X_res = MX(Sparsity.dense(nx, n_res+1))

k = 0

for i in range(ns):
    for j in range(int(round(h_hist_value[0, i]/dt))):

        n_prev = int(round(T_i[i]/dt))

        if j == 0:
            integrator_1 = F_int(x0=X[i], p=Qddot[i])
            X_res[0:nx, k+1] = integrator_1['xf']
        else:
            integrator_2 = F_int(x0=X_res[0:nx, k], p=Qddot[i])
            X_res[0:nx, k+1] = integrator_2['xf']

        q_res[0:nq, k] = X_res[0:nq, k+1]
        qdot_res[0:nv, k] = X_res[nq:nx, k+1]
        qddot_res[0:nv, k] = Qddot[i]
        F_res[0:nf, k] = Force[i]

        k += 1

Resampler = Function("Resampler", [V], [q_res, qdot_res, qddot_res, F_res], ['V'], ['q_res', 'qdot_res', 'qddot_res', 'F_res'])

q_hist_res = Resampler(V=w_opt)['q_res'].full()
qdot_hist_res = Resampler(V=w_opt)['qdot_res'].full()
qddot_hist_res = Resampler(V=w_opt)['qddot_res'].full()
F_hist_res = Resampler(V=w_opt)['F_res'].full()

logger.add('q_resample', q_hist_res)
logger.add('qdot_resample', qdot_hist_res)
logger.add('qddot_resample', qddot_hist_res)
logger.add('F_resample', F_hist_res)

# RESAMPLER REPLAY TRAJECTORY

Tf = 0.0
T_i = {}

node_replay = touch_down_node

for k in range(node_replay):
    if k == 0:
        T_i[k] = 0.0
    else:
        T_i[k] = T_i[k-1] + h_hist_value[0, k-1]

    Tf += h_hist_value[0, k]

n_replay = int(round(Tf/dt))

q_replay = MX(Sparsity.dense(DoF, n_replay))
qdot_replay = MX(Sparsity.dense(DoF, n_replay))
tau_replay = MX(Sparsity.dense(DoF, n_replay))
X_res = MX(Sparsity.dense(nx, n_replay+1))

k = 0

for i in range(node_replay):
    for j in range(int(round(h_hist_value[0, i]/dt))):

        n_prev = int(round(T_i[i]/dt))

        if j == 0:
            integrator_1 = F_int(x0=X[i], p=Qddot[i])
            X_res[0:nx, k+1] = integrator_1['xf']
        else:
            integrator_2 = F_int(x0=X_res[0:nx, k], p=Qddot[i])
            X_res[0:nx, k+1] = integrator_2['xf']

        q_replay[0:DoF, k] = X_res[7:nq, k+1]
        qdot_replay[0:DoF, k] = X_res[nq+6:nx, k+1]
        tau_replay[0:DoF, k] = tau_a_history[0:DoF, i]

        k += 1

Resampler_replay = Function("Resampler_replay", [V], [q_replay, qdot_replay, tau_replay], ['V'], ['q_replay', 'qdot_replay', 'tau_replay'])

q_replay = Resampler_replay(V=w_opt)['q_replay'].full()
qdot_replay = Resampler_replay(V=w_opt)['qdot_replay'].full()
tau_replay = Resampler_replay(V=w_opt)['tau_replay'].full()

logger.add('q_replay', q_replay)
logger.add('qdot_replay', qdot_replay)
logger.add('tau_replay', tau_replay)

del(logger)

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import geometry_msgs.msg

pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(1./dt)
joint_state_pub = JointState()
joint_state_pub.header = Header()
joint_state_pub.name = ['hip_yaw_1', 'hip_pitch_1', 'knee_pitch_1', 'ankle_pitch_1', 'ankle_yaw_1',
                        'hip_yaw_2', 'hip_pitch_2', 'knee_pitch_2', 'ankle_pitch_2', 'ankle_yaw_2',
                        'hip_yaw_3', 'hip_pitch_3', 'knee_pitch_3', 'ankle_pitch_3', 'ankle_yaw_3',
                        'hip_yaw_4', 'hip_pitch_4', 'knee_pitch_4', 'ankle_pitch_4', 'ankle_yaw_4',
                        'torso_yaw',
                        'j_arm1_1', 'j_arm1_2', 'j_arm1_3', 'j_arm1_4', 'j_arm1_5', 'j_arm1_6', 'j_arm1_7',
                        'j_arm2_1', 'j_arm2_2', 'j_arm2_3', 'j_arm2_4', 'j_arm2_5', 'j_arm2_6', 'j_arm2_7']

br = tf.TransformBroadcaster()
m = geometry_msgs.msg.TransformStamped()
m.header.frame_id = 'world_odom'
m.child_frame_id = 'pelvis'

while not rospy.is_shutdown():
    for k in range(n_res):

        m.transform.translation.x = q_hist_res[0, k]
        m.transform.translation.y = q_hist_res[1, k]
        m.transform.translation.z = q_hist_res[2, k]
        m.transform.rotation.x = q_hist_res[3, k]
        m.transform.rotation.y = q_hist_res[4, k]
        m.transform.rotation.z = q_hist_res[5, k]
        m.transform.rotation.w = q_hist_res[6, k]

        br.sendTransform((m.transform.translation.x, m.transform.translation.y, m.transform.translation.z),
                         (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w),
                         rospy.Time.now(), m.child_frame_id, m.header.frame_id)

        joint_state_pub.header.stamp = rospy.Time.now()
        joint_state_pub.position = [q_hist_res[7, k], q_hist_res[8, k], q_hist_res[9, k], -0.301666, 0.746874,
                                    q_hist_res[10, k], q_hist_res[11, k], q_hist_res[12, k], 0.301666, -0.746874,
                                    q_hist_res[13, k], q_hist_res[14, k], q_hist_res[15, k], 0.301666, -0.746874,
                                    q_hist_res[16, k], q_hist_res[17, k], q_hist_res[18, k], -0.301666, 0.746874,
                                    3.56617e-13,
                                    0.520149, 0.320865, 0.274669, -2.23604, 0.0500815, -0.781461, -0.0567608,
                                    0.520149, -0.320865, -0.274669, -2.23604, -0.0500815, -0.781461, 0.0567608]
        joint_state_pub.velocity = []
        joint_state_pub.effort = []
        pub.publish(joint_state_pub)
        rate.sleep()

