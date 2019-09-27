#!/usr/bin/env python
from casadi import *
from cartesian_interface.pyci_all import *
import matlogger2.matlogger as matl
import centroidal_planner.pycpl_casadi as cpl_cas
import rospy

logger = matl.MatLogger2('/tmp/test_casadi_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

# get cartesio ros client
ci = pyci.CartesianInterfaceRos()

urdf = rospy.get_param('robot_description')

fk_waist = cpl_cas.generate_forward_kin(urdf, 'Waist')
FK_waist = Function.deserialize(fk_waist)

fk1 = cpl_cas.generate_forward_kin(urdf, 'Contact1')
FK1 = Function.deserialize(fk1)

fk2 = cpl_cas.generate_forward_kin(urdf, 'Contact2')
FK2 = Function.deserialize(fk2)

fk3 = cpl_cas.generate_forward_kin(urdf, 'Contact3')
FK3 = Function.deserialize(fk3)

fk4 = cpl_cas.generate_forward_kin(urdf, 'Contact4')
FK4 = Function.deserialize(fk4)

id_string = cpl_cas.generate_inv_dyn(urdf)
ID = Function.deserialize(id_string)

tf = 1.  # Normalized time horizon
ns = 50  # number of shooting nodes

nc = 4  # number of contacts

nq = 18 + 1  # number of DoFs - TODO: check "universe" joint (+1 DoF)

# Declare model variables
t = SX.sym('t')
T = SX.sym('T')
q = SX.sym('q', nq)
qdot = SX.sym('qdot', nq)

# Control variables
qddot = SX.sym('qddot', nq)
f = SX.sym('f', 3*nc)

x = vertcat(q, qdot)

# Number of differential states
nx = x.size1()

# Number of controls
nq = q.size1()
nf = f.size1()

# Bounds and initial guess for the control
qddot_min = np.full((1, nq), -inf)
qddot_max = -qddot_min
qddot_init = np.zeros_like(qddot_min)

f_min = np.full((1, nf), -inf)
f_max = -f_min
f_init = np.zeros_like(f_min)

# Bounds and initial guess for the state # TODO: get from robot
q_min = np.full((1, nq), -inf)
# q_min = np.array([-10., -10., -10., 0., 0., 0., -2., -2., -2., -2., -2., -2., -2., -2., -2., -2., -2., -2., 0.])
q_max = -q_min
q_init = np.zeros_like(q_min)

qdot_min = q_min
qdot_max = -qdot_min
qdot_init = np.zeros_like(q_min)

xmin = q_min
xmin = np.append(xmin, qdot_min)
xmax = -xmin

xfmin = np.full((1, nx), -inf)
xfmax = -xfmin

x0_min = np.zeros_like(xmin)
x0_max = x0_min

x_init = np.zeros_like(xmin)

# Model equations
xdot = T*vertcat(qdot, qddot)

# Objective term
L = 0

# Formulate discrete time dynamics

# CVODES from the SUNDIALS suite
dae = {'x': x, 'p': vertcat(qddot, T), 'ode': xdot, 'quad': L}
opts = {'tf': tf/ns}
F = integrator('F', 'cvodes', dae, opts)

# Start with an empty NLP
NV = nx*(ns+1) + (nq+nf)*ns + 1
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
Time = V[NV-1]

# Formulate the NLP
for k in range(ns):

    # State at k-th node
    X.append(V[offset:offset+nx])

    if k == 0:
        v_min += x0_min.tolist()
        v_max += x0_max.tolist()
    else:
        v_min += xmin.tolist()
        v_max += xmax.tolist()

    v_init += x_init.tolist()

    offset += nx

    # Control at k-th node
    Qddot.append(V[offset:offset+nq])
    v_min += qddot_min.tolist()
    v_max += qddot_max.tolist()
    v_init += qddot_init.tolist()

    offset += nq

    Force.append(V[offset:offset+nf])
    v_min += f_min.tolist()
    v_max += f_max.tolist()
    v_init += f_init.tolist()

    offset += nf

# Final state
X.append(V[offset:offset+nx])
v_min += xfmin.tolist()
v_max += xfmax.tolist()
v_init += x_init.tolist()
offset += nx

# Trajectory time
v_min += [0.0]
v_max += [100.0]
v_init += [1.0]
offset += 1

assert offset == NV

# Create NLP
J = MX([0])
g = []

Waist_pos_hist = MX(Sparsity.dense(3, ns))
Waist_vel_hist = MX(Sparsity.dense(3, ns))

Waist_pos = None
Waist_vel = None

for k in range(ns):

    integrator_out = F(x0=X[k], p=vertcat(Qddot[k], Time))

    g += [integrator_out['xf'] - X[k+1]]
    g_min += [0] * X[k + 1].size1()
    g_max += [0] * X[k + 1].size1()

    J += integrator_out['qf']

    #if k == ns:
    Q_k = X[k][0:nq]
    Qdot_k = X[k][nq:2*nq]

    Waist_pos = FK_waist(q=Q_k)['ee_pos']
    dFK_dq = FK_waist.jacobian_old(0, 0)

    dWaist_pos_dq = dFK_dq(Q_k)[0]
    Waist_vel = mtimes(dWaist_pos_dq, Qdot_k)
    Waist_pos_ref = MX([0, 0, 0])

    Waist_pos_hist[0:3, k] = Waist_pos
    Waist_vel_hist[0:3, k] = Waist_vel

    J += 1000.0 * dot(Waist_pos - Waist_pos_ref, Waist_pos - Waist_pos_ref)
    # J += 1000.0 * dot(Waist_vel, Waist_vel)


tau_u_history = MX(Sparsity.dense(6, ns))
tau_a_history = MX(Sparsity.dense(12, ns))
tau0 = np.zeros((6, 1))
tau_min = np.full((1, 12), -10.)
tau_max = -tau_min

Fc1_history = MX(Sparsity.dense(3, ns))
Fc2_history = MX(Sparsity.dense(3, ns))
Fc3_history = MX(Sparsity.dense(3, ns))
Fc4_history = MX(Sparsity.dense(3, ns))

C1_history = MX(Sparsity.dense(3, ns))
C2_history = MX(Sparsity.dense(3, ns))
C3_history = MX(Sparsity.dense(3, ns))
C4_history = MX(Sparsity.dense(3, ns))

C1_pos = None
C2_pos = None
C3_pos = None
C4_pos = None

for k in range(ns):

    Q_k = X[k][0:nq]
    Qdot_k = X[k][nq:2 * nq]

    Tau_k = ID(q=Q_k, qdot=Qdot_k, qddot=Qddot[k])['tau']

    tau_u_history[0:6, k] = Tau_k[0:6]
    tau_a_history[0:12, k] = Tau_k[6:18]

    C1_pos = FK1(q=Q_k)['ee_pos']
    dFK1_dq = FK1.jacobian_old(0, 0)
    dC1_pos_dq = dFK1_dq(Q_k)[0]

    C2_pos = FK2(q=Q_k)['ee_pos']
    dFK2_dq = FK2.jacobian_old(0, 0)
    dC2_pos_dq = dFK2_dq(Q_k)[0]

    C3_pos = FK3(q=Q_k)['ee_pos']
    dFK3_dq = FK3.jacobian_old(0, 0)
    dC3_pos_dq = dFK3_dq(Q_k)[0]

    C4_pos = FK4(q=Q_k)['ee_pos']
    dFK4_dq = FK4.jacobian_old(0, 0)
    dC4_pos_dq = dFK4_dq(Q_k)[0]

    Fc1_history[0:3, k] = Force[k][0:3]
    Fc2_history[0:3, k] = Force[k][3:6]
    Fc3_history[0:3, k] = Force[k][6:9]
    Fc4_history[0:3, k] = Force[k][9:12]

    C1_history[0:3, k] = C1_pos
    C2_history[0:3, k] = C2_pos
    C3_history[0:3, k] = C3_pos
    C4_history[0:3, k] = C4_pos

    JtF_k = mtimes(dC1_pos_dq.T, Force[k][0:3]) + \
            mtimes(dC2_pos_dq.T, Force[k][3:6]) + \
            mtimes(dC3_pos_dq.T, Force[k][6:9]) + \
            mtimes(dC4_pos_dq.T, Force[k][9:12])

    g += [Tau_k[0:6] - JtF_k[0:6]]
    g_min += tau0.tolist()
    g_max += tau0.tolist()

    g += [Tau_k[6:18] - JtF_k[6:18]]
    g_min += tau_min.tolist()
    g_max += tau_max.tolist()


J += Time

g = vertcat(*g)
v_init = vertcat(*v_init)
g_min = vertcat(*g_min)
g_max = vertcat(*g_max)
v_min = vertcat(*v_min)
v_max = vertcat(*v_max)

# Create an NLP solver
prob = {'f': J, 'x': V, 'g': g}
opts = {'ipopt.tol': 1e-5,
        'ipopt.max_iter': 100,
        'ipopt.linear_solver': 'ma57'}
solver = nlpsol('solver', 'ipopt', prob, opts)

# Solve the NLP
sol = solver(x0=v_init, lbx=v_min, ubx=v_max, lbg=g_min, ubg=g_max)
w_opt = sol['x'].full().flatten()

# Plot the solution
tgrid = [w_opt[-1]/ns*k for k in range(ns+1)]

F_tau_u_hist = Function("F_tau_u_hist", [V], [tau_u_history])
tau_u_hist_value = F_tau_u_hist(w_opt).full()
F_tau_a_hist = Function("F_tau_a_hist", [V], [tau_a_history])
tau_a_hist_value = F_tau_a_hist(w_opt).full()

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

q_opt = np.zeros((nq, ns+1))
qdot_opt = np.zeros((nq, ns+1))
qddot_opt = np.zeros((nq, ns))

for k in range(ns+1):
    q_opt[:, k] = w_opt[((3*nq+nf)*k):((3*nq+nf)*k + nq)]
    qdot_opt[:, k] = w_opt[((3*nq+nf)*k + nq):((3*nq+nf)*k + 2*nq)]

    if k < ns:
        qddot_opt[:, k] = w_opt[((3*nq+nf)*k + 2*nq):((3*nq+nf) * k + 3*nq)]

logger.add('q', q_opt)
logger.add('qdot', qdot_opt)
logger.add('qddot', qddot_opt)
logger.add('tau_u', tau_u_hist_value)
logger.add('tau_a', tau_a_hist_value)
logger.add('Waist_vel', Waist_vel_value)
logger.add('Waist_pos', Waist_pos_value)
logger.add('tf', w_opt[-1])
logger.add('Fc1', Fc1_hist_value)
logger.add('Fc2', Fc2_hist_value)
logger.add('Fc3', Fc3_hist_value)
logger.add('Fc4', Fc4_hist_value)
logger.add('C1', C1_hist_value)
logger.add('C2', C2_hist_value)
logger.add('C3', C3_hist_value)
logger.add('C4', C4_hist_value)


print('tf: ', w_opt[-1])

del(logger)

# import matplotlib.pyplot as plt
#
# plt.figure(1)
# plt.clf()
# plt.plot(tgrid, q_opt.transpose())
# plt.xlabel('t')
# plt.ylabel('q')
# plt.grid()
#
# plt.figure(2)
# plt.plot(tgrid, qdot_opt.transpose())
# plt.xlabel('t')
# plt.ylabel('qdot')
# plt.grid()
#
# plt.figure(3)
# plt.step(tgrid[0:ns], qddot_opt.transpose())
# plt.xlabel('t')
# plt.ylabel('qddot')
# plt.grid()
#
# plt.figure(4)
# plt.plot(tgrid[0:ns], tau_u_hist_value.transpose())
# plt.xlabel('t')
# plt.ylabel('tau')
# plt.grid()
#
# plt.figure(5)
# plt.plot(tgrid[0:ns], Waist_vel_value.transpose())
# plt.xlabel('t')
# plt.ylabel('waist_vel')
# plt.grid()
#
# plt.show()


print('Exiting..')
