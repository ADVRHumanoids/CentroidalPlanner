#!/usr/bin/env python
from casadi import *
from cartesian_interface.pyci_all import *
import matlogger2.matlogger as matl
import centroidal_planner.pycpl_casadi as cpl_cas
import rospy
import xbot_interface.config_options as cfg
import xbot_interface.xbot_interface as xbot

logger = matl.MatLogger2('/tmp/legs_wall_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

# get cartesio ros client
ci = pyci.CartesianInterfaceRos()
opt = cfg.ConfigOptions()
model = xbot.ModelInterface(opt)

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

jac_waist = cpl_cas.generate_jacobian(urdf, 'Waist')
Jac_waist = Function.deserialize(jac_waist)

jac_C1 = cpl_cas.generate_jacobian(urdf, 'Contact1')
Jac_C1 = Function.deserialize(jac_C1)

jac_C2 = cpl_cas.generate_jacobian(urdf, 'Contact2')
Jac_C2 = Function.deserialize(jac_C2)

jac_C3 = cpl_cas.generate_jacobian(urdf, 'Contact3')
Jac_C3 = Function.deserialize(jac_C3)

jac_C4 = cpl_cas.generate_jacobian(urdf, 'Contact4')
Jac_C4 = Function.deserialize(jac_C4)

tf = 2.  # Normalized time horizon
ns = 30  # number of shooting nodes

nc = 4  # number of contacts

nq = 12+7  # number of DoFs - NB: 7 DoFs floating base (quaternions)
nv = nq-1
nf = 3*nc

# Variables
q = SX.sym('q', nq)
qdot = SX.sym('qdot', nv)
qddot = SX.sym('qddot', nv)
f = SX.sym('f', nf)
t = MX.sym('t', ns)

# Bounds and initial guess for the control
qddot_min = np.full((1, nv), -1000.)
qddot_max = np.full((1, nv), 1000.)
qddot_init = np.zeros_like(qddot_min)

# f_min = np.tile(np.array([-100, -100, 20]), 4)
# f_min[2] = 0
f_min = np.tile(np.array([-1000., -1000., -1000.]), 4)
f_max = np.tile(np.array([1000., 1000., 1000.]), 4)
f_init = np.zeros_like(f_min)

# Bounds and initial guess for the state # TODO: get from robot
# q_min = np.full((1, nq), -inf)
# q_max = -q_min
q_min = np.array([-inf, -inf, -inf, -inf, -inf, -inf, -inf, 0.1, 0.0, -0.5, 0.1, -0.4, -0.5, -0.5, -0.4, -0.5, -0.5, 0.0, -0.5])
q_max = np.array([inf,  inf,  inf,  inf,  inf,  inf,  inf, 0.5, 0.4, -0.2, 0.5, 0.0, -0.2, -0.1, 0.0, -0.2, -0.1, 0.4, -0.2])
q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.3, 0.2, -0.5, 0.3, -0.2, -0.5, -0.3, -0.2, -0.5, -0.3, 0.2, -0.5])

qdot_min = np.full((1, nv), -1000.)
qdot_max = np.full((1, nv), 1000.)
qdot_init = np.zeros_like(qdot_min)

xmin = np.append(q_min, qdot_min)
xmax = np.append(q_max, qdot_max)

x_init = np.append(q_init, np.zeros_like(qdot_min))

x0_min = x_init
x0_max = x_init

# xf_min = x_init
# xf_max = x_init
xf_min = np.append(q_min, np.zeros_like(qdot_min))
xf_max = np.append(q_max, np.zeros_like(qdot_min))

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
xdot = vertcat(qdot[0:3], tmp1, tmp2, qdot[6:18], qddot)

nx = x.size1()

# Objective term
L = 0.1*dot(qddot, qddot)

# Formulate discrete time dynamics
dae = {'x': x, 'p': qddot, 'ode': xdot, 'quad': L}
opts = {'tf': tf/ns}
F_int = integrator('F_int', 'cvodes', dae, opts)

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
        v_min += xmin.tolist()
        v_max += xmax.tolist()

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
    v_min += np.array([0.05]).tolist()
    v_max += np.array([0.50]).tolist()
    v_init += np.array([0.05]).tolist()

    offset += 1

# Final state
X.append(V[offset:offset+nx])
v_min += xmin.tolist()
v_max += xmax.tolist()
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
Fc1_history = MX(Sparsity.dense(3, ns))
Fc2_history = MX(Sparsity.dense(3, ns))
Fc3_history = MX(Sparsity.dense(3, ns))
Fc4_history = MX(Sparsity.dense(3, ns))
tau_u_history = MX(Sparsity.dense(6, ns))
tau_a_history = MX(Sparsity.dense(12, ns))
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

    # integrator_out = F_int(x0=X[k], p=Qddot[k])
    integrator_out = F_RK(x0=X[k], p=Qddot[k], time=Time[k])

    Q_k = X[k][0:nq]
    Qdot_k = X[k][nq:nq + nv]

    Waist_pos = FK_waist(q=Q_k)['ee_pos']
    C1_pos = FK1(q=Q_k)['ee_pos']
    C2_pos = FK2(q=Q_k)['ee_pos']
    C3_pos = FK3(q=Q_k)['ee_pos']
    C4_pos = FK4(q=Q_k)['ee_pos']

    Waist_jac = Jac_waist(q=Q_k)['J']
    C1_jac = Jac_C1(q=Q_k)['J']
    C2_jac = Jac_C2(q=Q_k)['J']
    C3_jac = Jac_C3(q=Q_k)['J']
    C4_jac = Jac_C4(q=Q_k)['J']

    Waist_vel = mtimes(Waist_jac, Qdot_k)

    JtF_k = mtimes(C1_jac.T, vertcat(Force[k][0:3], MX.zeros(3, 1))) + \
            mtimes(C2_jac.T, vertcat(Force[k][3:6], MX.zeros(3, 1))) + \
            mtimes(C3_jac.T, vertcat(Force[k][6:9], MX.zeros(3, 1))) + \
            mtimes(C4_jac.T, vertcat(Force[k][9:12], MX.zeros(3, 1)))

    Tau_k = ID(q=Q_k, qdot=Qdot_k, qddot=Qddot[k])['tau'] - JtF_k

    J += integrator_out['qf']
    J += 100*Time[k]
    J += 1000.*dot(Q_k[3:7] - MX([0., 0., 0., 1.]), Q_k[3:7] - MX([0., 0., 0., 1.]))
    J += 100*dot(Qdot_k[0:6], Qdot_k[0:6])
    J += 1000.*dot(Waist_pos, Waist_pos)

    g += [integrator_out['xf'] - X[k+1]]
    g_min += [0] * X[k + 1].size1()
    g_max += [0] * X[k + 1].size1()

    g += [Tau_k]
    g_min += np.append(np.zeros((6, 1)), np.full((12, 1), -400.)).tolist()
    g_max += np.append(np.zeros((6, 1)), np.full((12, 1), 400.)).tolist()

    C1_pos_ground = [0.3, 0.2, -0.5]
    C2_pos_ground = [0.3, -0.2, -0.5]
    C3_pos_ground = [-0.3, -0.2, -0.5]
    C4_pos_ground = [-0.3, 0.2, -0.5]
    C3_pos_wall = [-0.4, -0.2, -0.3]
    C4_pos_wall = [-0.4, 0.2, -0.3]

    g += [C1_pos, C2_pos]
    g_min += np.array([C1_pos_ground, C2_pos_ground]).tolist()
    g_max += np.array([C1_pos_ground, C2_pos_ground]).tolist()

    if k < 10:
        g += [C3_pos, C4_pos]
        g_min += np.array([C3_pos_ground, C4_pos_ground]).tolist()
        g_max += np.array([C3_pos_ground, C4_pos_ground]).tolist()

    if k >= 20:
        g += [C3_pos, C4_pos]
        g_min += np.array([C3_pos_wall, C4_pos_wall]).tolist()
        g_max += np.array([C3_pos_wall, C4_pos_wall]).tolist()

    if 10 <= k < 20:
        g += [Force[k][6:12]]
        g_min += np.zeros((6, 1)).tolist()
        g_max += np.zeros((6, 1)).tolist()

    if k >= 25:
        g += [Waist_vel]
        g_min += np.zeros((6, 1)).tolist()
        g_max += np.zeros((6, 1)).tolist()

    # Collisions Waist/rear legs
    g += [Waist_pos[2]-C3_pos[2], Waist_pos[2]-C4_pos[2]]
    g_min += np.array([0.3, 0.3]).tolist()
    g_max += np.array([100., 100.]).tolist()

    # Linearized friction cones
    g += [mtimes(A_fr, Force[k][0:3]), mtimes(A_fr, Force[k][3:6])]
    g_min += np.full((10, 1), -inf).tolist()
    g_max += np.zeros((10, 1)).tolist()

    if k < 10:
        g += [mtimes(A_fr, Force[k][6:9]), mtimes(A_fr, Force[k][9:12])]
        g_min += np.full((10, 1), -inf).tolist()
        g_max += np.zeros((10, 1)).tolist()

    if k >= 20:
        g += [mtimes(A_fr_R, Force[k][6:9]), mtimes(A_fr_R, Force[k][9:12])]
        g_min += np.full((10, 1), -inf).tolist()
        g_max += np.zeros((10, 1)).tolist()

    Waist_pos_hist[0:3, k] = Waist_pos
    Waist_vel_hist[0:6, k] = Waist_vel
    C1_history[0:3, k] = C1_pos
    C2_history[0:3, k] = C2_pos
    C3_history[0:3, k] = C3_pos
    C4_history[0:3, k] = C4_pos
    Fc1_history[0:3, k] = Force[k][0:3]
    Fc2_history[0:3, k] = Force[k][3:6]
    Fc3_history[0:3, k] = Force[k][6:9]
    Fc4_history[0:3, k] = Force[k][9:12]
    tau_u_history[0:6, k] = Tau_k[0:6]
    tau_a_history[0:12, k] = Tau_k[6:18]
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
opts = {'ipopt.tol': 1e-5,
        'ipopt.max_iter': 2000,
        'ipopt.linear_solver': 'ma57'}
solver = nlpsol('solver', 'ipopt', prob, opts)

# Solve the NLP
sol1 = solver(x0=v_init, lbx=v_min, ubx=v_max, lbg=g_min, ubg=g_max)
w_opt1 = sol1['x'].full().flatten()
lam_w_opt = sol1['lam_x']
lam_g_opt = sol1['lam_g']

# sol = solver(x0=w_opt1, lbx=v_min, ubx=v_max, lbg=g_min, ubg=g_max, lam_x0=lam_w_opt, lam_g0=lam_g_opt)
# w_opt = sol['x'].full().flatten()

w_opt = w_opt1

# Plot the solution
tgrid = [1./ns*k for k in range(ns+1)]

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
logger.add('t', tgrid)
logger.add('h', h_hist_value)
logger.add('ns', ns)

del(logger)

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import tf
import geometry_msgs.msg

pub = rospy.Publisher('joint_states', JointState, queue_size=10)
rospy.init_node('joint_state_publisher')
rate = rospy.Rate(10)
joint_state_pub = JointState()
joint_state_pub.header = Header()
joint_state_pub.name = ['Contact1_x', 'Contact1_y', 'Contact1_z',
                        'Contact2_x', 'Contact2_y', 'Contact2_z',
                        'Contact3_x', 'Contact3_y', 'Contact3_z',
                        'Contact4_x', 'Contact4_y', 'Contact4_z']

br = tf.TransformBroadcaster()
m = geometry_msgs.msg.TransformStamped()
m.header.frame_id = 'world_odom'
m.child_frame_id = 'base_link'

while not rospy.is_shutdown():
    for k in range(ns):

        m.transform.translation.x = q_hist_value[0, k]
        m.transform.translation.y = q_hist_value[1, k]
        m.transform.translation.z = q_hist_value[2, k]
        m.transform.rotation.x = q_hist_value[3, k]
        m.transform.rotation.y = q_hist_value[4, k]
        m.transform.rotation.z = q_hist_value[5, k]
        m.transform.rotation.w = q_hist_value[6, k]

        br.sendTransform((m.transform.translation.x, m.transform.translation.y, m.transform.translation.z),
                         (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w),
                         rospy.Time.now(), m.child_frame_id, m.header.frame_id)

        joint_state_pub.header.stamp = rospy.Time.now()
        joint_state_pub.position = q_hist_value[7:19, k]
        joint_state_pub.velocity = []
        joint_state_pub.effort = []
        pub.publish(joint_state_pub)
        rate.sleep()