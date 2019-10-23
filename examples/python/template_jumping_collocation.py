#!/usr/bin/env python
from casadi import *
import matlogger2.matlogger as matl
import casadi_kin_dyn.pycasadi_kin_dyn as cas_kin_dyn
import rospy

logger = matl.MatLogger2('/tmp/jumping_collocation_log')
logger.setBufferMode(matl.BufferMode.CircularBuffer)

urdf = rospy.get_param('robot_description')
kindyn = cas_kin_dyn.CasadiKinDyn(urdf)

fk_waist = kindyn.fk('Waist')
FK_waist = Function.deserialize(fk_waist)

fk1 = kindyn.fk('Contact1')
FK1 = Function.deserialize(fk1)

fk2 = kindyn.fk('Contact2')
FK2 = Function.deserialize(fk2)

fk3 = kindyn.fk('Contact3')
FK3 = Function.deserialize(fk3)

fk4 = kindyn.fk('Contact4')
FK4 = Function.deserialize(fk4)

id_string = kindyn.rnea()
ID = Function.deserialize(id_string)

jac_waist = kindyn.jacobian('Waist')
Jac_waist = Function.deserialize(jac_waist)

jac_C1 = kindyn.jacobian('Contact1')
Jac_C1 = Function.deserialize(jac_C1)

jac_C2 = kindyn.jacobian('Contact2')
Jac_C2 = Function.deserialize(jac_C2)

jac_C3 = kindyn.jacobian('Contact3')
Jac_C3 = Function.deserialize(jac_C3)

jac_C4 = kindyn.jacobian('Contact4')
Jac_C4 = Function.deserialize(jac_C4)

# Define the interpolation polynomial degree
d = 5
# Get collocation points
tau_root = np.append(0, collocation_points(d, 'legendre'))
# Coefficient of the collocation equation
C = np.zeros((d+1, d+1))
# Coefficients of the continuity equation
D = np.zeros(d+1)
# Coefficient of the quadrature function
B = np.zeros(d+1)

for j in range(d + 1):
    # Initialize the polynomial equal to 1
    p = np.poly1d([1])
    # Perform another for cycle to evaluate the lagrangian basis only when the denominator differs from 0
    for r in range(d + 1):
        if r != j:
            p *= np.poly1d([1, -tau_root[r]]) / (tau_root[j] - tau_root[r])

    # Evaluate the polynomial at the final time to get the coefficients of the continuity equation
    D[j] = p(1.0)

    # Evaluate the time derivative of the polynomial at the collocation points to get the coefficients of the continuity equation
    pder = np.polyder(p)
    for r in range(d + 1):
        C[j, r] = pder(tau_root[r])

    # Evaluate the integral of the polynomial to get the coefficients of the quadrature function
    pint = np.polyint(p)
    B[j] = pint(1.0)

tf = 1.  # Time horizon
ns = 30  # number of shooting nodes
h = 1.  # tf/ns  # Integration step

nc = 4  # number of contacts

nq = 12+7  # number of DoFs - NB: 7 DoFs floating base (quaternions)
nv = nq-1
nf = 3*nc

# Variables
q = SX.sym('q', nq)
qdot = SX.sym('qdot', nv)
qddot = SX.sym('qddot', nv)
f = SX.sym('f', nf)

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
Waist_pos_ref = np.array([0, 0, 1])
C1_pos_ref = np.array([0.3, 0.2, -0.5])
C2_pos_ref = np.array([0.3, -0.2, -0.5])
C3_pos_ref = np.array([-0.3, -0.2, -0.5])
C4_pos_ref = np.array([-0.3, 0.2, -0.5])

Waist_pos = FK_waist(q=q)['ee_pos']
C1_pos = FK1(q=q)['ee_pos']
C2_pos = FK2(q=q)['ee_pos']
C3_pos = FK3(q=q)['ee_pos']
C4_pos = FK4(q=q)['ee_pos']

L1 = 0.1*dot(qddot, qddot)
L1 += dot(C1_pos - C1_pos_ref, C1_pos - C1_pos_ref)
L1 += dot(C2_pos - C2_pos_ref, C2_pos - C2_pos_ref)
L1 += dot(C3_pos - C3_pos_ref, C3_pos - C3_pos_ref)
L1 += dot(C4_pos - C4_pos_ref, C4_pos - C4_pos_ref)

L2 = 0.1*dot(qddot, qddot)
L2 += dot(Waist_pos - Waist_pos_ref, Waist_pos - Waist_pos_ref)

# Continuous time dynamics
f_model = Function('f_model', [q, qdot, qddot], [xdot], ['q', 'qdot', 'qddot'], ['xdot'])
f_cost1 = Function('f_cost', [q, qdot, qddot], [L1], ['q', 'qdot', 'qddot'], ['L1'])
f_cost2 = Function('f_cost', [q, qdot, qddot], [L2], ['q', 'qdot', 'qddot'], ['L2'])

# Bounds
qddot_min = np.full((1, nv), -inf)
qddot_max = -qddot_min
qddot_init = np.zeros_like(qddot_min)

f_min = np.tile(np.array([-100, -100, 0]), 4)
f_max = np.tile(np.array([100, 100, 1000]), 4)
f_init = np.zeros_like(f_min)

q_min = np.array([-inf, -inf, -inf, -inf, -inf, -inf, -inf, 0.2, 0.1, -0.5, 0.2, -0.3, -0.5, -0.4, -0.3, -0.5, -0.4, 0.1, -0.5])
q_max = np.array([inf,  inf,  inf,  inf,  inf,  inf,  inf, 0.4, 0.3, -0.2, 0.4, -0.1, -0.2, -0.2, -0.1, -0.2, -0.2, 0.3, -0.2])
q_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.3, 0.2, -0.5, 0.3, -0.2, -0.5, -0.3, -0.2, -0.5, -0.3, 0.2, -0.5])

qdot_min = np.full((1, nv), -inf)
qdot_max = -qdot_min
qdot_init = np.zeros_like(qdot_min)

x_min = np.append(q_min, qdot_min)
x_max = np.append(q_max, qdot_max)

x_init = np.append(q_init, np.zeros_like(qdot_min))

x0_min = x_init
x0_max = x_init

xf_min = x_init
xf_max = x_init

# Start with an empty NLP
w = []
w0 = []
lbw = []
ubw = []
J = 0
g = []
lbg = []
ubg = []

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
q_history = MX(Sparsity.dense(nq, ns+1))
qdot_history = MX(Sparsity.dense(nv, ns+1))
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

# Initial condition
Xk = MX.sym('X0', nx)
w.append(Xk)
lbw += x0_min.tolist()
ubw += x0_max.tolist()
w0 += x_init.tolist()

for k in range(ns):

    # New NLP variables for the control
    Qddot_k = MX.sym('Qddot_' + str(k), nv)
    w.append(Qddot_k)
    lbw += qddot_min.tolist()
    ubw += qddot_max.tolist()
    w0 += qddot_init.tolist()

    Force_k = MX.sym('Force_' + str(k), nf)
    w.append(Force_k)
    lbw += f_min.tolist()
    ubw += f_max.tolist()
    w0 += f_init.tolist()

    h_k = MX.sym('h_' + str(k), 1)
    w.append(h_k)
    lbw += np.array([1.0/ns]).tolist()
    ubw += np.array([2.0]).tolist()
    w0 += np.array([1.0/ns]).tolist()

    # # START COLLOCATION
    # State collocation points
    Xc = []
    for j in range(1, d + 1):
        Xkj = MX.sym('X_' + str(k) + '_' + str(j), nx)
        Xc.append(Xkj)
        w.append(Xkj)
        lbw += x_min.tolist()
        ubw += x_max.tolist()
        w0 += x_init.tolist()
    # Loop over collocation points
    Xk_end = D[0] * Xk
    for j in range(1, d + 1):
        # Expression for the state derivative at the collocation point
        xp = C[0, j] * Xk
        for r in range(d):
            xp = xp + C[r + 1, j] * Xc[r]

        # Append collocation equation (8.10)
        fj = f_model(q=Xc[j-1][0:nq], qdot=Xc[j-1][nq:nx], qddot=Qddot_k)['xdot']

        # g.append(h * fj - xp)
        g.append(h_k * fj - xp)
        lbg += np.zeros_like(x_min).tolist()
        ubg += np.zeros_like(x_min).tolist()

        # Add contribution to the end state (8.11)
        Xk_end = Xk_end + D[j] * Xc[j - 1]

        # Add contribution to quadrature function
        if ns/3 < k <= 2*ns/3:
            qj = f_cost2(q=Xc[j - 1][0:nq], qdot=Xc[j - 1][nq:nx], qddot=Qddot_k)['L2']
        else:
            qj = f_cost1(q=Xc[j - 1][0:nq], qdot=Xc[j - 1][nq:nx], qddot=Qddot_k)['L1']

        # J = J + B[j] * qj * h
        J = J + B[j] * qj * h_k

    # New NLP variable for state at end of interval
    Xk = MX.sym('X_' + str(k+1), nx)
    w.append(Xk)
    lbw += x_min.tolist()
    ubw += x_max.tolist()
    w0 += np.zeros_like(x_min).tolist()
    # Add equality constraint
    g.append(Xk_end - Xk)
    lbg += np.zeros_like(x_min).tolist()
    ubg += np.zeros_like(x_min).tolist()
    # # END COLLOCATION

    Q_k = Xk[0:nq]
    Qdot_k = Xk[nq:nq+nv]

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

    JtF_k = mtimes(C1_jac.T, vertcat(Force_k[0:3], MX.zeros(3, 1))) + \
            mtimes(C2_jac.T, vertcat(Force_k[3:6], MX.zeros(3, 1))) + \
            mtimes(C3_jac.T, vertcat(Force_k[6:9], MX.zeros(3, 1))) + \
            mtimes(C4_jac.T, vertcat(Force_k[9:12], MX.zeros(3, 1)))

    Tau_k = ID(q=Q_k, v=Qdot_k, a=Qddot_k)['tau'] - JtF_k

    g += [Tau_k[0:6]]
    lbg += np.zeros((6, 1)).tolist()
    ubg += np.zeros((6, 1)).tolist()

    # if ns/3 < k <= 2*ns/3:
    #     g += [Force_k]
    #     lbg += np.zeros((nf, 1)).tolist()
    #     ubg += np.zeros((nf, 1)).tolist()
    # else:
    #     g += [C1_pos, C2_pos, C3_pos, C4_pos]
    #     lbg += np.array([0.3, 0.2, -0.5, 0.3, -0.2, -0.5, -0.3, -0.2, -0.5, -0.3, 0.2, -0.5]).tolist()
    #     ubg += np.array([0.3, 0.2, -0.5, 0.3, -0.2, -0.5, -0.3, -0.2, -0.5, -0.3, 0.2, -0.5]).tolist()

    # # Linearized friction cones
    # g += [mtimes(A_fr, Force_k[0:3]), mtimes(A_fr, Force_k[3:6]),
    #       mtimes(A_fr, Force_k[6:9]), mtimes(A_fr, Force_k[9:12])]
    # lbg += np.full((20, 1), -inf).tolist()
    # ubg += np.zeros((20, 1)).tolist()

    Waist_pos_hist[0:3, k] = Waist_pos
    Waist_vel_hist[0:6, k] = Waist_vel
    C1_history[0:3, k] = C1_pos
    C2_history[0:3, k] = C2_pos
    C3_history[0:3, k] = C3_pos
    C4_history[0:3, k] = C4_pos
    Fc1_history[0:3, k] = Force_k[0:3]
    Fc2_history[0:3, k] = Force_k[3:6]
    Fc3_history[0:3, k] = Force_k[6:9]
    Fc4_history[0:3, k] = Force_k[9:12]
    tau_u_history[0:6, k] = Tau_k[0:6]
    tau_a_history[0:12, k] = Tau_k[6:18]
    q_history[0:nq, k] = Q_k
    qdot_history[0:nv, k] = Qdot_k
    qddot_history[0:nv, k] = Qddot_k
    h_history[0, k] = h_k


# Final condition
Xk = MX.sym('XF', nx)
w.append(Xk)
lbw += xf_min.tolist()
ubw += xf_max.tolist()
w0 += x_init.tolist()


# Concatenate vectors
w = vertcat(*w)
g = vertcat(*g)
w0 = vertcat(*w0)
lbw = vertcat(*lbw)
ubw = vertcat(*ubw)
lbg = vertcat(*lbg)
ubg = vertcat(*ubg)

# Create a NLP SOLVER
prob = {'f': J, 'x': w, 'g': g}
opts = {'ipopt.tol': 1e-5,
        'ipopt.max_iter': 100,
        'ipopt.linear_solver': 'ma57'}
solver = nlpsol('solver', 'ipopt', prob, opts)

# Solve the NLP
sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
w_opt = sol['x'].full().flatten()

# Plot the solution
tgrid = [1/ns*k for k in range(ns+1)]

F_tau_u_hist = Function("F_tau_u_hist", [w], [tau_u_history])
tau_u_hist_value = F_tau_u_hist(w_opt).full()
F_tau_a_hist = Function("F_tau_a_hist", [w], [tau_a_history])
tau_a_hist_value = F_tau_a_hist(w_opt).full()

Waist_vel = Function("Waist_vel", [w], [Waist_vel_hist])
Waist_vel_value = Waist_vel(w_opt).full()

Waist_pos = Function("Waist_vel", [w], [Waist_pos_hist])
Waist_pos_value = Waist_pos(w_opt).full()

Fc1_hist = Function("Fc1_hist", [w], [Fc1_history])
Fc1_hist_value = Fc1_hist(w_opt).full()
Fc2_hist = Function("Fc2_hist", [w], [Fc2_history])
Fc2_hist_value = Fc2_hist(w_opt).full()
Fc3_hist = Function("Fc3_hist", [w], [Fc3_history])
Fc3_hist_value = Fc3_hist(w_opt).full()
Fc4_hist = Function("Fc4_hist", [w], [Fc4_history])
Fc4_hist_value = Fc4_hist(w_opt).full()

C1_hist = Function("C1_hist", [w], [C1_history])
C1_hist_value = C1_hist(w_opt).full()
C2_hist = Function("C2_hist", [w], [C2_history])
C2_hist_value = C2_hist(w_opt).full()
C3_hist = Function("C3_hist", [w], [C3_history])
C3_hist_value = C3_hist(w_opt).full()
C4_hist = Function("C4_hist", [w], [C4_history])
C4_hist_value = C4_hist(w_opt).full()

q_hist = Function("q_hist", [w], [q_history])
q_hist_value = q_hist(w_opt).full()
qdot_hist = Function("qdot_hist", [w], [qdot_history])
qdot_hist_value = qdot_hist(w_opt).full()
qddot_hist = Function("qddot_hist", [w], [qddot_history])
qddot_hist_value = qddot_hist(w_opt).full()

h_hist = Function("h_hist", [w], [h_history])
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