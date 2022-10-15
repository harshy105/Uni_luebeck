import numpy as np
from ime_fgs.basic_nodes import MatrixNode, AdditionNode, PriorNode, EqualityNode
from ime_fgs.messages import GaussianMeanCovMessage, GaussianWeightedMeanInfoMessage
import matplotlib.pyplot as plt
from ime_fgs.plot import draw_graph

'''
# load the data
data = np.load("range.npz", allow_pickle=True)
z_ds = data['z_ds']  # list len=n*n-1/2, {'i': i, 'j': j, 'z_d': z_d}
x_as = data['x_as']  # (n,3)
sys_noise = data['sys_noise']  # (3,3)
obs_noise = data['obs_noise']  # [[#]]
v = data['v']  # (t,3)
t = data['t']  # (t,)
x0 = data['x0']  # (3,)
x_ds = data['x_ds']  # (101,3)
'''

# variable initialisation
x_as = np.array([[0, 0, 0], [5, 0, 0], [5, 5, 0], [0, 5, 0], [0, 0, 12], [5, 0, 12], [5, 5, 12], [0, 5, 12]])  # box of (5,5,12), shape (n,3)
sys_noise = 1e-4*np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # (3,3)
obs_noise = np.array([[1e-1]])  # [[#]]
t = np.linspace(0, 10, 500)  # (t,)
vx = np.cos(t)
vy = np.sin(t)
vz = 0.5*np.ones(t.shape)
v = np.concatenate((vx.reshape(-1, 1), vy.reshape(-1, 1), vz.reshape(-1, 1)), axis=1)  # (t,3)
x0 = np.array([1, 1, 1])  # (3,)

# trajectory
x_ds = np.zeros((len(t)+1, 3))
x_ds[0] = x0
delta_t = t[1]-t[0]
for i in range(len(t)):
    x_ds[i+1] = x_ds[i] + v[i]*delta_t + np.random.multivariate_normal([0, 0, 0], sys_noise)

x_ds = np.array(x_ds)

# sensor data
z_ds = []
for i in range(x_as.shape[0]):
    for j in range(i + 1, x_as.shape[0]):
        z_d = []
        for x in x_ds:
            d_i = np.linalg.norm(x_as[i] - x)
            d_j = np.linalg.norm(x_as[j] - x)
            z_d.append(d_i - d_j + np.random.normal(0, obs_noise[0][0]))
        the_dict = {'i': i, 'j': j, 'z_d': z_d}
        z_ds.append(the_dict)

# Jacobian linearisation

class f_jacob:
    def __init__(self, x_ai, x_aj, m_x):
        self.x_ai = x_ai.reshape(-1, 1)
        self.x_aj = x_aj.reshape(-1, 1)
        self.m_x = m_x.reshape(-1, 1)

    def value(self, x_d):
        x_d = x_d.reshape(-1, 1)
        d_i = np.linalg.norm(self.x_ai - x_d)
        d_j = np.linalg.norm(self.x_aj - x_d)
        return d_i - d_j

    def derivative(self, x_d):
        x_d = x_d.reshape(-1, 1)
        d_i = np.linalg.norm(self.x_ai - x_d)
        d_j = np.linalg.norm(self.x_aj - x_d)
        derivative = (x_d - self.x_ai) / d_i - (x_d - self.x_aj) / d_j

        return derivative.reshape(-1, 1)

    def M(self):
        return self.derivative(self.m_x)

    def mean_n(self):
        return self.value(self.m_x) - np.matmul(self.derivative(self.m_x).T, self.m_x)


#                 vel_in_node
#                    +-+
#                    +-+
#                     |
#                     | v_k
#                     v       system_noise_in_node
#                   +---+           +---+
#                   | dt| dt_node   |   |
#                   +---+           +---+                               no. anchor-times
#                     |               |             +-----------------------------------------------+
#                     |               | S_k+1       |                                               |
#                     v               v             |                equality_node                  |
#             X_k   +---+   X_k+1   +---+ X'_k+1    |                    +---+  Xij_k+1             |  X'''_k+1
#             ----->| + |---------->| + |-----------+------------------->| = |----------------------+------->
#  state_in_node    +---+           +---+           |                    +---+                      |       state_out_node
#               add_vel_node  add_system_noise_node |                      |                        |
#                                                   |                      | Xij_k+1                |
#                                                   |                      v                        |
#                                                   |                    +---+                      |
#                                                   |                    | M | M_node               |
#                                                   |                    +---+                      |
#                                                   |                      |                        |
#                                                   |                      |                        |
#                                                   |                      v                        |
#                                                   |      +---+ N_node  +---+                      |
#                                                   |      | N |-------->| + | add_N_node           |
#                                                   |      +---+         +---+                      |
#                                                   |                      |                        |
#                                                   |                      v                        |
#                                                   |      +---+ O_k+1   +---+                      |
#                                                   |      |   |-------->| + | add_obs_noise_node   |
#                                                   |      +---+         +---+                      |
#                                                   |  obs_noise_node      |                        |
#                                                   +----------------------+------------------------+
#                                                                          | Z_k+1
#                                                                          v
#                                                                         +-+
#                                                                         +-+
#                                                                     obs_in_node

# Compute important variables
delta_t = t[1] - t[0]
dt = np.identity(3)*delta_t
m_x = np.array([1, 1, 1])

# Define the nodes
state_in_node = PriorNode(name='x_in')
add_vel_node = AdditionNode(name='+')
dt_node = MatrixNode(dt, name='dt')
vel_in_node = PriorNode(name='v_in')
add_system_noise_node = AdditionNode(name='+')
system_noise_in_node = PriorNode(GaussianMeanCovMessage([[0], [0], [0]], sys_noise), name="N_s")
add_system_noise_node = AdditionNode(name='+')
state_out_node = PriorNode(name='x_out')

M_nodes = []
add_N_nodes = []
N_nodes = []
add_obs_noise_nodes = []
obs_noise_nodes = []
obs_in_nodes = []
equality_nodes = []
for k in range(len(z_ds)):
    i = z_ds[k]['i']
    j = z_ds[k]['j']
    f = f_jacob(x_as[i], x_as[j], m_x)
    M = f.M()
    mean_n = f.mean_n()
    var_n = [[0]]
    M_nodes.append(MatrixNode(M, name='M'))
    add_N_nodes.append(AdditionNode(name='+'))
    N_nodes.append(PriorNode(GaussianMeanCovMessage(mean_n, var_n), name='N'))
    add_obs_noise_nodes.append(AdditionNode(name='+'))
    obs_noise_nodes.append(PriorNode(GaussianMeanCovMessage([[0]], obs_noise), name='N_o'))
    obs_in_nodes.append(PriorNode(name='z_d'))
    equality_nodes.append(EqualityNode(name='='))

# Connections
state_in_node.port_a.connect(add_vel_node.port_a)
add_vel_node.port_b.connect(dt_node.port_b)
dt_node.port_a.connect(vel_in_node.port_a)
add_system_noise_node.port_a.connect(add_vel_node.port_c)
system_noise_in_node.port_a.connect(add_system_noise_node.port_b)
equality_nodes[0].ports[0].connect(add_system_noise_node.port_c)

for k in range(len(z_ds)):
    M_nodes[k].port_a.connect(equality_nodes[k].ports[1])
    add_N_nodes[k].port_a.connect(M_nodes[k].port_b)
    add_N_nodes[k].port_b.connect(N_nodes[k].port_a)
    add_obs_noise_nodes[k].port_a.connect(add_N_nodes[k].port_c)
    add_obs_noise_nodes[k].port_b.connect(obs_noise_nodes[k].port_a)
    obs_in_nodes[k].port_a.connect(add_obs_noise_nodes[k].port_c)
    if k + 1 < len(z_ds):
        equality_nodes[k + 1].ports[0].connect(equality_nodes[k].ports[2])
    else:
        state_out_node.port_a.connect(equality_nodes[k].ports[2])

# draw the Factor graph
# draw_graph(state_in_node)

# set a (wrong) start state with high variances (low confidence)
start_state = GaussianMeanCovMessage(x0.reshape(-1, 1), 1e2 * np.identity(3))

# Create a list of estimated messages by updating all ports
estimated_state_list = [start_state]
estimated_x_ds_cov = []

for idx in range(len(t)):
    state_in_node.update_prior(estimated_state_list[-1])
    estimated_x_ds_cov.append(state_in_node.port_a.out_msg.cov)
    vel_in_node.update_prior(GaussianMeanCovMessage(v[idx].reshape(-1, 1), np.zeros((3, 3))))
    dt_node.port_b.update(GaussianMeanCovMessage)
    add_vel_node.port_c.update(GaussianMeanCovMessage)
    add_system_noise_node.port_c.update(GaussianMeanCovMessage)
    mean_x = add_system_noise_node.port_c.out_msg.mean
    add_system_noise_node.port_c.update(GaussianWeightedMeanInfoMessage)
    for obs in range(len(z_ds)):
        obs_in_nodes[obs].update_prior(GaussianMeanCovMessage([[z_ds[obs]['z_d'][idx]]], [[0]]))
        add_obs_noise_nodes[obs].port_a.update(GaussianMeanCovMessage)
        i = z_ds[obs]['i']
        j = z_ds[obs]['j']
        f = f_jacob(x_as[i], x_as[j], mean_x)
        M = f.M()
        mean_n = f.mean_n()
        var_n = [[0]]
        N_nodes[obs].update_prior(GaussianMeanCovMessage(mean_n, var_n))
        M_nodes[obs]._matrix = M.transpose()
        add_N_nodes[obs].port_a.update(GaussianMeanCovMessage)
        add_N_nodes[obs].port_a.update(GaussianWeightedMeanInfoMessage)
        M_nodes[obs].port_a.update(GaussianWeightedMeanInfoMessage)
        equality_nodes[obs].ports[2].update(GaussianWeightedMeanInfoMessage)
        if obs < len(z_ds)-1:
            equality_nodes[obs].ports[2].update(GaussianMeanCovMessage)
            mean_x = equality_nodes[obs].ports[2].out_msg.mean
            equality_nodes[obs].ports[2].update(GaussianWeightedMeanInfoMessage)
        else:
            estimated_state_list.append(equality_nodes[obs].ports[2].update(GaussianMeanCovMessage))

estimated_x_ds = np.array([x.mean for x in estimated_state_list]).squeeze(2)

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.plot(x_ds[:, 0], x_ds[:, 1], x_ds[:, 2], label="Original Trajectory")
ax.plot(estimated_x_ds[:, 0], estimated_x_ds[:, 1], estimated_x_ds[:, 2], label="Estimated Trajectory")
ax.legend(loc="best")
ax.set_title("Trajectories")
plt.savefig('Trajectories.pdf')

fig, ax = plt.subplots(nrows=3)
#ax_twin = [a.twinx() for a in ax]
ax[0].plot(t, x_ds[:-1, 0])
ax[0].plot(t, estimated_x_ds[1:, 0])
#ax_twin[0].plot(t, estimated_x_ds_cov[1:, 0, 0], color="r")
ax[1].plot(t, x_ds[:-1, 1])
ax[1].plot(t, estimated_x_ds[1:, 1])
#ax_twin[1].plot(t, estimated_x_ds_cov[1:, 1, 1], color="r")
ax[2].plot(t, x_ds[:-1, 2])
ax[2].plot(t, estimated_x_ds[1:, 2])
#ax_twin[2].plot(t, estimated_x_ds_cov[1:, 2, 2], color="r")
ax[0].set_title('x coordinate with time')
ax[1].set_title('y coordinate with time')
ax[2].set_title('z coordinate with time')
fig.tight_layout(pad=1.0)
plt.savefig('x, y and z with time.pdf')
plt.show()