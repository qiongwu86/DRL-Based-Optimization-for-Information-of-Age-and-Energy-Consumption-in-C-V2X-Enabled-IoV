import socket
import os
import time
import gym
from gym import spaces
import numpy as np
import torch
import matplotlib.pyplot as plt
host = '127.0.0.1'
port = 8889
N=1
def pad_action(act, act_param):
    params = []
    for j in range(N):
        param = [np.zeros((1,), dtype=np.float32), np.zeros((1,), dtype=np.float32), np.zeros((1,), dtype=np.float32)]
        param[int(act[j])] = act_param[j]
        params.append(param)

    return (act, params)


def pad_act(act,act_param):
    merged_list = []
    for item in act:
        merged_list.append(int(item))
    merged_list += act_param
    return merged_list


def evaluate(env, agent, episodes=1000):
    returns = []
    timesteps = []
    for i in range(episodes):
        print(i)
        if i % episodes == 0:
            state, reward = env.reset()
            state = np.array(state, dtype=np.float32, copy=False)

            act = []
            act_param = []
            all_action_parameters = []
            for k in range(len(reward)):  # 初始获得N个动作
                acti, act_parami, all_action_parametersi = agent.act(state[k * 6:(k + 1) * 6 - 1])
                act.append(acti)
                act_param.append(act_parami)
                all_action_parameters.append(all_action_parametersi)

        episode_reward = 0.
        agent.start_episode()
        trans = 0

        terminal = False
        total_reward = 0.

        while trans < 10:
            act_param_tensor = torch.Tensor(act_param)  # 将列表 act_param 转换为张量
            act_param_tensor = act_param_tensor.sigmoid()  # 在张量上调用 sigmoid() 方法
            act_param_tensor = act_param_tensor.detach()  # 分离张量，使其成为可修改的
            act_param = act_param_tensor.tolist()  # 将张量 act_param_tensor 转换为列表

            action_ = pad_act(act, act_param)
            (next_state, steps), next_reward, terminal, _ = env.step(action_)
            reselection = [next_state[i] for i in range(len(next_state)) if (i + 1) % 6 == 0]  # 提取车辆RCt是否为0的信息
            rese_ind = [i for i in range(len(reselection)) if reselection[i] == 1]  # 提取RCt为0的车辆编号

            for k in range(len(rese_ind)):
                re_k = int(rese_ind[k])  # 提取是第几辆车需要重选
                next_acti, next_act_parami, next_all_action_parametersi = agent.act(
                    next_state[re_k * 6:(re_k + 1) * 6 - 1])

                trans += 1
                act[re_k] = next_acti
                act_param[re_k] = next_act_parami
                all_action_parameters[re_k] = next_all_action_parametersi
                reward[re_k] = next_reward[re_k]
                state[re_k * 6:(re_k + 1) * 6 - 1] = next_state[re_k * 6:(re_k + 1) * 6 - 1]
                episode_reward += reward[re_k]

                total_reward += reward[re_k]
        timesteps.append(trans)
        returns.append(total_reward)
    # return np.column_stack((returns, timesteps))
    return np.array(returns)


# element = ((spaces.Box(low= 0.0, high=1.0, shape=(1,)),))
# repeated_element = tuple(element for _ in range(3))

class ENV1:

    def __init__(self, addr_in, port_in):

        self.action_space = spaces.Tuple((spaces.Discrete(3 * N),
                                        spaces.Box(low=0.0, high=1.0, shape=(1,)),
                                        spaces.Box(low=0.0, high=1.0, shape=(1,)),
                                        spaces.Box(low=0.0, high=1.0, shape=(1,)),
                                        ))
        self.observation_space = spaces.Tuple([spaces.Box(low=0, high=1, shape=(5,), dtype=float),  # 连续状态空间，范围为-1到1，维度为9
                                                spaces.Discrete(200*N)  # 离散状态空间，共有200个离散动作
                                                ])

        # 定义环境内部状态变量
        self.state = None
        # self.episode_rewards = []
        self.addr = addr_in
        self.port_in = port_in
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((addr_in, port_in))
        self.server_socket.listen(1)
        self.client_socket, _ = self.server_socket.accept()
        print("Construct env, socket is listening at {0}".format(port_in))
        self.steps = 0

    def reset(self):

        # 发送 'reset' 消息到服务器，获取初始状态
        try:
            self.client_socket.sendall(b'reset')
        except ValueError:
            print('无法将数据输出')

        # 接收来自MATLAB的数据
        # next_state = self.client_socket.recv(1024).decode()
        # reward = self.client_socket.recv(1024).decode()
        #
        # reward = reward.strip('[]')  # 去掉方括号
        # reward = np.fromstring(reward, sep=' ', dtype=np.float32)
        # reward = reward.astype(np.float32)
        #
        # string = next_state.strip('[]')  # 去掉方括号
        # next_state = np.fromstring(string, sep=' ')
        # next_state = next_state.astype(np.float32)
        next_state, reward = self.client_socket.recv(10240).decode().split(',')
        # reward = reward.strip('[]')  # 去掉方括号
        # reward = np.float(reward)
        reward = reward.strip('[]')  # 去掉方括号
        reward = np.fromstring(reward, sep=' ', dtype=np.float32)
        reward = reward.astype(np.float32)

        string = next_state.strip('[]')  # 去掉方括号
        next_state = np.fromstring(string, sep=' ', dtype=np.float32)
        next_state = next_state.astype(np.float32)

        return next_state, reward

    def step(self, action):
        # 向MATLAB发送数据
        action = str(action).encode()
        # print(action)
        self.client_socket.sendall(action)

        # 发完消息需要等待matlab的通知才能开始接收，否则一直阻塞
        next_state = self.client_socket.recv(10240).decode()
        reward = self.client_socket.recv(10240).decode()
        # next_state, reward = self.client_socket.recv(1024).decode().split(',')
        reward = reward.strip('[]')  # 去掉方括号
        reward = np.fromstring(reward, sep=' ', dtype=np.float32)
        reward = reward.astype(np.float32)

        string = next_state.strip('[]')  # 去掉方括号
        next_state = np.fromstring(string, sep=' ')
        next_state = next_state.astype(np.float32)

        ret_steps = self.steps  # 保存当前的步数
        self.steps += 1

        done = (sum(reward) == 0)
        return (next_state,ret_steps), reward, done, {}


    def _calculate_reward(self, state, action):
        ## input: action & state
        ## output： reward（state， reward）
        return 0.0



env = ENV1(host, port)

from agents.pdqn_multipass import MultiPassPDQNAgent
agent_class = MultiPassPDQNAgent
save_dir = "results/platform"
title = 'PDDQN'

agent = agent_class(
        env.observation_space.spaces[0], env.action_space,
        batch_size=128,    #128
        learning_rate_actor=0.001,    #0.001
        learning_rate_actor_param=0.001,     #0.0001
        epsilon_steps=1000,  #100回合后贪婪
        gamma=0.99,
        tau_actor=0.01,
        tau_actor_param=0.001,
        clip_grad=10.0,
        indexed=False,
        weighted=False,
        average=False,
        random_weighted=False,
        initial_memory_threshold=1000,
        use_ornstein_noise=True,
        replay_memory_size=10000,
        epsilon_final=0.01,
        inverting_gradients=True,
        actor_kwargs={'hidden_layers': [100],
                      'action_input_layer': 0, },
        actor_param_kwargs={'hidden_layers': [100],
                            'squashing_function': False,
                            'output_layer_init_std': 0.0001, },#0.0001
        zero_index_gradients=False,
        seed=1)


if __name__ == '__main__':
    agent.load_models(r'E:\Code\6\MPDQN\results\RHO100')
    # print(agent)
    evaluation_episodes = 1000
    # max_steps = 10000
    start_time = time.time()
    Reward_plot = []

    if evaluation_episodes > 0:
        print("Evaluating agent over {} episodes".format(evaluation_episodes))
        agent.epsilon_final = 0.
        agent.epsilon = 0.
        agent.noise = None
        evaluation_returns = evaluate(env, agent, evaluation_episodes)
        print("Ave. evaluation return =", sum(evaluation_returns) / len(evaluation_returns))
        file_path = os.path.join(save_dir, title)
        np.save(file_path, evaluation_returns)


