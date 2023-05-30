# import numpy as np
# import rospy
# from sensor_msgs.msg import LaserScan
# from std_msgs.msg import Int32
# from nav_msgs.msg import OccupancyGrid
# import matplotlib.pyplot as plt
# import time

# # Define ROS node
# rospy.init_node('q_learning_autonomous_driving')

# # ROS topics
# scan_topic = '/scan'
# map_topic = '/map'
# action_topic = '/action'

# # Define state and action space
# num_states = 10
# num_actions = 4

# # Initialize Q-table
# Q = np.zeros((num_states, num_actions))

# # Define reward function
# def get_reward(state, action):
#     # Get state-specific reward
#     state_reward = 0

#     # Get action-specific reward
#     action_reward = 1

#     # Define additional reward criteria
#     # ...

#     # Combine rewards
#     total_reward = state_reward + action_reward
#     return total_reward

# # Define callback function for LiDAR scan data
# def scan_callback(msg):
#     # Process LiDAR scan data and determine current state
#     ranges = np.array(msg.ranges)
#     valid_ranges = ranges[np.isfinite(ranges)]
#     state = int(np.mean(valid_ranges) * num_states)

#     # Exploration vs. exploitation trade-off
#     if np.random.uniform(0, 1) < epsilon:
#         action = np.random.randint(0, num_actions)  # exploration
#     else:
#         action = np.argmax(Q[state, :])  # exploitation

#     # Publish action
#     action_msg = Int32()
#     action_msg.data = action
#     action_pub.publish(action_msg)

#     # Perform action and observe new state
#     # ...
#     next_state = 0
#     reward = get_reward(state, action)
#     done = False
#     discount_factor = 0.99

#     # Update Q-table
#     learning_rate = initial_learning_rate * (1 - episode/num_episodes) + final_learning_rate * (episode/num_episodes)
#     Q[state, action] = (1 - learning_rate) * Q[state, action] + learning_rate * (reward + discount_factor * np.max(Q[next_state, :]))
#     print(Q)

#     # if detect_obstacle(msg):
#     #     avoid_obstacle(action)

#     # Save Q-table to a file
#     np.savetxt('q_table.txt', Q)

# # Define obstacle detection function
# def detect_obstacle(scan_msg):
#     # Extract necessary information from the scan message
#     ranges = scan_msg.ranges
#     angle_min = scan_msg.angle_min
#     angle_max = scan_msg.angle_max
#     angle_increment = scan_msg.angle_increment

#     # Define obstacle detection parameters
#     min_distance = 1.0  # Minimum distance to consider an obstacle
#     obstacle_detected = False

#     # Iterate through the scan ranges
#     for i, scan_range in enumerate(ranges):
#         angle = angle_min + i * angle_increment

#         # Check if the range is within the minimum distance threshold
#         if scan_range < min_distance:
#             obstacle_detected = True
#             break

#     return obstacle_detected

# # Define callback function for map data
# def map_callback(msg):
#     global episode
#     global rewards_per_episode

#     # Process map data and update Q-learning
#     # ...
#     episode += 1

#     # Save rewards per episode to a file
#     np.savetxt('rewards_per_episode.txt', rewards_per_episode)

#     if episode == num_episodes:
#         # Plot rewards per episode
#         plt.plot(range(1, num_episodes+1), rewards_per_episode)
#         plt.xlabel('Episode')
#         plt.ylabel('Total Reward')
#         plt.title('Q-learning Rewards')
#         plt.show()

# # Create ROS subscribers and publishers
# scan_sub = rospy.Subscriber(scan_topic, LaserScan, scan_callback)
# map_sub = rospy.Subscriber(map_topic, OccupancyGrid, map_callback)
# action_pub = rospy.Publisher(action_topic, Int32, queue_size=1)

# # Initialize Q-learning parameters
# initial_learning_rate = 0.5
# final_learning_rate = 0.1
# num_episodes = 1000
# epsilon = 0.1
# epsilon_decay = 0.99

# # Q-learning algorithm
# rewards_per_episode = []  # Rewards per episode
# episode = 0

# while episode < num_episodes:
#     state = 0  # initial state
#     action = 0
#     done = False
#     next_state = 0
#     episode_reward = 0

#     while not done:
#         # Exploration vs. Exploitation
#         if np.random.uniform(0, 1) < epsilon:
#             action = np.random.randint(0, num_actions)
#         else:
#             action = np.argmax(Q[state, :])

#         # Perform action and observe new state
#         # ...
#         next_state = 0
#         reward = get_reward(state, action)
#         episode_reward += reward
#         discount_factor = 0.99

#         # Update Q-table
#         learning_rate = initial_learning_rate * (1 - episode/num_episodes) + final_learning_rate * (episode/num_episodes)
#         Q[state, action] = (1 - learning_rate) * Q[state, action] + learning_rate * (reward + discount_factor * np.max(Q[next_state, :]))

#         state = next_state

#         # Decay epsilon
#         epsilon *= epsilon_decay

#         if done:
#             rewards_per_episode.append(episode_reward)

#     # Save Q-table to a file
#     np.savetxt('q_table.txt', Q)

# # Spin ROS node
# rospy.spin()


        