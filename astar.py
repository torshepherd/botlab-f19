import numpy as np

'''
Initialize map
'''
gridmap = np.ones([20,20])
gridmap[1:-1,1:-1] = 0

obstacle_topleft = np.array([np.random.randint(2,17), np.random.randint(2,17)])
obstacle_botright = np.array([np.random.randint(obstacle_topleft[0]+1,18), np.random.randint(obstacle_topleft[0]+1,18)])

gridmap[obstacle_topleft[0]:obstacle_botright[0],obstacle_topleft[1]:obstacle_botright[1]] = 1
print(gridmap)

'''
Brushfire algorithm to punish being close to obstacles
'''
