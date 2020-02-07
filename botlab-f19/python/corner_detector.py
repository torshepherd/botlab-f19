import pygame, numpy as np, time

R = [255,   0,   0]
G = [  0, 255,   0]
B = [  0,   0, 255]
K = [  0,   0,   0]
W = [255, 255, 255]

COLORS = [W,K,R,B]

preformat_path = 'C:/Users/Tor/Desktop/School/Michigan/550/botlab/voronoi/saved_distance_map0.map'
map_path = 'C:/Users/Tor/Desktop/School/Michigan/550/botlab/voronoi/formatted_map.map'
output_path = 'C:/Users/Tor/Desktop/School/Michigan/550/botlab/voronoi/corner_list.wp'
with open(preformat_path) as file:
    with open(map_path, 'w') as tofile:
        for line in file:
            line = line.rstrip()
            if line:
                tofile.write(line+'\n')

def corner_detector_vector(distance_map):
    corner_list = []
    diffs = []
    for p in range(1,4):
        diffs.append(4*distance_map[4:-4,4:-4] - distance_map[4-p:-4-p,4:-4] - distance_map[4:-4,4+p:-4+p] - distance_map[4+p:-4+p,4:-4] - distance_map[4:-4,4-p:-4-p])
    corner_mask_1 = (np.round(diffs[0],2) == 0.15) * (np.round(diffs[1],2) == 0.30)
    corner_mask_2 = (np.round(diffs[0],2) == 0.10) * (np.round(diffs[1],2) == 0.25) * (np.round(diffs[2],2) == 0.40)
    corner_mask = (corner_mask_1 + corner_mask_2) * (distance_map[4:-4,4:-4] < 0.4)
    for j, row in enumerate(corner_mask):
        for i, cell in enumerate(row):
            if cell and (corner_mask[j-1,i] or corner_mask[j,i-1]):
                corner_mask[j,i] = 0
    indices = np.vstack(np.where(corner_mask)) + 4
    indices[[0,1]] = indices[[1,0]]
    corner_list = list(np.transpose(indices))
    return corner_list

def draw_map(cell_map, screen_size, screen, color_list, crop_idx):
    if crop_idx != None:
        cell_map = cell_map[crop_idx[0]:crop_idx[1], crop_idx[2]:crop_idx[3]]
    cell_dim = (size/np.shape(cell_map)).astype(int)
    cell_w = int(cell_dim[0])
    cell_h = int(cell_dim[1])
    for j, row in enumerate(cell_map):
        for i, cell in enumerate(row):
            pygame.draw.rect(screen, color_list[int(cell)], pygame.Rect(i*cell_w, j*cell_h, cell_w, cell_h))

def draw_circles(surface, color, center_list_idx, cell_map, screen_size, radius, crop_idx, width=0):
    if crop_idx != None:
        cell_map = cell_map[crop_idx[0]:crop_idx[1], crop_idx[2]:crop_idx[3]]
    dim = np.shape(cell_map)
    cell_w = int(screen_size[0]/(2*dim[0]))
    cell_h = int(screen_size[1]/(2*dim[1]))
    for center_i in center_list_idx:
        if crop_idx != None:
            center_i = [center_i[0] - crop_idx[0], center_i[1] - crop_idx[2]]
        center = [cell_w + int((screen_size[0]/dim[0])*center_i[0]), cell_h + int((screen_size[1]/dim[1])*center_i[1])]
        pygame.draw.circle(surface, color, center, radius, width)

if __name__ == '__main__':
    pygame.init()
    size = np.array((800, 800))
    screen = pygame.display.set_mode(size)

    clock = pygame.time.Clock()

    map_from_file = np.loadtxt(map_path, delimiter=' ')
    path_map = map_from_file * (map_from_file > 0)
    path_map *= 20
    color_list_2 = np.transpose(np.tile(np.linspace(0,255,13).round().astype(int), (3,1)))

    starttime = time.time()
    corner_list = corner_detector_vector(map_from_file)
    print('Found corners in {} ms.'.format(round((time.time()-starttime)*1000),2))
    with open(output_path, 'w') as file:
        for i, corner in enumerate(corner_list):
            if i != 0:
                file.write('\n')
            formatted_corner = '{}, {}'.format(corner[0],corner[1])
            file.write(formatted_corner)

    # PYGAME DISPLAY LOOP
    crop_idx = None
    img = 1
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

            if event.type == pygame.MOUSEBUTTONDOWN:
                pos = np.floor(np.array(pygame.mouse.get_pos())/4).astype(int)
                print("Location: {}    Value: {}    BB_Value: {}".format(pos, map_from_file[pos[1]][pos[0]], path_map[pos[1]][pos[0]]))
                starttime = time.time()
                corner_list = corner_detector_vector(map_from_file)
                print(time.time()-starttime)
                img += 1

        screen.fill(W)

        if img%2:
            draw_map(path_map, size, screen, color_list_2, crop_idx)
            draw_circles(screen, R, corner_list, path_map, size, 5, crop_idx)
        else:
            draw_map(path_map, size, screen, color_list_2, crop_idx)

        pygame.display.flip()
        clock.tick(60)