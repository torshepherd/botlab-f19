import pygame, numpy as np, time

R = [255,   0,   0]
G = [  0, 255,   0]
B = [  0,   0, 255]
K = [  0,   0,   0]
W = [255, 255, 255]

COLORS = [W,K,R,B]

pygame.init()
size = np.array((800, 800))
screen = pygame.display.set_mode(size)

clock = pygame.time.Clock()

map_from_file = np.loadtxt('saved_distance_map0.map', delimiter=',')
distance_map = map_from_file + 1
border_map = distance_map * (distance_map == 1)

# VORONOI
color_list_1 = [np.array(W),np.array(K)]

def add_pt(coords):
    color = B
    pt_list.append(coords)
    color_list_1.append(color)
    calc_voronoi()

def list_generators(cell_map):
    generator_cells = []
    cell_map_out = np.copy(cell_map)
    for j, row in enumerate(cell_map_out):
        for i, cell in enumerate(row):
            if int(cell) == 1:
                generator_cells.append(np.array([i,j]))
                cell_map_out[j][i] = len(generator_cells)
    return [np.array(generator_cells), cell_map_out]

def voronoi_cell_map(cell_map, generator_cells):
    cell_map_out = np.copy(cell_map)
    for j, row in enumerate(cell_map_out):
        for i, cell in enumerate(row):
            distances = (np.sqrt(np.matmul(((generator_cells - np.array([i,j])) ** 2), np.array([1,1]))))
            if (np.min(distances)) == 0:
                continue
            min_2 = np.partition(distances, 1)[0:2]
            min_ids = np.where(distances==np.min(distances))
            if (len(min_ids) == 1):
                i_near, j_near = generator_cells[min_ids][0]
                cell_map_out[j][i] = cell_map_out[j_near][i_near]
    return cell_map_out

def draw_map(cell_map, screen_size, screen, color_list):
    cell_dim = (size/np.shape(cell_map)).astype(int)
    cell_w = int(cell_dim[0])
    cell_h = int(cell_dim[1])
    for j, row in enumerate(cell_map):
        for i, cell in enumerate(row):
            pygame.draw.rect(screen, color_list[int(cell)], pygame.Rect(i*cell_w, j*cell_h, cell_w, cell_h))

def draw_generators(cell_map, screen_size, screen, generator_cells):
    cell_dim = (size/np.shape(cell_map)).astype(int)
    cell_w = int(cell_dim[0])
    cell_h = int(cell_dim[1])
    for cell in generator_cells:
        pygame.draw.rect(screen, color_list_1[1], pygame.Rect(cell[0]*cell_w, cell[1]*cell_h, cell_w, cell_h))
'''
def bounding_box_shrink(distance_map):
    new_map = np.copy(distance_map)
    prev_bounding_box = np.array([0,0])
    while True:
        guess = np.min(new_map)
        print("Guess: {}".format(guess))
        id_row, id_col = np.where(new_map == guess)
        bounding_box = np.array([np.max(id_col) - np.min(id_col), np.max(id_row) - np.min(id_row)])
        print("BB: {}".format(bounding_box))
        if (np.abs(bounding_box - prev_bounding_box) > 2).all():
            guess = prev_guess
            break
        else:
            new_map = new_map * (new_map < guess)
        prev_bounding_box = bounding_box
        prev_guess = guess

    return (distance_map >= guess) * np.copy(distance_map)
    '''

def corner_detector(distance_map):
    corner_list = []
    for j_0 in range(len(distance_map) - 4):
        j = j_0 + 2
        row = distance_map[j]
        for i_0 in range(len(row) - 4):
            i = i_0 + 2
            cell = row[i]
            diffs = []
            diffs.append(round(cell-distance_map[j-1][i],2))
            diffs.append(round(cell-distance_map[j][i+1],2))
            diffs.append(round(cell-distance_map[j+1][i],2))
            diffs.append(round(cell-distance_map[j][i-1],2))
            if diffs.count(0.05) == 3:
                # Candidate for dead end width 1
                diffs = []
                diffs.append(round(cell-distance_map[j-2][i],2))
                diffs.append(round(cell-distance_map[j][i+2],2))
                diffs.append(round(cell-distance_map[j+2][i],2))
                diffs.append(round(cell-distance_map[j][i-2],2))
                if diffs.count(0.10) == 3:
                    corner_list.append([i,j])
            elif diffs.count(0.05) == 2:
                # Candidate for dead end width 2
                diffs = []
                diffs.append(round(cell-distance_map[j-2][i],2))
                diffs.append(round(cell-distance_map[j][i+2],2))
                diffs.append(round(cell-distance_map[j+2][i],2))
                diffs.append(round(cell-distance_map[j][i-2],2))
                if diffs.count(0.10) == 2 and diffs.count(0.05) == 1:
                    # Further investigate
                    diffs = []
                    diffs.append(round(cell-distance_map[j-3][i],2))
                    diffs.append(round(cell-distance_map[j][i+3],2))
                    diffs.append(round(cell-distance_map[j+3][i],2))
                    diffs.append(round(cell-distance_map[j][i-3],2))
                    if diffs.count(0.15) == 2 and diffs.count(0.10) == 1:
                        if ([i-1,j] not in corner_list) and ([i,j-1] not in corner_list):
                            corner_list.append([i,j])
    return corner_list

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

def draw_circles(surface, color, center_list_in_map, map, screen_size, radius, width=1):
    dim = np.shape(map)
    cell_w = int(screen_size[0]/(2*dim[0]))
    cell_h = int(screen_size[1]/(2*dim[1]))
    for center_i in center_list_in_map:
        center = [cell_w + int((screen_size[0]/dim[0])*center_i[0]), cell_h + int((screen_size[1]/dim[1])*center_i[1])]
        pygame.draw.circle(surface, color, center, radius, width)

'''gen_cells, cell_map = list_generators(border_map)
num_colors = np.shape(gen_cells)[0]
for i in range(num_colors+1):
    color_list_1.append(np.random.randint(0,256,3))
v_map = voronoi_cell_map(cell_map, gen_cells)
'''
path_map = map_from_file + (map_from_file < 0) * 2
path_map *= 20
#path_map = path_map * (path_map > 0)
#bb_map = bounding_box_shrink(path_map)
dx = (np.diff(np.vstack([np.zeros([1,200]),path_map,np.zeros([1,200])]), 2, 0))
dy = (np.diff(np.hstack([np.zeros([200,1]),path_map,np.zeros([200,1])]), 2, 1))
grad = (dx + dy) / 2
grad = grad * (grad<0)
color_list_2 = np.transpose(np.tile(np.linspace(0,255,21).round().astype(int), (3,1)))

starttime = time.time()
corner_list = corner_detector_vector(map_from_file)
print(time.time()-starttime)

print(corner_list)

# PYGAME DISPLAY LOOP
img = 0
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = np.floor(np.array(pygame.mouse.get_pos())/4).astype(int)
            print("Location: {}    Value: {}    BB_Value: {}".format(pos, map_from_file[pos[1]][pos[0]], path_map[pos[1]][pos[0]]))
            img += 1
            #add_pt(pos)

    screen.fill(W)
    
    # VORONOI version
    #draw_map(v_map, size, screen, color_list)
    #draw_generators(border_map, size, screen, gen_cells)

    # Gradient version
    if img%2:
        #draw_map(2.5*(grad +3), size, screen, color_list_2)
        draw_map(path_map, size, screen, color_list_2)
        draw_circles(screen, R, corner_list, path_map, size, 5)
    else:
        draw_map(path_map, size, screen, color_list_2)

    pygame.display.flip()
    clock.tick(60)