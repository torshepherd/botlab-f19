import pygame, numpy as np

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
border_map = map_from_file + 1
border_map = border_map * (border_map == 1)

# VORONOI
color_list = [np.array(W),np.array(K)]

def add_pt(coords):
    color = B
    pt_list.append(coords)
    color_list.append(color)
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

def draw_map(cell_map, screen_size, screen):
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
        pygame.draw.rect(screen, color_list[1], pygame.Rect(cell[0]*cell_w, cell[1]*cell_h, cell_w, cell_h))

gen_cells, cell_map = list_generators(border_map)
num_colors = np.shape(gen_cells)[0]
for i in range(num_colors+1):
    color_list.append(np.random.randint(0,256,3))
v_map = voronoi_cell_map(cell_map, gen_cells)
path_map = 1

# PYGAME DISPLAY LOOP
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = 5 + 10*np.floor(np.array(pygame.mouse.get_pos())/10)
            #add_pt(pos)

    screen.fill(W)
    draw_map(v_map, size, screen)
    draw_generators(border_map, size, screen, gen_cells)

    pygame.display.flip()
    clock.tick(60)