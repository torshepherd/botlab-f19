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

map_from_file = np.loadtxt('saved_distance_map0.map', delimiter=' ')
border_map = map_from_file + 1
border_map = border_map * (border_map == 1)

# VORONOI
pt_list = []
color_list = []

def add_pt(coords):
    color = B
    pt_list.append(coords)
    color_list.append(color)
    calc_voronoi()

def calc_voronoi():
    for i in range(size[0]):
        pass

def draw_rect(topleft, width, height, color):
    x, y = topleft
    c = pygame.Rect(x, y, width, height)
    pygame.draw.rect(screen, color, c)

def draw_map(semantic_map, screen_size, screen):
    cell_dim = (size/np.shape(semantic_map)).as_type(int)
    for j, row in enumerate(semantic_map):
        for i, cell in enumerate(row):
            pygame.draw.rect(screen, color, pygame.Rect(i*cell_dim[0], j*cell_dim[1], cell_dim[0], cell_dim[1]))

# PYGAME DISPLAY LOOP
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            pos = 5 + 10*np.floor(np.array(pygame.mouse.get_pos())/10)
            add_pt(pos)

    screen.fill(W)
    draw_map(border_map, size, screen)

    pygame.display.flip()
    clock.tick(60)