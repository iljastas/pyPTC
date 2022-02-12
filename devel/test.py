import pygame

pygame.init()

screen = pygame.display.set_mode((400, 400))

x = [5, 5, 4, 5]
y = [2, 3, 4, 4]


def draw(x, y):
    block = 30

    coord = zip(x, y)

    for _x, _y in coord:

        _x1 = _x * block
        _y1 = _y * block
        x2 = _x1 + block
        y2 = _y1 + block

        # pygame.draw.rect(screen, 0xffffff, (_x1, _y1, x2, y2))
        pygame.draw.rect(screen, 0xffffff, (_x1, _y1, block, block))

running = True

while running:

    for event in pygame.event.get():

        if event.type == pygame.QUIT:
            running = False
    draw(x, y)
    pygame.display.flip()

pygame.quit()
quit()