import pygame

def init_pygame():
    # Initialisation de Pygame
    pygame.init()

    # Créer une fenêtre pour capturer les événements clavier
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Controller")

def get_values(running):
    key=""
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        
        if event.type == pygame.KEYDOWN:
            print(f"Touche appuyée : {pygame.key.name(event.key)}")
            key = pygame.key.name(event.key)

    return running, key

def quit_pygame(running):
    # Quitter pygame
    pygame.quit()


if __name__=="__main__":

    init_pygame()
    running=True
    while running:
        running,key = get_values(running)

    quit_pygame(running)