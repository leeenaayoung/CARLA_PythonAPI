import pygame
from pygame.locals import ( K_p )

class KeyboradModeToggle:
    def __init__(self):
        self.toggle_mode_requested = False
        self.force_manual = False

    def parse_events(self):
        self.toggle_mode_requested = False
        for event in pygame.event.get():
            if event.type == pygame.KEYUP:
                if event.key == K_p:
                    self.toggle_mode_requested = True
