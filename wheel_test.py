import pygame
pygame.init()
pygame.joystick.init()

js = pygame.joystick.Joystick(0)
js.init()

print("num axes:", js.get_numaxes())

while True:
    pygame.event.pump()
    values = [js.get_axis(i) for i in range(js.get_numaxes())]
    print(values)
