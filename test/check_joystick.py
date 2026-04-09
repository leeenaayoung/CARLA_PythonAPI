import pygame

def main():
    pygame.init()
    pygame.joystick.init()
    pygame.event.pump()

    n = pygame.joystick.get_count()
    print(f"connected joysticks: {n}")
    if n == 0:
        return

    js = pygame.joystick.Joystick(0)
    js.init()

    print("name:", js.get_name())
    print("num axes:", js.get_numaxes())
    print("num buttons:", js.get_numbuttons())
    print("num hats:", js.get_numhats())
    print("num balls:", js.get_numballs())

    print("press/operate buttons/axes to see realtime values (Ctrl+C to stop)")
    while True:
        pygame.event.pump()

        # 버튼
        for i in range(js.get_numbuttons()):
            v = js.get_button(i)
            if v:
                print(f"[button] index={i} pressed")

        # 축
        for i in range(js.get_numaxes()):
            v = js.get_axis(i)
            if abs(v) > 0.05:
                print(f"[axis] index={i} value={v:.3f}")

        pygame.time.wait(100)

if __name__ == "__main__":
    main()
