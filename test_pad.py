import os
import time

try:
    import pygame
except ImportError:
    print("Installing pygame...")
    os.system("pip3 install pygame-ce --quiet")
    import pygame

pygame.init()
pygame.joystick.init()

count = pygame.joystick.get_count()
print(f"SDL2 found {count} joysticks.")

if count == 0:
    print("No joysticks found by SDL2 either. macOS is dropping it at the HID level.")
    os._exit(0)

joysticks = [pygame.joystick.Joystick(i) for i in range(count)]
for j in joysticks:
    j.init()
    print(f"[{j.get_id()}] {j.get_name()} | Axes: {j.get_numaxes()} | Hats: {j.get_numhats()} | Buttons: {j.get_numbuttons()}")

print("\n--- Move sticks now (testing for 5 seconds) ---")
start = time.time()
while time.time() - start < 5:
    pygame.event.pump()
    for j in joysticks:
        for a in range(j.get_numaxes()):
            val = j.get_axis(a)
            if abs(val) > 0.1:
                print(f"SDL2 Axis {a} active! Value: {val:.2f}")
    time.sleep(0.05)
print("Done testing.")
pygame.quit()
