from gps_module import GPS
import pygame
import time

def main():

    gps = GPS()

    if not gps.gps:
        print("GPS has not been found. try changing the PORT")
        return


    clock = pygame.time.Clock()

    t_end = time.time() + 60 * 1

    while time.time() < t_end:
        clock.tick_busy_loop(2)  # fps = 2
        print(gps.readMsg)

if __name__ == "__main__":
    main()
