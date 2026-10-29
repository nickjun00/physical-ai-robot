"""Run with: .venv/bin/python pygame_app.py"""
import argparse
import math
import pygame
from robot_world import World, WIDTH, HEIGHT, ROBOT_RADIUS


def main():
    # Optional terminal flags let us run a short, automatically closing test.
    parser = argparse.ArgumentParser(description="2D robot simulator")
    parser.add_argument("--frames", type=int, default=0, help="Exit after N frames (smoke test)")
    parser.add_argument("--auto", action="store_true", help="Start in automatic mode")
    args = parser.parse_args()
    pygame.init()
    # Visualization lives here; World owns the movement and navigation rules.
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("2D Robot Simulator")
    font = pygame.font.Font(None, 23)
    clock = pygame.time.Clock()
    world = World()
    if args.auto:
        world.toggle_mode()
    running, frames = True, 0
    try:
        while running:
            # Each frame: read input -> update the model -> draw its new state.
            # Clamp pauses so dragging the window does not cause a huge jump.
            dt = min(clock.tick(60) / 1000.0, 0.05)
            for event in pygame.event.get():
                # KEYDOWN fires on a press; mode toggles should happen once,
                # not every frame that the Space key remains held down.
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        world.toggle_mode()
                    elif event.key == pygame.K_r:
                        world = World()
            keys = pygame.key.get_pressed()
            # Held keys give continuous movement. Each axis is -1, 0, or +1;
            # pressing opposite directions together cancels that axis.
            direction = (int(keys[pygame.K_d] or keys[pygame.K_RIGHT]) - int(keys[pygame.K_a] or keys[pygame.K_LEFT]),
                         int(keys[pygame.K_s] or keys[pygame.K_DOWN]) - int(keys[pygame.K_w] or keys[pygame.K_UP]))
            world.update(dt, direction)
            # Clear the previous frame before drawing obstacles and the robot.
            screen.fill((240, 242, 247))
            for obstacle in world.obstacles:
                pygame.draw.rect(screen, (210, 55, 60), (obstacle.x, obstacle.y, obstacle.width, obstacle.height))
            if world.path:
                pygame.draw.lines(screen, (150, 165, 180), False, [world.position] + world.path, 2)
            pygame.draw.circle(screen, (35, 175, 75), world.target, 12)
            reading = world.sensor()
            # Draw the sensor along the heading, ending at its measured hit.
            endpoint = (world.position[0] + math.cos(world.heading) * reading,
                        world.position[1] + math.sin(world.heading) * reading)
            pygame.draw.line(screen, (230, 165, 20), world.position, endpoint, 2)
            pygame.draw.circle(screen, (35, 110, 225), world.position, ROBOT_RADIUS)
            nose = (world.position[0] + math.cos(world.heading) * ROBOT_RADIUS,
                    world.position[1] + math.sin(world.heading) * ROBOT_RADIUS)
            pygame.draw.line(screen, (255, 255, 255), world.position, nose, 3)
            lines = [f"Mode: {world.mode}    Position: ({world.position[0]:.1f}, {world.position[1]:.1f})    Speed: {world.speed:.1f} px/s",
                     f"Forward sensor: {reading:.1f} px (from centre; maximum 300 px)",
                     "WASD / arrows: move    Space: manual/auto    R: reset    Esc: quit",
                     world.message]
            panel = pygame.Surface((WIDTH, 102), pygame.SRCALPHA)
            # Alpha (225 out of 255) makes the information panel translucent.
            panel.fill((255, 255, 255, 225))
            screen.blit(panel, (0, HEIGHT - 102))
            for i, line in enumerate(lines):
                screen.blit(font.render(line, True, (25, 30, 40)), (12, HEIGHT - 96 + i * 24))
            pygame.display.flip()
            # flip() presents the completed frame in the visible window.
            frames += 1
            if args.frames and frames >= args.frames:
                running = False
    finally:
        # Release Pygame resources even if the loop raises an exception.
        pygame.quit()


if __name__ == "__main__":
    # Launch only when run directly, not when another file imports this module.
    main()
