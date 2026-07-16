import pymunk
import pymunk.pygame_util
import pygame

def create_ball(space, pos, radius=15, mass=1.0):
    body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, radius))
    body.position = pos
    shape = pymunk.Circle(body, radius)
    shape.elasticity = 0.999
    shape.friction = 0.2
    space.add(body, shape)
    return body

def create_rope(space, body, anchor_point, length):
    joint = pymunk.PinJoint(body, space.static_body, (0, 0), anchor_point)
    joint.distance = length
    space.add(joint)

def newtons_cradle(space, start_x, y, num_balls, spacing=35):
    balls = []
    for i in range(num_balls):
        x = start_x + i * spacing
        if i == 0:
            ball_mass = 7.0
        else:
            ball_mass = 1.0
        ball = create_ball(space, (x, y + 150),mass=ball_mass)
        create_rope(space, ball, (x, y), 150)
        balls.append(ball)
    return balls

def run_simulation():
    pygame.init()
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Newton's Cradle - PyMunk Simulation")
    clock = pygame.time.Clock()

    space = pymunk.Space()
    space.gravity = (0, 900)

    draw_options = pymunk.pygame_util.DrawOptions(screen)
    balls = newtons_cradle(space, start_x=250, y=100, num_balls=10)

    # Pull the leftmost ball to start the cradle
    balls[0].position = (balls[0].position.x - 120, balls[0].position.y - 100)
    balls[0].velocity = (0, 0)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        screen.fill((255, 255, 255))
        space.step(1 / 60.0)
        space.debug_draw(draw_options)
        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    run_simulation()
