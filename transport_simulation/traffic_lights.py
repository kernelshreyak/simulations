import pygame
import random
import time
import sys
import os

# Initialize Pygame
pygame.init()

# Screen settings
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Traffic Intersection Simulation")

# Colors
BLACK   = (0, 0, 0)
WHITE   = (255, 255, 255)
RED     = (255, 0, 0)
GREEN   = (0, 255, 0)
YELLOW  = (255, 255, 0)
GRAY    = (100, 100, 100)

# Load collision sound (replace 'collision.wav' with your sound file path)
try:
    collision_sound = pygame.mixer.Sound('collision.mp3')
except FileNotFoundError:
    pygame.mixer.init()
    collision_sound = pygame.mixer.Sound(pygame.mixer.Sound(buffer=b'\x00\x00' * 1000))

# Intersection and road settings
INTERSECTION_X, INTERSECTION_Y = WIDTH // 2, HEIGHT // 2
ROAD_WIDTH = 100
LANE_WIDTH = ROAD_WIDTH // 2
CAR_WIDTH, CAR_HEIGHT = 30, 20

# Traffic light class
class TrafficLight:
    def __init__(self, x, y, direction):
        """Initialize a traffic light at (x, y) for the given direction ('N', 'S', 'E', 'W')."""
        self.x = x
        self.y = y
        self.direction = direction  # 'N', 'S', 'E', 'W'
        self.state = 'red'

    def draw(self):
        """Draw the traffic light with the direction letter inside."""
        color = {'green': GREEN, 'yellow': YELLOW, 'red': RED}[self.state]
        pygame.draw.circle(screen, color, (self.x, self.y), 15)
        font = pygame.font.SysFont(None, 20)
        text = font.render(self.direction, True, BLACK)
        text_rect = text.get_rect(center=(self.x, self.y))
        screen.blit(text, text_rect)

# Helper function to get the "stop line" coordinate from a traffic light’s position
def get_stop_line(light):
    radius = 15
    offset = 10  # Adjust to control how close/far vehicles stop from the light

    if light.direction == 'N':
        # Vehicles moving north (from bottom upward) stop when their front reaches below the light.
        return light.y + radius + offset
    elif light.direction == 'S':
        # Vehicles moving south stop when their front reaches above the light.
        return light.y - radius - offset
    elif light.direction == 'W':
        # Vehicles moving west stop when their front reaches to the right of the light.
        return light.x + radius + offset
    elif light.direction == 'E':
        # Vehicles moving east stop when their front reaches to the left of the light.
        return light.x - radius - offset

# Helper function to determine the front coordinate of a vehicle based on its direction.
def get_front_position(vehicle):
    if vehicle.direction == 'N':
        return vehicle.y           # Top edge for northbound
    elif vehicle.direction == 'S':
        return vehicle.y + CAR_HEIGHT  # Bottom edge for southbound
    elif vehicle.direction == 'E':
        return vehicle.x + CAR_WIDTH   # Right edge for eastbound
    elif vehicle.direction == 'W':
        return vehicle.x           # Left edge for westbound

# Vehicle class
class Vehicle(pygame.sprite.Sprite):
    def __init__(self, x, y, direction):
        """Initialize a vehicle at (x, y) moving in the given direction ('N', 'S', 'E', 'W')."""
        super().__init__()
        self.direction = direction
        self.speed = 2
        self.collision_time = 0       # Time when collision occurred
        self.collision_duration = 5   # Seconds to wait before removal after collision

        # Load sprite or fallback to a white rectangle.
        sprite_path = "Car.png"
        try:
            # Suppress libpng warnings
            devnull = open(os.devnull, 'w')
            sys.stderr = devnull
            self.original_image = pygame.image.load(sprite_path).convert_alpha()
            self.original_image = pygame.transform.scale(self.original_image, (CAR_WIDTH, CAR_HEIGHT))
            # Restore stderr to default after loading
            sys.stderr = sys.__stderr__
        except pygame.error:
            self.original_image = pygame.Surface((CAR_WIDTH, CAR_HEIGHT))
            self.original_image.fill(WHITE)

        # Rotate image based on direction.
        if self.direction == 'N':
            self.image = self.original_image
        elif self.direction == 'S':
            self.image = pygame.transform.rotate(self.original_image, 180)
        elif self.direction == 'E':
            self.image = pygame.transform.rotate(self.original_image, -90)
        elif self.direction == 'W':
            self.image = pygame.transform.rotate(self.original_image, 90)

        self.rect = self.image.get_rect(topleft=(x, y))
        self.x = x
        self.y = y

    def move(self, lights, vehicles):
        """Move the vehicle based on traffic light states and nearby vehicles."""
        # If in collision, do nothing.
        if self.collision_time > 0:
            return

        # Identify the traffic light for this vehicle’s lane.
        light = next((l for l in lights if l.direction == self.direction), None)

        # Check red signal behavior.
        if light and light.state == 'red':
            stop_line = get_stop_line(light)
            front = get_front_position(self)
            # Determine if there's a vehicle ahead in the same lane.
            car_ahead = False
            for other in vehicles:
                if other is self or other.direction != self.direction:
                    continue
                if self.direction == 'N' and other.y < self.y and abs(other.x - self.x) < CAR_WIDTH:
                    car_ahead = True
                    break
                elif self.direction == 'S' and other.y > self.y and abs(other.x - self.x) < CAR_WIDTH:
                    car_ahead = True
                    break
                elif self.direction == 'E' and other.x > self.x and abs(other.y - self.y) < CAR_HEIGHT:
                    car_ahead = True
                    break
                elif self.direction == 'W' and other.x < self.x and abs(other.y - self.y) < CAR_HEIGHT:
                    car_ahead = True
                    break

            # If no car ahead, use a minimal stop gap for the red signal.
            if not car_ahead:
                min_stop_gap = 10  # Minimal gap from the stop line when no vehicle is ahead.
                if self.direction == 'N':
                    # For northbound, vehicle comes from below (y decreases).
                    if self.y > stop_line and (self.y - stop_line) <= min_stop_gap:
                        return
                elif self.direction == 'S':
                    # For southbound, vehicle comes from above.
                    if (self.y + CAR_HEIGHT) < stop_line and (stop_line - (self.y + CAR_HEIGHT)) <= min_stop_gap:
                        return
                elif self.direction == 'E':
                    # For eastbound, vehicle comes from left.
                    if (self.x + CAR_WIDTH) < stop_line and (stop_line - (self.x + CAR_WIDTH)) <= min_stop_gap:
                        return
                elif self.direction == 'W':
                    # For westbound, vehicle comes from right.
                    if self.x > stop_line and (self.x - stop_line) <= min_stop_gap:
                        return
            else:
                # If there is a car ahead, rely on the safe_distance check below.
                pass

        # Check for vehicles ahead in the same lane using the usual safe distance.
        safe_distance = 50
        for other in vehicles:
            if other is self or other.direction != self.direction:
                continue
            if self.direction == 'N' and other.y < self.y:
                if abs(other.x - self.x) < CAR_WIDTH and (self.y - other.y) < safe_distance:
                    return
            elif self.direction == 'S' and other.y > self.y:
                if abs(other.x - self.x) < CAR_WIDTH and (other.y - self.y) < safe_distance:
                    return
            elif self.direction == 'W' and other.x < self.x:
                if abs(other.y - self.y) < CAR_HEIGHT and (self.x - other.x) < safe_distance:
                    return
            elif self.direction == 'E' and other.x > self.x:
                if abs(other.y - self.y) < CAR_HEIGHT and (other.x - self.x) < safe_distance:
                    return

        # Move the vehicle.
        if self.direction == 'N':
            self.y -= self.speed
        elif self.direction == 'S':
            self.y += self.speed
        elif self.direction == 'W':
            self.x -= self.speed
        elif self.direction == 'E':
            self.x += self.speed

        self.rect.topleft = (self.x, self.y)

    def draw(self):
        """Draw the vehicle on the screen."""
        screen.blit(self.image, (self.x, self.y))

# Updated Traffic Light positions based on the intersection.
# - North: light at the bottom of the intersection.
# - South: light at the top.
# - East: light on the left.
# - West: light on the right.
lights = [
    TrafficLight(INTERSECTION_X - ROAD_WIDTH // 4, INTERSECTION_Y + ROAD_WIDTH // 2 + 30, 'N'),
    TrafficLight(INTERSECTION_X + ROAD_WIDTH // 4, INTERSECTION_Y - ROAD_WIDTH // 2 - 30, 'S'),
    TrafficLight(INTERSECTION_X - ROAD_WIDTH // 2 - 30, INTERSECTION_Y + ROAD_WIDTH // 4, 'E'),
    TrafficLight(INTERSECTION_X + ROAD_WIDTH // 2 + 30, INTERSECTION_Y - ROAD_WIDTH // 4, 'W')
]

vehicles = []

def spawn_vehicle():
    """Spawn a new vehicle at the appropriate starting position based on its direction."""
    direction = random.choice(['N', 'S', 'E', 'W'])
    if direction == 'N':
        x = INTERSECTION_X - LANE_WIDTH // 2 - CAR_WIDTH // 2
        y = HEIGHT
    elif direction == 'S':
        x = INTERSECTION_X + LANE_WIDTH // 2 - CAR_WIDTH // 2
        y = -CAR_HEIGHT
    elif direction == 'W':
        x = WIDTH
        y = INTERSECTION_Y - LANE_WIDTH // 2 - CAR_HEIGHT // 2
    elif direction == 'E':
        x = -CAR_WIDTH
        y = INTERSECTION_Y + LANE_WIDTH // 2 - CAR_HEIGHT // 2
    vehicles.append(Vehicle(x, y, direction))

def draw_roads():
    """Draw the roads with two lanes for each direction."""
    # Vertical road lanes
    pygame.draw.rect(screen, GRAY, (INTERSECTION_X - ROAD_WIDTH // 2, 0, LANE_WIDTH, HEIGHT))
    pygame.draw.rect(screen, GRAY, (INTERSECTION_X + ROAD_WIDTH // 2 - LANE_WIDTH, 0, LANE_WIDTH, HEIGHT))
    # Horizontal road lanes
    pygame.draw.rect(screen, GRAY, (0, INTERSECTION_Y - ROAD_WIDTH // 2, WIDTH, LANE_WIDTH))
    pygame.draw.rect(screen, GRAY, (0, INTERSECTION_Y + ROAD_WIDTH // 2 - LANE_WIDTH, WIDTH, LANE_WIDTH))

clock = pygame.time.Clock()
spawn_timer = 0
cycle_time = 26  # Total cycle time for lights in seconds
start_time = time.time()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Spawn a new vehicle approximately every 2 seconds.
    spawn_timer += 1
    if spawn_timer >= 100:  # ~2 seconds at 60 FPS
        spawn_vehicle()
        spawn_timer = 0

    # Traffic light timing logic.
    current_time = time.time() - start_time
    cycle_position = current_time % cycle_time
    if cycle_position < 10:
        ns_state = 'green'
        ew_state = 'red'
    elif cycle_position < 13:
        ns_state = 'yellow'
        ew_state = 'red'
    elif cycle_position < 23:
        ns_state = 'red'
        ew_state = 'green'
    else:
        ns_state = 'red'
        ew_state = 'yellow'

    # Apply states to lights.
    for light in lights:
        if light.direction in ['N', 'S']:
            light.state = ns_state
        else:
            light.state = ew_state

    # Update vehicles.
    for vehicle in vehicles[:]:
        vehicle.move(lights, vehicles)

        # Check for collisions.
        for other in vehicles:
            if vehicle is not other and vehicle.rect.colliderect(other.rect):
                if vehicle.collision_time == 0 and other.collision_time == 0:
                    print("collision")
                    collision_sound.play()
                    vehicle.collision_time = time.time()
                    other.collision_time = time.time()

        # Remove vehicles that leave the screen or have exceeded collision duration.
        if (vehicle.x < -CAR_WIDTH or vehicle.x > WIDTH or
            vehicle.y < -CAR_HEIGHT or vehicle.y > HEIGHT or
            (vehicle.collision_time > 0 and time.time() - vehicle.collision_time >= vehicle.collision_duration)):
            if vehicle in vehicles:
                vehicles.remove(vehicle)

    # Drawing.
    screen.fill(BLACK)
    draw_roads()
    for light in lights:
        light.draw()
    for vehicle in vehicles:
        vehicle.draw()
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
