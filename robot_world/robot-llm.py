from pydantic import BaseModel
from typing import List, Literal, Optional, Union
import openai
import pygame
import sys
import threading
import json
import time
import random
import pandas as pd  # for inventory display
import math

# ==== Constants ====
SPRITE_SIZE = 50  # uniform size for all sprites
NUM_APPLES = 5    # initial number of apples to spawn

# ==== Action Schemas ====
class MoveParams(BaseModel):
    direction: Literal["up", "down", "left", "right"]
    steps: int

class ObjectParams(BaseModel):
    object_name: str

class Action(BaseModel):
    type: Literal["move", "use", "drop", "consume", "inventory"]
    params: Optional[Union[MoveParams, ObjectParams]]

class ActionPlan(BaseModel):
    actions: List[Action]

# ==== World Object Definition ====
class WorldObject:
    def __init__(self, name: str, x: int, y: int, sprite_path: str,
                 kind: Literal["consumable", "resource"], max_uses: int = 1):
        self.name = name
        self.x = x
        self.y = y
        self.kind = kind
        self.max_uses = max_uses
        self.uses_left = max_uses if kind == "consumable" else None
        self.max_value = max_uses if kind == "resource" else None
        self.value = max_uses if kind == "resource" else None
        original = pygame.image.load(sprite_path).convert_alpha()
        self.sprite = pygame.transform.scale(original, (SPRITE_SIZE, SPRITE_SIZE))
        self.rect = self.sprite.get_rect(center=(x, y))
        self.next_refill = None

    def draw(self, surface):
        surface.blit(self.sprite, self.rect)

    def interact(self):
        if self.kind == "consumable":
            return "pickup"
        else:
            if self.value and self.value > 0:
                self.value -= 1
                if self.next_refill is None:
                    delay = random.randint(10, 50)
                    self.next_refill = time.time() + delay
                    print(f"[INFO] {self.name} will refill in {delay} seconds.")
                return "resource_used"
            else:
                print(f"[ERROR] {self.name} is empty.")
                return None

    def update(self):
        if self.kind == "resource" and self.next_refill:
            if time.time() >= self.next_refill:
                self.value = self.max_value
                self.next_refill = None
                print(f"[INFO] {self.name} has been refilled.")

# ==== Agent Definition with Inventory & Memory ====
class Agent:
    def __init__(self, sprite_path: str):
        self.start_x = 400
        self.start_y = 300
        self.x = self.start_x
        self.y = self.start_y
        self.history = []
        self.inventory = {}  # item_name -> count
        original = pygame.image.load(sprite_path).convert_alpha()
        self.sprite = pygame.transform.scale(original, (SPRITE_SIZE, SPRITE_SIZE))
        self.record_state()

    def record_state(self):
        self.history.append({"x": self.x, "y": self.y})

    def move(self, direction: str, steps: int):
        dist = steps * 5
        if direction == "up":   self.y -= dist
        elif direction == "down": self.y += dist
        elif direction == "left": self.x -= dist
        elif direction == "right":self.x += dist
        self.x = max(0, min(800, self.x))
        self.y = max(0, min(600, self.y))
        self.record_state()

    def pickup(self, obj: WorldObject, world_objects: List[WorldObject]):
        self.inventory[obj.name] = self.inventory.get(obj.name, 0) + 1
        print(f"[INFO] Picked up {obj.name}.")
        world_objects.remove(obj)

    def use(self, obj: WorldObject, world_objects: List[WorldObject]):
        if obj.kind == "resource":
            if self.inventory.get("bucket", 0) < 1:
                print("[ERROR] Need bucket in inventory to use resource.")
                return
        # proximity check
        if not self.get_rect().colliderect(obj.rect):
            print(f"[ERROR] Agent not in range to use {obj.name}.")
            return
        result = obj.interact()
        if result == "pickup":
            self.pickup(obj, world_objects)
        elif result == "resource_used":
            print(f"[INFO] Used resource {obj.name}.")

    def drop(self, item_name: str, world_objects: List[WorldObject], world_defs):
        if self.inventory.get(item_name, 0) < 1:
            print(f"[ERROR] No {item_name} in inventory.")
            return
        self.inventory[item_name] -= 1
        if self.inventory[item_name] == 0:
            del self.inventory[item_name]
        defs = world_defs[item_name]
        world_objects.append(WorldObject(item_name, self.x, self.y, defs["sprite"], defs["kind"], defs.get("max_uses",1)))
        print(f"[INFO] Dropped {item_name}.")

    def consume(self, item_name: str):
        if self.inventory.get(item_name, 0) < 1:
            print(f"[ERROR] No {item_name} to consume.")
            return
        self.inventory[item_name] -= 1
        if self.inventory[item_name] == 0:
            del self.inventory[item_name]
        print(f"[INFO] Consumed {item_name}.")

    def view_inventory(self):
        data = []
        for item, count in self.inventory.items():
            kind = world_defs.get(item, {}).get("kind", "unknown")
            data.append({"item": item, "type": kind, "count": count})
        df = pd.DataFrame(data)
        print("\nInventory:")
        print(df.to_string(index=False))

    def get_rect(self):
        return self.sprite.get_rect(center=(self.x, self.y))

    def draw(self, surface):
        rect = self.sprite.get_rect(center=(self.x, self.y))
        surface.blit(self.sprite, rect)

    def to_dict(self):
        return {
            "position": {"x": self.x, "y": self.y},
            "inventory": self.inventory
        }

# ==== World Definitions for drop ====
world_defs = {
    "bucket": {"sprite": "bucket.png", "kind": "consumable", "max_uses": 1},
    "apple":  {"sprite": "apple.png",  "kind": "consumable", "max_uses": 1},
    "water_tank": {"sprite": "water_tank.png","kind": "resource","max_uses": 5}
}

# ==== Initialize Agent & World ====
pygame.init()
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Agent World")
WHITE = (255, 255, 255)
clock = pygame.time.Clock()

agent = Agent(sprite_path="agent_sprite.png")
world_objects: List[WorldObject] = []
world_objects.append(WorldObject("bucket", 200, 150, "bucket.png", kind="consumable", max_uses=1))
world_objects.append(WorldObject("water_tank", 600, 450, "water_tank.png", kind="resource", max_uses=5))
for _ in range(NUM_APPLES):
    x, y = random.randint(50, 750), random.randint(50, 550)
    world_objects.append(WorldObject("apple", x, y, "apple.png", kind="consumable", max_uses=1))

# ==== OpenAI Query ====
def get_action_plan(user_input: str) -> ActionPlan:
    client = openai.OpenAI()
    context = json.dumps(agent.to_dict(), indent=2)
    objects_info = json.dumps(
        [{"name": o.name, "kind": o.kind, "uses_left": o.uses_left or o.value} for o in world_objects],
        indent=2
    )
    response = client.responses.parse(
        model="gpt-4.1-nano",
        input=[
            {"role": "system", "content": (
                f"Agent state:\n{context}\nWorld objects:\n{objects_info}\n"
                "Respond with actions: 'move' (direction, steps), 'use' (object_name), 'drop' (object_name), 'consume' (object_name), or 'inventory'."
                "If command requires more than one action, return list of actions"
            )},
            {"role": "user", "content": user_input}
        ],
        text_format=ActionPlan
    )
    return response.output_parsed

# ==== Action Dispatch ====
movement_queue: List[Action] = []

def process_action(action: Action):
    t = action.type
    if t == "move":
        params = action.params  # type: ignore
        agent.move(params.direction, params.steps)
    elif t == "use":
        params = action.params  # type: ignore
        name = params.object_name
        kind = world_defs.get(name, {}).get("kind")
        # if resource and no bucket, queue bucket then tank
        if kind == "resource" and agent.inventory.get("bucket", 0) < 1:
            # enqueue tank use
            movement_queue.insert(0, Action(type="use", params=ObjectParams(object_name=name)))
            # enqueue bucket use
            movement_queue.insert(0, Action(type="use", params=ObjectParams(object_name="bucket")))
            return
        # find nearest object of that name
        objs = [o for o in world_objects if o.name == name]
        if not objs:
            print(f"[ERROR] No {name} available in world.")
            return
        nearest = min(objs, key=lambda o: math.hypot(o.x-agent.x, o.y-agent.y))
        # if not in range, queue move then use
        if not agent.get_rect().colliderect(nearest.rect):
            dx, dy = nearest.x-agent.x, nearest.y-agent.y
            if abs(dx) >= SPRITE_SIZE:
                dir_x = "right" if dx>0 else "left"
                steps_x = int(abs(dx)//SPRITE_SIZE)
                movement_queue.insert(0, Action(type="use", params=ObjectParams(object_name=name)))
                movement_queue.insert(0, Action(type="move", params=MoveParams(direction=dir_x, steps=steps_x)))
                return
            if abs(dy) >= SPRITE_SIZE:
                dir_y = "down" if dy>0 else "up"
                steps_y = int(abs(dy)//SPRITE_SIZE)
                movement_queue.insert(0, Action(type="use", params=ObjectParams(object_name=name)))
                movement_queue.insert(0, Action(type="move", params=MoveParams(direction=dir_y, steps=steps_y)))
                return
        # in range or moved, perform use
        agent.use(nearest, world_objects)
    elif t == "drop":
        params = action.params  # type: ignore
        agent.drop(params.object_name, world_objects, world_defs)
    elif t == "consume":
        params = action.params  # type: ignore
        agent.consume(params.object_name)
    elif t == "inventory":
        agent.view_inventory()
    else:
        if action.params is None:
            return
        print(f"[WARN] Unknown action: {t}")

# ==== CLI Input Thread ====
def input_thread():
    while True:
        cmd = input("Command> ")
        if not cmd.strip():
            continue
        try:
            plan = get_action_plan(cmd)
            print("PLAN:",plan)
            movement_queue.extend(plan.actions)
        except Exception as e:
            print(f"[ERROR] {e}")

threading.Thread(target=input_thread, daemon=True).start()

# ==== Main Loop ====
running = True
while running:
    screen.fill(WHITE)
    for obj in list(world_objects):
        obj.update()
        obj.draw(screen)
    agent.draw(screen)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if movement_queue:
        act = movement_queue.pop(0)
        process_action(act)
    pygame.display.flip()
    clock.tick(30)

pygame.quit()
sys.exit()
