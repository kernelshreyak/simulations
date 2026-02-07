import datetime
import os
import random


class Plant:
    def __init__(self, name):
        self.name = name
        self.health = 80
        self.last_cared = datetime.date.today()
        self.log = []

    def days_without_care(self):
        today = datetime.date.today()
        return (today - self.last_cared).days

    def update_health(self):
        days = self.days_without_care()
        if days > 0:
            self.health -= days * 4
        self.health = max(self.health, 20)  # never drops below 20

    def add_activity(self, activity, impact):
        if impact < 0:
            self.health += impact
        else:
            self.health += impact
        self.health = max(10, min(self.health, 100))
        self.last_cared = datetime.date.today()
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
        self.log.append((timestamp, activity, impact))
        self.log = self.log[-12:]

    def status(self):
        self.update_health()
        if self.health > 75:
            mood = "thriving"
        elif self.health > 55:
            mood = "steady"
        elif self.health > 35:
            mood = "struggling"
        else:
            mood = "fragile"
        return f"{self.name} is {mood}. Health: {self.health}/100"

    def stage_index(self):
        if self.health > 85:
            return 3
        if self.health > 65:
            return 2
        if self.health > 45:
            return 1
        return 0

    def issues(self):
        self.update_health()
        issues = []
        days = self.days_without_care()
        if self.health <= 35:
            issues += ["drooping leaves", "dry soil", "weak stem"]
        elif self.health <= 55:
            issues += ["faded color", "slow growth", "thin leaves"]
        elif self.health <= 75:
            issues += ["slight tilt", "dusty leaves"]
        if days >= 2:
            issues.append("thirsty roots")
        if not issues:
            issues = ["none"]
        return issues

    def render(self):
        stages = [
            [
                "      .",
                "     .'.",
                "    /___\\",
                "      |",
                "     _|_",
                "   _/___\\_",
            ],
            [
                "      .",
                "     \\|/",
                "    --*--",
                "      |",
                "     / \\",
                "   _/___\\_",
            ],
            [
                "     \\|/",
                "    --*--",
                "     \\|/",
                "      |",
                "     / \\",
                "   _/___\\_",
            ],
            [
                "    \\ | /",
                "   -- * --",
                "    / | \\",
                "      |",
                "     / \\",
                "   _/___\\_",
            ],
        ]
        bar = "#" * (self.health // 10)
        bar = bar.ljust(10, "-")
        art = "\n".join(stages[self.stage_index()])
        return f"{art}\nHealth [{bar}] {self.health}/100"


def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def header(title):
    line = "=" * 50
    return f"{line}\n{title}\n{line}"


def prompt_choice():
    print("\nChoose:")
    print("  1) Log a deed")
    print("  2) Tend the plant (gentle care)")
    print("  3) View recent log")
    print("  4) Help")
    print("  5) Exit")
    return input("Select 1-5: ").strip()


def sentiment_score(text):
    good_words = [
        "help", "care", "kind", "listen", "exercise", "clean", "cook", "water",
        "study", "read", "walk", "rest", "apologize", "thank", "share", "donate",
        "hug", "plan", "organize", "save"
    ]
    bad_words = [
        "yell", "argue", "ignore", "lie", "procrastinate", "waste", "scroll",
        "drink", "skip", "hurt", "gossip", "oversleep", "forget", "miss", "rage"
    ]
    t = text.lower()
    good = sum(1 for w in good_words if w in t)
    bad = sum(1 for w in bad_words if w in t)
    if good > bad:
        return 1
    if bad > good:
        return -1
    return 0


def intensity_prompt():
    while True:
        amt = input("How strong did it feel? (1-3): ").strip()
        if amt in {"1", "2", "3"}:
            return int(amt)
        print("Please choose 1, 2, or 3.")


def log_deed(plant):
    clear_screen()
    print(header("A Small Moment"))
    activity = input("Describe one thing you did today: ").strip()
    if not activity:
        print("Nothing logged.")
        input("Press Enter to continue...")
        return

    sentiment = sentiment_score(activity)
    print("\nHow did it affect your day?")
    print("  1) It lifted me")
    print("  2) It drained me")
    print("  3) It was neutral")
    choice = input("Choose 1-3: ").strip()

    if choice == "1":
        direction = 1
    elif choice == "2":
        direction = -1
    else:
        direction = 0

    if direction == 0:
        impact = 0
    else:
        intensity = intensity_prompt()
        base = 6 if direction > 0 else -7
        impact = base * intensity

    if sentiment != 0 and direction == 0:
        # If their words carry weight, reflect a mild effect.
        impact = 4 if sentiment > 0 else -4

    plant.add_activity(activity, impact)
    outcome = "strengthened" if impact > 0 else "strained" if impact < 0 else "left unchanged"
    print(f"\nThe deed {outcome} the plant.")
    input("Press Enter to continue...")


def tend_plant(plant):
    clear_screen()
    print(header("Quiet Care"))
    prompts = [
        "You wipe dust from the leaves.",
        "You loosen the soil gently.",
        "You open the curtain for soft light.",
        "You check the water level.",
    ]
    print(random.choice(prompts))
    plant.add_activity("gentle care", 5)
    print("The plant responds with a subtle lift.")
    input("Press Enter to continue...")


def view_log(plant):
    clear_screen()
    print(header("Recent Moments"))
    if not plant.log:
        print("No entries yet.")
    else:
        for timestamp, activity, impact in plant.log[-6:]:
            mark = "+" if impact > 0 else "-" if impact < 0 else "0"
            print(f"[{timestamp}] {mark} {activity}")
    input("\nPress Enter to continue...")


def show_help():
    clear_screen()
    print(header("How This Works"))
    print("- Log one deed at a time; good deeds lift the plant.")
    print("- Heavy or harmful moments create wider issues.")
    print("- Gentle care is always a small boost.")
    print("- The plant reflects your day, not perfection.")
    input("\nPress Enter to continue...")


def main():
    my_plant = Plant("Fern")

    while True:
        clear_screen()
        print(header("My Plant"))
        print(my_plant.render())
        print("\n" + my_plant.status())
        print("Issues: " + ", ".join(my_plant.issues()))
        print(f"Days without care: {my_plant.days_without_care()}")

        choice = prompt_choice()
        if choice == "1":
            log_deed(my_plant)
        elif choice == "2":
            tend_plant(my_plant)
        elif choice == "3":
            view_log(my_plant)
        elif choice == "4":
            show_help()
        elif choice == "5":
            clear_screen()
            print("You leave the garden quietly.")
            break
        else:
            print("Please choose 1-5.")
            input("Press Enter to continue...")


if __name__ == "__main__":
    main()
