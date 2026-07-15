import argparse
import random
import sqlite3
from pathlib import Path

try:
    from tqdm import tqdm
except ImportError:  # pragma: no cover - optional dependency
    tqdm = None

DB_PATH = Path(__file__).with_name("family_tree.db")

class Human:
    def __init__(self, uid, name, age, gender):
        self.uid = uid  # Stable unique identifier used across in-memory and SQLite records.
        self.name = name  # Display name for the person.
        self.age = age  # Current age in simulation years.
        self.gender = gender  # Biological sex marker used by reproduction logic.
        self.is_alive = True  # Whether the person is still alive in the simulation.
        self.parents = []  # Ordered as [mother, father] when known.
        self.children = []  # Direct descendants produced by this person.
        self.generation = 0  # Founder generation starts at 0 and increments per birth.
        self.inbreeding_coefficient = 0.0  # Probability that paired genes are identical by descent.
        self.parent_kinship = 0.0  # Kinship coefficient between this person's parents.


class FamilyTree:
    def __init__(self, start_year=1995):
        self.start_year = start_year  # Baseline year from which births and aging are calculated.
        self.current_year = start_year  # Mutable simulation clock.
        self.humans = []  # Flat registry of every person created in the simulation.
        self._humans_by_id = {}  # Fast lookup table from UID to Human instance.
        self._next_person_id = 1  # Sequence counter for generating unique person IDs.
        self._next_child_index = 1  # Sequence counter for default child names.
        self._kinship_cache = {}  # Memoized pairwise kinship coefficients keyed by sorted UID pairs.

        self.mother = self._create_founder("Alice", 20, "F")  # Initial female founder.
        self.father = self._create_founder("Bob", 22, "M")  # Initial male founder.

    def _new_uid(self):
        uid = f"p{self._next_person_id:06d}"
        self._next_person_id += 1
        return uid

    def _create_founder(self, name, age, gender):
        founder = Human(self._new_uid(), name, age, gender)
        self.humans.append(founder)
        self._humans_by_id[founder.uid] = founder
        return founder

    def _create_child(self, mother, father):
        child = Human(
            self._new_uid(),
            f"Child_{self._next_child_index}",
            0,
            random.choice(["M", "F"]),
        )
        self._next_child_index += 1
        child.parents = [mother, father]
        child.generation = max(mother.generation, father.generation) + 1
        child.parent_kinship = self.kinship_coefficient(mother, father)
        child.inbreeding_coefficient = child.parent_kinship
        mother.children.append(child)
        father.children.append(child)
        self.humans.append(child)
        self._humans_by_id[child.uid] = child
        return child

    def kinship_coefficient(self, left, right):
        key = tuple(sorted((left.uid, right.uid)))
        cached = self._kinship_cache.get(key)
        if cached is not None:
            return cached

        if left.uid == right.uid:
            value = 0.5 * (1.0 + left.inbreeding_coefficient)
        elif left.parents:
            mother, father = left.parents
            value = 0.5 * (
                self.kinship_coefficient(mother, right)
                + self.kinship_coefficient(father, right)
            )
        elif right.parents:
            mother, father = right.parents
            value = 0.5 * (
                self.kinship_coefficient(left, mother)
                + self.kinship_coefficient(left, father)
            )
        else:
            value = 0.0

        self._kinship_cache[key] = value
        return value

    def advance_time(self, years):
        self.current_year += years
        for human in self.humans:
            if human.is_alive:
                human.age += years
                if random.random() < 0.001:
                    human.is_alive = False

    def reproduce(self):
        adults = [human for human in self.humans if human.is_alive and human.age >= 18]
        mothers = [human for human in adults if human.gender == "F"]
        fathers = [human for human in adults if human.gender == "M"]
        if not mothers or not fathers:
            return

        for mother in mothers:
            if random.random() >= 0.9:
                continue
            father = random.choice(fathers)
            births = random.randint(1, 3)
            for _ in range(births):
                self._create_child(mother, father)

    def simulate(self, years=70):
        iterator = (
            range(years)
        )
        for year_index in iterator:
            self.advance_time(1)
            self.reproduce()
            if not tqdm and (year_index % 5 == 0 or year_index == years - 1):
                print(f"Simulated {year_index + 1}/{years} years")

    def save_database(self, db_path=DB_PATH):
        print("Writing SQLite graph store")
        db_file = Path(db_path)
        with sqlite3.connect(db_file) as conn:
            conn.execute("PRAGMA journal_mode=WAL")
            conn.execute("PRAGMA synchronous=NORMAL")
            conn.execute("PRAGMA temp_store=MEMORY")
            conn.executescript(
                """
                DROP VIEW IF EXISTS child_records;
                DROP TABLE IF EXISTS metadata;
                DROP TABLE IF EXISTS people;
                DROP TABLE IF EXISTS parent_child;

                CREATE TABLE metadata (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL
                );

                CREATE TABLE people (
                    id TEXT PRIMARY KEY,
                    name TEXT NOT NULL,
                    gender TEXT NOT NULL,
                    age INTEGER NOT NULL,
                    birth_year INTEGER NOT NULL,
                    is_alive INTEGER NOT NULL,
                    generation INTEGER NOT NULL,
                    parent_kinship REAL NOT NULL,
                    inbreeding_coefficient REAL NOT NULL
                );

                CREATE TABLE parent_child (
                    parent_id TEXT NOT NULL,
                    child_id TEXT NOT NULL,
                    parent_role TEXT NOT NULL CHECK(parent_role IN ('mother', 'father')),
                    PRIMARY KEY (parent_id, child_id, parent_role),
                    FOREIGN KEY (parent_id) REFERENCES people(id),
                    FOREIGN KEY (child_id) REFERENCES people(id)
                );

                CREATE INDEX idx_people_name ON people(name);
                CREATE INDEX idx_people_generation ON people(generation);
                CREATE INDEX idx_parent_child_parent ON parent_child(parent_id);
                CREATE INDEX idx_parent_child_child ON parent_child(child_id);

                CREATE VIEW child_records AS
                SELECT
                    child.id AS child_id,
                    child.name AS child_name,
                    child.gender AS child_gender,
                    child.age AS child_age,
                    child.birth_year AS child_birth_year,
                    child.generation AS child_generation,
                    child.parent_kinship AS parent_kinship,
                    child.inbreeding_coefficient AS inbreeding_coefficient,
                    mother.id AS mother_id,
                    mother.name AS mother_name,
                    father.id AS father_id,
                    father.name AS father_name
                FROM people child
                JOIN parent_child pcm
                    ON pcm.child_id = child.id AND pcm.parent_role = 'mother'
                JOIN people mother
                    ON mother.id = pcm.parent_id
                JOIN parent_child pcf
                    ON pcf.child_id = child.id AND pcf.parent_role = 'father'
                JOIN people father
                    ON father.id = pcf.parent_id;
                """
            )

            conn.executemany(
                """
                INSERT INTO people (
                    id, name, gender, age, birth_year, is_alive, generation,
                    parent_kinship, inbreeding_coefficient
                )
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                [
                    (
                        human.uid,
                        human.name,
                        human.gender,
                        human.age,
                        self.current_year - human.age,
                        int(human.is_alive),
                        human.generation,
                        human.parent_kinship,
                        human.inbreeding_coefficient,
                    )
                    for human in self.humans
                ],
            )

            edges = []
            for human in self.humans:
                if len(human.parents) == 2:
                    edges.append((human.parents[0].uid, human.uid, "mother"))
                    edges.append((human.parents[1].uid, human.uid, "father"))

            conn.executemany(
                "INSERT INTO parent_child (parent_id, child_id, parent_role) VALUES (?, ?, ?)",
                edges,
            )

            metadata = [
                ("start_year", str(self.start_year)),
                ("current_year", str(self.current_year)),
                ("total_people", str(len(self.humans))),
                (
                    "living_people",
                    str(sum(1 for human in self.humans if human.is_alive)),
                ),
            ]
            conn.executemany(
                "INSERT INTO metadata (key, value) VALUES (?, ?)", metadata
            )

        print(f"Saved {db_file}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Family tree simulator with SQLite output."
    )
    parser.add_argument("--years", type=int, default=70)
    return parser.parse_args()


def main():
    args = parse_args()
    tree = FamilyTree()
    tree.simulate(args.years)
    tree.save_database()


if __name__ == "__main__":
    main()
