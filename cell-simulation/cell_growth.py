# Simulation of cell growth

import open3d as o3d
import numpy as np
import tkinter as tk
import multiprocessing
import random

class GrowthEnvironment:
    def __init__(self,environment_energy: float) -> None:
        self.environment_energy = environment_energy

    def consumeEnergy(self,amount: float):
        self.environment_energy -= amount
        # print("[ENVIRONMENT]: Consumed energy units = ",amount)

class CellTypes:
    SPHERICAL_CELL = "sphere"
    CUBOIDAL_CELL = "cuboid"


class Cell:
    def __init__(self,digital_dna: str,health: int,cell_radius: float,position: np.ndarray):
        self.digital_dna =  digital_dna
        self.health = health
        self.position = position
        self.cell_radius = cell_radius
        self.shape = CellTypes.SPHERICAL_CELL
        self.separation_factor = 0.6

    
    def multiply(self):
        self.health -= 1
        growth_factor_x = np.random.choice(np.array([0,1]), p=[0.5,0.5])
        growth_factor_y = np.random.choice(np.array([0,1]), p=[0.5,0.5])
        growth_factor_z = np.random.choice(np.array([0,1]), p=[0.5,0.5])
        new_cell_position = self.position + [self.separation_factor*growth_factor_x,self.separation_factor*growth_factor_y,self.separation_factor*growth_factor_z]
        # print("new cell at: ",new_cell_position)
        return Cell(digital_dna=self.digital_dna,health=10,cell_radius=self.cell_radius,position=new_cell_position)
            

class Organism:
    def __init__(self,growth_centre: np.ndarray,max_growth: int):
        # create initial cell group of the organism
        
        self.cells =  [Cell(digital_dna="ACTGAA",health=10,cell_radius=0.5,position=growth_centre)]
        self.current_growth = 0
        self.max_growth = max_growth
        self.visualizer =  o3d.visualization.Visualizer()
        self.rendering_thread = None

    def canGrow(self):
        return self.current_growth < self.max_growth
    
    # parse the digital_dna to determine next growth step for the cell group
    def grow(self,growth_environment: GrowthEnvironment):
        for i in range(self.max_growth):
            # decide which cell to multiply
            cellpointer = i
            if i > 1:
                cellpointer = random.randrange(0,len(self.cells) - 1)

            if self.canGrow():
                if self.current_growth == 0:
                    print("[ORGANISM]: Growth started")

                # consume energy
                growth_environment.consumeEnergy(len(self.cells))

                # try to multiply cells and then increment growth iteration
                newcell = self.cells[cellpointer].multiply()
                self.cells.append(newcell)

                self.current_growth += 1
        
        print("[ORGANISM]: Growth completed. Reached iteration = ",self.current_growth)
        print("Cell count after growth: ",len(self.cells))

    # Should be called only after growth is completed
    def startRender(self):
        self.rendering_thread = multiprocessing.Process(target=self.render, args=())
        self.rendering_thread.start()

    def render(self):
        if self.canGrow():
            raise Exception("Cannot render organism, growth not completed")
        
        self.visualizer.create_window(window_name='Cell Growth Render', width=1600, height=900)
        for cell in self.cells:
            cell_obj = o3d.geometry.TriangleMesh.create_sphere(radius=cell.cell_radius)
            cell_obj.translate(cell.position)
            cell_color = [0,0.5,0]
            if cell.health <= 5:
                cell_color = [0.5,0.5,0]
            elif cell.health == 0:
                cell_color = [0.5,0,0]
            cell_obj.paint_uniform_color(cell_color)
            self.visualizer.add_geometry(cell_obj)
    
        while True:
            self.visualizer.poll_events()
            self.visualizer.update_renderer()
            

    def stopRender(self):
        self.visualizer.destroy_window()
        if self.rendering_thread is not None:
            self.rendering_thread.terminate()

window = tk.Tk()
window.title("Cell growth simulation")

tk.Label(text="Environment Energy").pack()
env_energy_input = tk.Entry(width=50)
env_energy_input.insert(tk.END,"100")
env_energy_input.pack()

tk.Label(text="Max Growth").pack()
max_growth_input = tk.Entry(width=50)
max_growth_input.insert(tk.END,"500")
max_growth_input.pack()

start_btn = tk.Button(
    text="Start",
    background="green",
    width=10,
    height=2,
)
start_btn.pack()

stop_btn = tk.Button(
    text="Stop",
    width=10,
    background="red",
    height=2,
)
stop_btn.pack()

# ----------------handler functions--------------------
organism: Organism = None

def handle_start(event):
    try:
        global organism
        if organism is None:
            environment = GrowthEnvironment(environment_energy=int(env_energy_input.get()))
            organism = Organism(growth_centre=np.array([0,0,0]),max_growth=int(max_growth_input.get()))
            organism.grow(growth_environment=environment)
            organism.startRender()
        else:
            raise Exception("Organism already exists.Stop the session and start a new growth simulation")
    except Exception as e:
        print(e)
        
def handle_stop(event):
    global organism
    global window
    if organism is not None: 
        organism.stopRender()
        organism = None
# -----------------------------------------------------


start_btn.bind("<Button-1>",handle_start)
stop_btn.bind("<Button-1>",handle_stop)

window.mainloop()







