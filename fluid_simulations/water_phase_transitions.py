import matplotlib.pyplot as plt
import numpy as np
from CoolProp.CoolProp import PropSI

pressures_bar = [1, 100, 250]
temperatures_c = np.linspace(25, 500, 600)

plt.figure(figsize=(9, 6))

for P_bar in pressures_bar:
    P = P_bar * 1e5
    densities = []

    for T_c in temperatures_c:
        T = T_c + 273.15

        try:
            rho = PropsSI("D", "T", T, "P", P, "Water")
        except Exception:
            rho = np.nan

        densities.append(rho)

    plt.plot(temperatures_c, densities, label=f"{P_bar} bar")

plt.axvline(374, linestyle="--", label="Critical temperature ~374°C")
plt.axhline(322, linestyle="--", label="Critical density ~322 kg/m³")

plt.xlabel("Temperature [°C]")
plt.ylabel("Density [kg/m³]")
plt.title("Heating water at different pressures")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
