import numpy as np
import matplotlib.pyplot as plt

# Paramètre de Van der Pol
mu = 0.75

# Création du meshgrid
x = np.linspace(-3, 3, 20)
v = np.linspace(-3, 3, 20)
X, V = np.meshgrid(x, v)

# Définition du champ vectoriel de Van der Pol
dX = V
dV = mu * (1 - X**2) * V - X
#R = np.sqrt(dX**2+dV**2)
R=1
# Tracer le champ de vecteurs
plt.figure(figsize=(8, 8))
plt.quiver(X, V, dX/R, dV/R, color='r',scale_units='xy')

plt.title("Champ de vecteurs de l'oscillateur de Van der Pol")
plt.xlabel('Position (x)')
plt.ylabel('Vitesse (v)')
plt.grid(True)
plt.xlim([-3, 3])
plt.ylim([-3, 3])
plt.show()

