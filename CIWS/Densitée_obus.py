import math
import matplotlib.pyplot as plt

# Données initiales
cadence_tir = 4500  # Cadence de tir en obus par minute
cadence_tir_par_seconde = cadence_tir / 60  # Cadence de tir en obus par seconde
vitesse_missile = 0.27  # Vitesse du missile en km/s (Mach 0.8)
distance_initiale = 3000  # Distance initiale en mètres (3 km)
distance_finale = 300  # Distance finale en mètres (100 m)
angle_dispersion_deg = 1  # Angle de dispersion de chaque côté en degrés (dispersion totale de 2°)

# Calcul du rayon de dispersion à différentes distances
def rayon_dispersion(distance, angle_deg):
    angle_rad = math.radians(angle_deg)
    return math.tan(angle_rad) * distance

# Surface de dispersion (en m²)
def surface_dispersion(rayon):
    return math.pi * rayon ** 2

# Calcul du nombre d'obus tirés pendant un temps donné (en secondes)
def nombre_obus_tires(cadence_tir_par_seconde, temps):
    return cadence_tir_par_seconde * temps

# Paramètres pour la simulation
temps_total = (distance_initiale - distance_finale) / vitesse_missile  # Temps pour parcourir la distance
pas_temps = 0.1  # Pas de temps en secondes
obstacle_hit_count = 0  # Compteur pour les obus touchant la cible

# Variables de la simulation
temps = 0  # Temps initial (en secondes)
distance_missile = distance_initiale  # Distance initiale du missile
rayon_cible_initial = rayon_dispersion(distance_initiale, angle_dispersion_deg)  # Rayon de dispersion initial

# Listes pour stocker les valeurs du temps et du nombre d'obus touchant la cible
temps_list = []
obstacles_touch_list = []

# Listes pour stocker la distance et le nombre d'impacts associés
distance_list = []
impacts_distance_list = []

# Simulation par itération
while distance_missile > distance_finale:
    # Calcul du nombre d'obus tirés à ce moment-là
    nombre_obus = nombre_obus_tires(cadence_tir_par_seconde, temps)

    # Calcul du rayon de dispersion à la distance actuelle
    rayon_cible = rayon_dispersion(distance_missile, angle_dispersion_deg)

    # Calcul de la surface de dispersion à cette distance
    surface_cible = surface_dispersion(rayon_cible)

    # Supposons que la densité des obus qui frappent la cible est proportionnelle à la surface de dispersion
    densite_billes = (cadence_tir_par_seconde * pas_temps) / surface_cible  # Nombre d'obus par m²

    # Calcul du nombre d'obus touchant la cible
    nombre_billes_toucher = densite_billes * 6 * 0.4  # Surface de la cible (6m x 0.4m)

    # Ajouter les impacts à notre compteur total
    obstacle_hit_count += nombre_billes_toucher

    # Ajouter les valeurs dans les listes pour le graphique
    temps_list.append(temps)
    obstacles_touch_list.append(obstacle_hit_count)

    # Ajouter les valeurs dans les listes pour la distance
    distance_list.append(distance_missile)
    impacts_distance_list.append(obstacle_hit_count)

    # Mettre à jour la distance du missile
    distance_missile -= vitesse_missile * pas_temps

    # Incrémenter le temps
    temps += pas_temps

# Afficher le résultat
print(f"Le nombre total d'obus touchant la cible avant l'impact est : {obstacle_hit_count:.2f}")
if obstacle_hit_count >= 15:
    print("Le missile sera neutralisé.")
else:
    print("Le missile ne sera pas neutralisé.")

# Affichage du graphique avec deux courbes
plt.figure(figsize=(10, 6))

# Premier graphique : Nombre d'obus touchant la cible en fonction du temps
plt.subplot(2, 1, 1)
plt.plot(temps_list, obstacles_touch_list, label="Nombre d'obus touchant la cible", color='blue')
plt.xlabel('Temps (s)')
plt.ylabel('Nombre d\'obus touchant la cible')
plt.title('Nombre d\'obus touchant la cible en fonction du temps')
plt.grid(True)
plt.legend()

# Deuxième graphique : Nombre d'obus touchant la cible en fonction de la distance
plt.subplot(2, 1, 2)
plt.plot(distance_list, impacts_distance_list, label="Nombre d\'obus touchant la cible", color='red')
plt.xlabel('Distance (m)')
plt.ylabel('Nombre d\'obus touchant la cible')
plt.title('Nombre d\'obus touchant la cible en fonction de la distance')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()