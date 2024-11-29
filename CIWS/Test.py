import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Données initiales
cadence_tir = 4500  # Cadence de tir en obus par minute
cadence_tir_par_seconde = cadence_tir / 60  # Cadence de tir en obus par seconde
vitesse_missile = 0.27  # Vitesse du missile en km/s (Mach 0.8)
vitesse_obus = 1000  # Vitesse des obus en m/s
distance_initiale = 3000  # Distance initiale en mètres (3 km)
distance_finale = 0  # Distance finale en mètres (0 m à l'impact)
angle_dispersion_deg = 3  # Angle de dispersion de chaque côté en degrés (dispersion totale de 6°)
g_max = 25  # Maximum lateral maneuverability in terms of "g"
temps_pop_up_avant_impact = 5  # Temps avant l'impact pour déclencher le pop-up (en secondes)
altitude_max = 500  # Altitude maximale atteinte lors du pop-up (500 m)
altitude_missile = 3  # Altitude du missile avant le pop-up (3 m)
altitude_impact = 10  # Altitude d'impact (10 m)

# Conversion des unités
vitesse_missile_m_s = vitesse_missile * 1000  # Missile speed in m/s
g = 9.81  # Gravitational acceleration in m/s^2

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

# Calculate the amplitude of the zigzag based on g_max
def calculer_amplitude(g_max, vitesse_missile_m_s, period_distance=2000):
    T = period_distance / vitesse_missile_m_s  # Time to cover 2000 meters
    return (g_max * g * T ** 2) / (4 * math.pi ** 2)


# Zigzag movement: full cycle every 2000 meters
period_distance = 2000  # Distance for a full zigzag cycle (in meters)
amplitude = calculer_amplitude(g_max, vitesse_missile_m_s)


# Calcul de la position du missile à chaque instant avec une trajectoire parabolique corrigée
def position_missile(temps, amplitude, frequence_zigzag, temps_pop_up_avant_impact, temps_total, mode_vol):
    # Si le mode est "Vol direct", on ne fait aucune manœuvre
    if mode_vol == "Vol direct":
        y_missile = 0  # Pas de variation en Y
        z_missile = altitude_missile  # Altitude constante avant Pop-up
    elif mode_vol == "Vol manœuvrant":
        # Si le mode est "Vol manœuvrant", on fait un zigzag
        y_missile = amplitude * math.sin(2 * math.pi * frequence_zigzag * temps)
        z_missile = altitude_missile  # Altitude constante
    elif mode_vol == "Vol avec Pop-up":
        # Si le mode est "Vol avec Pop-up", il n'y a pas de zigzag, mais Pop-up
        y_missile = 0  # Pas de variation en Y
        # Mouvement en Z : trajectoire parabolique pour le pop-up
        temps_restant = temps_total - temps
        if temps_restant <= temps_pop_up_avant_impact:
            # On calcule la position sur la parabole entre altitude_missile et altitude_impact, avec l'apogée à altitude_max
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) * (
                        temps_restant - temps_milieu) ** 2
                    + altitude_max
            )
        else:
            # Avant le pop-up, altitude constante
            z_missile = altitude_missile
    elif mode_vol == "Vol avec Zigzag et Pop-up":
        # Si le mode est "Vol avec Zigzag et Pop-up", on fait un zigzag avant Pop-up
        temps_restant = temps_total - temps
        if temps_restant > temps_pop_up_avant_impact:
            # Mouvement Zigzag avant Pop-up
            amplitude_zigzag = 100  # Amplitude du zigzag en mètres
            frequence_zigzag = 0.5  # Fréquence du zigzag en Hz

            y_missile = amplitude_zigzag * math.sin(2 * math.pi * frequence_zigzag * temps)
            z_missile = altitude_missile  # Altitude constante pendant la manœuvre Zigzag
        else:
            # Pas de Zigzag pendant Pop-up, y est constant
            y_missile = 0
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) * (
                        temps_restant - temps_milieu) ** 2
                    + altitude_max
            )
    return y_missile, z_missile

# Paramètres pour la simulation
temps_total = (distance_initiale - distance_finale) / vitesse_missile_m_s  # Temps pour parcourir la distance
pas_temps = 0.1  # Pas de temps en secondes
obstacle_hit_count = 0  # Compteur pour les obus touchant la cible

# Variables de la simulation
temps = 0  # Temps initial (en secondes)
distance_missile = distance_initiale  # Distance initiale du missile

# Listes pour stocker les valeurs du temps, des distances et du nombre d'obus touchant la cible
temps_list = []
obstacles_touch_list = []
distance_list = []
impacts_distance_list = []
x_pos_list = []  # Position x du missile
y_pos_list = []  # Position y du missile
z_pos_list = []  # Position z du missile

# Mode de vol (changer selon le mode voulu)
mode_vol = "Vol avec Zigzag et Pop-up"  # Exemple de mode : "Vol direct", "Vol manœuvrant", "Vol avec Pop-up", "Vol avec Zigzag et Pop-up"

# Simulation par itération
while distance_missile > 0:
    # Calcul du mouvement du missile selon le mode de vol
    y_missile, z_missile = position_missile(temps, amplitude, frequence_zigzag, temps_pop_up_avant_impact, temps_total, mode_vol)

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

    # Calcul du temps de vol des obus
    temps_vol = distance_missile / vitesse_obus  # Temps que met chaque obus à atteindre la cible

    # Ajustement de la position du missile en fonction de la manœuvre et du temps de vol
    temps_reel = temps + temps_vol  # Le temps auquel l'obus touche la cible
    y_missile_ajuste, z_missile_ajuste = position_missile(temps_reel, amplitude, frequence_zigzag, temps_pop_up_avant_impact, temps_total, mode_vol)

    # Ajouter les impacts à notre compteur total si l'obus atteint la zone de la cible
    if abs(y_missile_ajuste) < rayon_cible:
        obstacle_hit_count += nombre_billes_toucher

    # Ajouter les valeurs dans les listes pour les graphiques
    temps_list.append(temps)
    obstacles_touch_list.append(obstacle_hit_count)
    distance_list.append(distance_missile)
    impacts_distance_list.append(obstacle_hit_count)
    x_pos_list.append(distance_missile)
    y_pos_list.append(y_missile_ajuste)
    z_pos_list.append(z_missile_ajuste)

    # Mise à jour de la distance
    distance_missile -= vitesse_missile_m_s * pas_temps
    temps += pas_temps

# Affichage des résultats
print(f"Le nombre total d'obus touchant la cible avant l'impact est : {obstacle_hit_count:.2f}")
if obstacle_hit_count >= 15:
    print("Le missile sera neutralisé.")
else:
    print("Le missile ne sera pas neutralisé.")

# Affichage des graphiques
plt.figure(figsize=(12, 8))

# Premier graphique : Nombre d'obus touchant la cible en fonction du temps
plt.subplot(3, 1, 1)
plt.plot(temps_list, obstacles_touch_list, label="Nombre d'obus touchant la cible", color='blue')
plt.xlabel('Temps (s)')
plt.ylabel('Nombre d\'obus touchant la cible')
plt.title('Nombre d\'obus touchant la cible en fonction du temps')
plt.grid(True)
plt.legend()

# Deuxième graphique : Nombre d'obus touchant la cible en fonction de la distance
plt.subplot(3, 1, 2)
plt.plot(distance_list, impacts_distance_list, label="Nombre d\'obus touchant la cible", color='red')
plt.xlabel('Distance (m)')
plt.ylabel('Nombre d\'obus touchant la cible')
plt.title('Nombre d\'obus touchant la cible en fonction de la distance')
plt.grid(True)
plt.legend()

# Graphique des positions x, y et z
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(temps_list, x_pos_list, label="Position x (distance)", color='green')
plt.xlabel('Temps (s)')
plt.ylabel('Position x (m)')
plt.title('Position x du missile (distance du navire) en fonction du temps')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(temps_list, y_pos_list, label="Position y (zigzag)", color='orange')
plt.xlabel('Temps (s)')
plt.ylabel('Position y (m)')
plt.title('Position y du missile (zigzag) en fonction du temps')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(temps_list, z_pos_list, label="Position z (pop-up)", color='purple')
plt.xlabel('Temps (s)')
plt.ylabel('Position z (m)')
plt.title('Position z du missile (pop-up) en fonction du temps')
plt.grid(True)
plt.legend()



plt.tight_layout()
plt.show()

# Graphique 3D de la trajectoire avec axes ajustés pour la même échelle
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_pos_list, y_pos_list, z_pos_list, label='Trajectoire du missile', color='blue')

# Définir les limites pour chaque axe
ax.set_xlim(0, 3000)   # Limites pour l'axe X
ax.set_ylim(-1500, 1500)  # Limites pour l'axe Y
ax.set_zlim(0, 3000)   # Limites pour l'axe Z

# Assurer que les proportions sont respectées entre les axes
ax.set_box_aspect([1, 1, 1])

# Paramétrage des labels et titre
ax.set_xlabel('Position x (distance du navire en m)')
ax.set_ylabel('Position y (zigzag en m)')
ax.set_zlabel('Position z (altitude en m)')
ax.set_title("Trajectoire 3D du missile avec échelle uniforme")
ax.legend()

plt.show()
