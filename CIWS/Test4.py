import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Données du missile
vitesse_missile = 0.27  # Vitesse du missile en km/s (Mach 0.8)
Surface_missile = 6 * 0.4  # Surface de la cible (6m x 0.4m)
distance_initiale = 15000  # Portée initiale en mètres (15 km)
distance_zigzag_start = 6000  # Distance de début du zigzag à partir de la cible
distance_zigzag_total = 6000  # Distance totale de zigzag en mètres
period_distance = 3000  # Distance d'une période complète de zigzag
g_max = 25  # Manœuvrabilité latérale maximale en termes de "g"
temps_pop_up_avant_impact = 5  # Temps avant l'impact pour déclencher le pop-up (en secondes)
altitude_max = 500  # Altitude maximale atteinte lors du pop-up (500 m)
altitude_missile = 3  # Altitude du missile avant le pop-up (3 m)
altitude_impact = 10  # Altitude d'impact (10 m)

# Données du CIWS
cadence_tir = 4500  # Cadence de tir en obus par minute
cadence_tir_par_seconde = cadence_tir / 60  # Cadence de tir en obus par seconde
vitesse_obus = 1000  # Vitesse des obus en m/s
distance_max_CIWS = 3000  # Portée max en mètres
distance_min_CIWS = 300  # Portée min en mètres
angle_dispersion_deg = 1  # Angle de dispersion du tir en degrés

# Conversion des unités
vitesse_missile_m_s = vitesse_missile * 1000  # Vitesse du missile en m/s
g = 9.81  # Accélération gravitationnelle en m/s^2

# Calcul de la durée totale et des périodes de zigzag
temps_total = distance_initiale / vitesse_missile_m_s
temps_zigzag_total = distance_zigzag_total / vitesse_missile_m_s


# Calcul de l'amplitude du zigzag en fonction de g_max
def calculer_amplitude(g_max, vitesse_missile_m_s, period_distance):
    T = period_distance / vitesse_missile_m_s  # Temps pour parcourir une période complète
    return (g_max * g * T ** 2) / (4 * math.pi ** 2)


amplitude_zigzag = calculer_amplitude(g_max, vitesse_missile_m_s, period_distance)


# Fonction pour calculer la position du missile
def position_missile(temps, amplitude_zigzag, period_distance, temps_pop_up_avant_impact, temps_total, mode_vol):
    # Distance restante
    distance_covered = vitesse_missile_m_s * temps
    distance_missile = distance_initiale - distance_covered

    # Mouvement en Y (zigzag) selon le mode
    if mode_vol == "Vol direct":
        y_missile = 0  # Pas de variation en Y
    elif mode_vol == "Vol manœuvrant":
        # Zigzag commence 6 km avant la cible
        if distance_missile <= distance_zigzag_start:
            distance_zigzag = distance_zigzag_start - distance_missile
            y_missile = amplitude_zigzag * math.sin(2 * math.pi * distance_zigzag / period_distance)
        else:
            y_missile = 0
    elif mode_vol == "Vol avec Pop-up":
        y_missile = 0  # Pas de variation en Y
    elif mode_vol == "Vol avec Zigzag et Pop-up":
        temps_restant = temps_total - temps
        if temps_restant > temps_pop_up_avant_impact:
            distance_pop_up = temps_pop_up_avant_impact * vitesse_missile_m_s
            if distance_missile <= distance_zigzag_start + distance_pop_up:
                distance_zigzag = distance_zigzag_start + distance_pop_up - distance_missile
                y_missile = amplitude_zigzag * math.sin(2 * math.pi * distance_zigzag / period_distance)
            else:
                y_missile = 0
        else:
            y_missile = 0

    # Mouvement en Z (altitude) selon le mode de vol
    if mode_vol == "Vol direct":
        z_missile = altitude_missile
    elif mode_vol == "Vol manœuvrant":
        z_missile = altitude_missile
    elif mode_vol == "Vol avec Pop-up":
        temps_restant = temps_total - temps
        if temps_restant <= temps_pop_up_avant_impact:
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) *
                    (temps_restant - temps_milieu) ** 2 + altitude_max
            )
        else:
            z_missile = altitude_missile
    elif mode_vol == "Vol avec Zigzag et Pop-up":
        temps_restant = temps_total - temps
        if temps_restant > temps_pop_up_avant_impact:
            z_missile = altitude_missile
        else:
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) *
                    (temps_restant - temps_milieu) ** 2 + altitude_max
            )

    return y_missile, z_missile


# Fonctions du CIWS
def rayon_dispersion(distance, angle_deg):
    angle_rad = math.radians(angle_deg)
    return math.tan(angle_rad) * distance

def surface_dispersion(rayon):
    return math.pi * rayon ** 2

def nombre_obus_tires(cadence_tir_par_seconde, temps):
    return cadence_tir_par_seconde * temps

# Simulation du tir du CIWS et impact
def simulation_CIWS(pas_temps, distance_missile, cadence_tir_par_seconde):
    if distance_min_CIWS <= distance_missile <= distance_max_CIWS:
        rayon_cible = rayon_dispersion(distance_missile, angle_dispersion_deg)
        surface_cible = surface_dispersion(rayon_cible)
        obus_tires = cadence_tir_par_seconde * pas_temps  # Obus tirés pendant le pas de temps
        densite_obus = obus_tires / surface_cible
        obus_touches = densite_obus * Surface_missile
        return obus_touches
    return 0

# Simulation
pas_temps = 0.1
mode_vol = "Vol avec Pop-up"

temps_list, x_pos_list, y_pos_list, z_pos_list, obus_impactes = [], [], [], [], 0
surface_dispersion_list = []
obus_touches_list = []
temps, distance_missile = 0, distance_initiale

while distance_missile > 0:
    y_missile, z_missile = position_missile(temps, amplitude_zigzag, period_distance, temps_pop_up_avant_impact, temps_total, mode_vol)
    obus_touches = simulation_CIWS(pas_temps, distance_missile, cadence_tir_par_seconde)
    obus_impactes += obus_touches

    # Calcul de la surface de dispersion
    rayon_cible = rayon_dispersion(distance_missile, angle_dispersion_deg)
    surface_disp = surface_dispersion(rayon_cible)

    # Stockage des données
    surface_dispersion_list.append(surface_disp)
    obus_touches_list.append(obus_touches)

    # Mise à jour des listes de positions et du temps
    temps_list.append(temps)
    x_pos_list.append(distance_missile)
    y_pos_list.append(y_missile)
    z_pos_list.append(z_missile)

    distance_missile -= vitesse_missile_m_s * pas_temps
    temps += pas_temps

# Graphique de la trajectoire et dispersion du CIWS
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(temps_list, x_pos_list, label="Position x (distance)", color='blue')
plt.xlabel("Temps (s)")
plt.ylabel("Position x (m)")

plt.subplot(3, 1, 2)
plt.plot(temps_list, y_pos_list, label="Position y (zigzag)", color='green')
plt.xlabel("Temps (s)")
plt.ylabel("Position y (m)")

plt.subplot(3, 1, 3)
plt.plot(temps_list, z_pos_list, label="Position z (altitude)", color='red')
plt.xlabel("Temps (s)")
plt.ylabel("Position z (m)")

plt.tight_layout()
plt.show()

# Affichage du nombre total d'obus impactant le missile
print(f"Nombre total d'obus impactant le missile : {obus_impactes:.2f}")
