import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Paramètres initiaux (unités en mètres et secondes) ---
vitesse_missile = 270  # Vitesse du missile en m/s (Mach 0.8 ≈ 270 m/s)
surface_missile = 6 * 0.4  # Surface de la cible en m² (6m x 0.4m)
distance_initiale = 15000  # Portée initiale en mètres (15 km)
distance_zigzag_start = 6000  # Distance de début du zigzag en mètres
period_distance = 3000  # Distance d'une période complète de zigzag en mètres
g_max = 25  # Manœuvrabilité latérale maximale en g
temps_pop_up_avant_impact = 5  # Temps avant impact pour pop-up en secondes
altitude_max = 500  # Altitude maximale lors du pop-up en mètres
altitude_missile = 3  # Altitude initiale en mètres
altitude_impact = 10  # Altitude d'impact en mètres

# Données du CIWS
cadence_tir = 4500  # Cadence de tir en obus par minute
cadence_tir_par_seconde = cadence_tir / 60  # Cadence de tir en obus par seconde
vitesse_obus = 1000  # Vitesse des obus en m/s
distance_max_CIWS = 3000  # Portée max en mètres
distance_min_CIWS = 300  # Portée min en mètres
angle_dispersion_deg = 1  # Angle de dispersion en degrés
seuil_destruction = 10  # Nombre d'obus nécessaires pour neutraliser le missile

# Constantes
g = 9.81  # Accélération gravitationnelle en m/s²

# --- Choix du mode de vol ---
# mode_vol = 1 : Vol direct (trajectoire droite à altitude constante)
# mode_vol = 2 : Vol manœuvrant (zigzag latéral à partir de 6 km)
# mode_vol = 3 : Vol avec pop-up (montée parabolique 5 s avant impact)
# mode_vol = 4 : Vol avec zigzag et pop-up (zigzag puis pop-up)
mode_vol = 4  # Modifier cette valeur (1, 2, 3 ou 4) pour changer le mode de vol

# --- Calculs préliminaires ---
temps_total = distance_initiale / vitesse_missile


# Calcul de l'amplitude du zigzag
def calculer_amplitude(g_max, vitesse_missile, period_distance):
    T = period_distance / vitesse_missile  # Période en secondes
    amplitude = (g_max * g * T ** 2) / (4 * np.pi ** 2)
    return amplitude


amplitude_zigzag = calculer_amplitude(g_max, vitesse_missile, period_distance)
print(f"Amplitude du zigzag : {amplitude_zigzag:.2f} m")


# --- Fonction de position du missile ---
def position_missile(temps, amplitude_zigzag, period_distance, temps_pop_up_avant_impact, temps_total, mode_vol):
    if temps > temps_total:  # Limiter le temps au temps total
        temps = temps_total
    distance_covered = vitesse_missile * temps
    distance_missile = distance_initiale - distance_covered

    # Mouvement en Y (zigzag)
    if mode_vol == 1:  # Vol direct
        y_missile = 0
    elif mode_vol == 2:  # Vol manœuvrant
        if distance_missile <= distance_zigzag_start:
            distance_zigzag = distance_zigzag_start - distance_missile
            y_missile = amplitude_zigzag * np.sin(2 * np.pi * distance_zigzag / period_distance)
        else:
            y_missile = 0
    elif mode_vol == 3:  # Vol avec pop-up
        y_missile = 0
    elif mode_vol == 4:  # Vol avec zigzag et pop-up
        temps_restant = temps_total - temps
        if temps_restant > temps_pop_up_avant_impact:
            distance_pop_up = temps_pop_up_avant_impact * vitesse_missile
            if distance_missile <= distance_zigzag_start + distance_pop_up:
                distance_zigzag = distance_zigzag_start + distance_pop_up - distance_missile
                y_missile = amplitude_zigzag * np.sin(2 * np.pi * distance_zigzag / period_distance)
            else:
                y_missile = 0
        else:
            y_missile = 0

    # Mouvement en Z (altitude)
    if mode_vol in [1, 2]:  # Vol direct ou manœuvrant
        z_missile = altitude_missile
    elif mode_vol == 3:  # Vol avec pop-up
        temps_restant = temps_total - temps
        if temps_restant <= temps_pop_up_avant_impact:
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) *
                    (temps_restant - temps_milieu) ** 2 + altitude_max
            )
        else:
            z_missile = altitude_missile
    elif mode_vol == 4:  # Vol avec zigzag et pop-up
        temps_restant = temps_total - temps
        if temps_restant > temps_pop_up_avant_impact:
            z_missile = altitude_missile
        else:
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) *
                    (temps_restant - temps_milieu) ** 2 + altitude_max
            )

    return distance_missile, y_missile, z_missile  # Retourne aussi la position x


# --- Fonctions du CIWS ---
def rayon_dispersion(distance, angle_deg):
    angle_rad = np.radians(angle_deg)
    return np.tan(angle_rad) * distance


def surface_dispersion(rayon):
    return np.pi * rayon ** 2


def simulation_CIWS(temps, distance_missile, cadence_tir_par_seconde, pas_temps, mode_vol):
    if distance_min_CIWS <= distance_missile <= distance_max_CIWS:
        # Temps de vol des obus jusqu'à la position actuelle du missile
        temps_vol_obus = distance_missile / vitesse_obus

        # Position prédite du missile quand les obus arriveront
        x_pred, y_pred, z_pred = position_missile(temps + temps_vol_obus, amplitude_zigzag, period_distance,
                                                  temps_pop_up_avant_impact, temps_total, mode_vol)

        # Position réelle du missile au moment du tir (temps actuel)
        x_actuel, y_actuel, z_actuel = position_missile(temps, amplitude_zigzag, period_distance,
                                                        temps_pop_up_avant_impact, temps_total, mode_vol)

        # Position réelle du missile quand les obus arrivent
        x_reel, y_reel, z_reel = position_missile(temps + temps_vol_obus, amplitude_zigzag, period_distance,
                                                  temps_pop_up_avant_impact, temps_total, mode_vol)

        # Calcul de la dispersion à la distance prédite
        rayon_cible = rayon_dispersion(distance_missile, angle_dispersion_deg)
        surface_cible = surface_dispersion(rayon_cible)
        obus_tires = cadence_tir_par_seconde * pas_temps
        densite_obus = obus_tires / surface_cible

        # Distance entre la position prédite et la position réelle
        distance_erreur = np.sqrt((x_pred - x_reel) ** 2 + (y_pred - y_reel) ** 2 + (z_pred - z_reel) ** 2)

        # Si l'erreur est dans le rayon de dispersion, les obus peuvent toucher
        if distance_erreur <= rayon_cible:
            obus_touches = min(densite_obus * surface_missile, obus_tires)
            return obus_touches
        else:
            return 0  # Les obus manquent car le missile a manœuvré hors de la zone
    return 0


# --- Simulation avec NumPy ---
pas_temps = 0.1
temps_array = np.arange(0, temps_total + pas_temps, pas_temps)
x_pos_array = distance_initiale - vitesse_missile * temps_array
y_pos_array = np.zeros_like(temps_array)
z_pos_array = np.zeros_like(temps_array)
obus_impactes = 0

for i, t in enumerate(temps_array):
    if x_pos_array[i] <= 0:  # Arrêt précis à la cible
        x_pos_array[i] = 0
        _, y_pos_array[i], z_pos_array[i] = position_missile(t, amplitude_zigzag, period_distance,
                                                             temps_pop_up_avant_impact, temps_total, mode_vol)
        break
    x_pos_array[i], y_pos_array[i], z_pos_array[i] = position_missile(t, amplitude_zigzag, period_distance,
                                                                      temps_pop_up_avant_impact, temps_total, mode_vol)
    obus_impactes += simulation_CIWS(t, x_pos_array[i], cadence_tir_par_seconde, pas_temps, mode_vol)

# --- Visualisation ---
mode_labels = {1: "Vol direct", 2: "Vol manœuvrant", 3: "Vol avec pop-up", 4: "Vol avec zigzag et pop-up"}
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_pos_array, y_pos_array, z_pos_array, label=f"Trajectoire ({mode_labels[mode_vol]})", color='blue')
ax.set_xlabel("Distance (m)")
ax.set_ylabel("Y - Zigzag (m)")
ax.set_zlabel("Altitude (m)")
ax.set_title(f"Trajectoire 3D - {mode_labels[mode_vol]}")
plt.legend()
plt.show()

# Graphiques 2D (optionnels)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
ax1.plot(temps_array, x_pos_array, label="Distance", color='blue')
ax1.set_ylabel("Distance (m)")
ax1.legend()

ax2.plot(temps_array, y_pos_array, label="Y (zigzag)", color='green')
ax2.set_ylabel("Y (m)")
ax2.legend()

ax3.plot(temps_array, z_pos_array, label="Altitude", color='red')
ax3.set_xlabel("Temps (s)")
ax3.set_ylabel("Altitude (m)")
ax3.legend()

plt.tight_layout()
plt.show()

# --- Résultats ---
print(f"Nombre total d'obus impactant le missile : {obus_impactes:.2f}")
if obus_impactes >= seuil_destruction:
    print(f"Le missile a été neutralisé (seuil de destruction : {seuil_destruction} obus).")
else:
    print(f"Le missile n'a pas été neutralisé (seuil de destruction : {seuil_destruction} obus).")