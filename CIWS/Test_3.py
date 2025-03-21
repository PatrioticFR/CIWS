import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- Paramètres initiaux (unités en mètres et secondes) ---
vitesse_missile = 300  # Vitesse du missile en m/s (Mach 0.8 ≈ 300 m/s)
surface_missile = 6 * 0.4  # Surface de la cible en m² (6m x 0.4m)
distance_initiale = 15000  # Portée initiale en mètres (15 km)
distance_zigzag_start = 6000  # Distance de début du zigzag en mètres
period_distance = 2000  # Distance d'une période complète de zigzag en mètres
g_max = 5  # Manœuvrabilité latérale maximale en g
temps_pop_up_avant_impact = 7  # Temps avant impact pour pop-up en secondes
altitude_max = 200  # Altitude maximale lors du pop-up en mètres
altitude_missile = 3  # Altitude initiale en mètres
altitude_impact = 10  # Altitude d'impact en mètres

# Données du CIWS
cadence_tir = 4500  # Cadence de tir en obus par minute
cadence_tir_par_seconde = cadence_tir / 60  # 100 obus/s
vitesse_obus = 1100  # Vitesse des obus en m/s
distance_max_CIWS = 3000  # Portée max en mètres
distance_min_CIWS = 300  # Portée min en mètres
angle_dispersion_deg = 0.3  # Angle de dispersion en degrés
seuil_destruction = 10  # Nombre d'obus nécessaires pour neutraliser le missile



# Constantes
g = 9.81  # Accélération gravitationnelle en m/s²

# --- Modes de vol ---
mode_labels = {
    1: "Vol direct",
    2: "Vol manœuvrant",
    3: "Vol avec pop-up",
    4: "Vol avec zigzag et pop-up"
}

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
    if temps > temps_total:
        temps = temps_total
    distance_covered = vitesse_missile * temps
    distance_missile = distance_initiale - distance_covered

    if mode_vol == 1:
        y_missile = 0
    elif mode_vol == 2:
        if distance_missile <= distance_zigzag_start:
            distance_zigzag = distance_zigzag_start - distance_missile
            y_missile = amplitude_zigzag * np.sin(2 * np.pi * distance_zigzag / period_distance)
        else:
            y_missile = 0
    elif mode_vol == 3:
        y_missile = 0
    elif mode_vol == 4:
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

    if mode_vol in [1, 2]:
        z_missile = altitude_missile
    elif mode_vol == 3:
        temps_restant = temps_total - temps
        if temps_restant <= temps_pop_up_avant_impact:
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) *
                    (temps_restant - temps_milieu) ** 2 + altitude_max
            )
        else:
            z_missile = altitude_missile
    elif mode_vol == 4:
        temps_restant = temps_total - temps
        if temps_restant > temps_pop_up_avant_impact:
            z_missile = altitude_missile
        else:
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) *
                    (temps_restant - temps_milieu) ** 2 + altitude_max
            )

    return distance_missile, y_missile, z_missile


# --- Fonctions du CIWS ---
def rayon_dispersion(distance, angle_deg):
    angle_rad = np.radians(angle_deg)
    return np.tan(angle_rad) * distance


def surface_dispersion(rayon):
    return np.pi * rayon ** 2


def simulation_CIWS(temps, distance_missile, cadence_tir_par_seconde, pas_temps, mode_vol):
    if distance_min_CIWS <= distance_missile <= distance_max_CIWS:
        temps_vol_obus = distance_missile / vitesse_obus
        x_actuel, y_actuel, z_actuel = position_missile(temps, amplitude_zigzag, period_distance,
                                                        temps_pop_up_avant_impact, temps_total, mode_vol)
        x_pred = distance_missile - vitesse_missile * temps_vol_obus
        y_pred = y_actuel
        z_pred = z_actuel

        x_reel, y_reel, z_reel = position_missile(temps + temps_vol_obus, amplitude_zigzag, period_distance,
                                                  temps_pop_up_avant_impact, temps_total, mode_vol)

        rayon_cible = rayon_dispersion(distance_missile, angle_dispersion_deg)
        surface_cible = surface_dispersion(rayon_cible)
        obus_tires = cadence_tir_par_seconde * pas_temps
        densite_obus = obus_tires / surface_cible

        distance_erreur = np.sqrt((x_pred - x_reel) ** 2 + (y_pred - y_reel) ** 2 + (z_pred - z_reel) ** 2)
        if distance_erreur <= rayon_cible:
            obus_touches = min(densite_obus * surface_missile, obus_tires)
            return obus_touches
        else:
            return 0
    return 0


# --- Simulation pour les 4 modes ---
pas_temps = 0.01
temps_array = np.arange(0, temps_total + pas_temps, pas_temps)
results = {}

for mode in [1, 2, 3, 4]:
    x_pos_array = distance_initiale - vitesse_missile * temps_array
    y_pos_array = np.zeros_like(temps_array)
    z_pos_array = np.zeros_like(temps_array)
    obus_par_seconde = np.zeros_like(temps_array)
    obus_cumules = np.zeros_like(temps_array)
    obus_total = 0  # Nouveau : suivi explicite du total

    for i, t in enumerate(temps_array):
        if x_pos_array[i] <= 0:
            x_pos_array[i] = 0
            _, y_pos_array[i], z_pos_array[i] = position_missile(t, amplitude_zigzag, period_distance,
                                                                 temps_pop_up_avant_impact, temps_total, mode)
            break
        x_pos_array[i], y_pos_array[i], z_pos_array[i] = position_missile(t, amplitude_zigzag, period_distance,
                                                                          temps_pop_up_avant_impact, temps_total, mode)
        obus_touches = simulation_CIWS(t, x_pos_array[i], cadence_tir_par_seconde, pas_temps, mode)
        obus_par_seconde[i] = obus_touches / pas_temps
        obus_total += obus_touches  # Ajout au total
        obus_cumules[i] = obus_total  # Mise à jour du cumul

    results[mode] = {
        'x': x_pos_array,
        'y': y_pos_array,
        'z': z_pos_array,
        'obus': obus_total,  # Utilisation du total explicite
        'obus_par_seconde': obus_par_seconde,
        'obus_cumules': obus_cumules
    }

# --- Visualisation 3D côte à côte ---
fig = plt.figure(figsize=(20, 10))
for i, mode in enumerate([1, 2, 3, 4], 1):
    ax = fig.add_subplot(2, 2, i, projection='3d')
    ax.plot(results[mode]['x'], results[mode]['y'], results[mode]['z'], label=mode_labels[mode], color='blue')
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Altitude (m)")
    ax.set_title(mode_labels[mode])
    ax.legend()

plt.tight_layout()
plt.show()

# --- Visualisation 2D côte à côte ---
fig, axes = plt.subplots(3, 4, figsize=(20, 12), sharex=True, sharey='row')
for i, mode in enumerate([1, 2, 3, 4]):
    axes[0, i].plot(temps_array, results[mode]['x'], color='blue')
    axes[0, i].set_title(mode_labels[mode])
    axes[0, i].set_ylabel("Distance (m)")

    axes[1, i].plot(temps_array, results[mode]['y'], color='green')
    axes[1, i].set_ylabel("Y (m)")

    axes[2, i].plot(temps_array, results[mode]['z'], color='red')
    axes[2, i].set_xlabel("Temps (s)")
    axes[2, i].set_ylabel("Altitude (m)")

plt.tight_layout()
plt.show()

# --- Graphiques des obus touchés ---
fig, axes = plt.subplots(2, 4, figsize=(20, 8), sharex=True, sharey='row')
for i, mode in enumerate([1, 2, 3, 4]):
    axes[0, i].plot(temps_array, results[mode]['obus_par_seconde'], color='purple')
    axes[0, i].set_title(mode_labels[mode])
    axes[0, i].set_ylabel("Obus/s")

    axes[1, i].plot(temps_array, results[mode]['obus_cumules'], color='orange')
    axes[1, i].set_xlabel("Temps (s)")
    axes[1, i].set_ylabel("Obus cumulés")

plt.tight_layout()
plt.show()

# --- Résultats ---
print("\nRésultats pour chaque mode de vol :")
print("-" * 50)
print(f"{'Mode de vol':<25} | {'Obus impactés':<15} | {'Neutralisé':<10}")
print("-" * 50)
for mode in [1, 2, 3, 4]:
    obus = results[mode]['obus']
    neutralise = "Oui" if obus >= seuil_destruction else "Non"
    print(f"{mode_labels[mode]:<25} | {obus:<15.2f} | {neutralise:<10}")
print("-" * 50)