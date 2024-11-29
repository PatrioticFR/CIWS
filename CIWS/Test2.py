import math
import matplotlib.pyplot as plt

# Données initiales
cadence_tir = 4500  # Cadence de tir en obus par minute
cadence_tir_par_seconde = cadence_tir / 60  # Cadence de tir en obus par seconde
vitesse_missile = 0.27  # Vitesse du missile en km/s (Mach 0.8)
vitesse_obus = 1000  # Vitesse des obus en m/s
distance_initiale = 15000  # Portée étendue pour la simulation en mètres (15 km)
distance_zigzag_start = 6000  # Distance de début du zigzag à partir de la cible
distance_zigzag_total = 6000  # Distance totale de manœuvre en zigzag en mètres
period_distance = 2000  # Distance d'une période complète de zigzag
g_max = 25  # Manœuvrabilité latérale maximale en termes de "g"
temps_pop_up_avant_impact = 5  # Temps avant l'impact pour déclencher le pop-up (en secondes)
altitude_max = 500  # Altitude maximale atteinte lors du pop-up (500 m)
altitude_missile = 3  # Altitude du missile avant le pop-up (3 m)
altitude_impact = 10  # Altitude d'impact (10 m)

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
            # Zigzag commence 6 km avant la cible
            distance_pop_up = temps_pop_up_avant_impact * vitesse_missile_m_s
            if distance_missile <= distance_zigzag_start + distance_pop_up:
                distance_zigzag = distance_zigzag_start + distance_pop_up - distance_missile
                y_missile = amplitude_zigzag * math.sin(2 * math.pi * distance_zigzag / period_distance)
            else:
                y_missile = 0
        else:
            y_missile = 0  # Pas de zigzag pendant le pop-up

    # Mouvement en Z (altitude) selon le mode de vol
    if mode_vol == "Vol direct":
        z_missile = altitude_missile
    elif mode_vol == "Vol manœuvrant":
        z_missile = altitude_missile
    elif mode_vol == "Vol avec Pop-up":
        temps_restant = temps_total - temps
        if temps_restant <= temps_pop_up_avant_impact:
            # Parabole pour le pop-up
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
            # Parabole pour le pop-up
            temps_milieu = temps_pop_up_avant_impact / 2
            z_missile = (
                    (altitude_max - altitude_impact) * (-4 / (temps_pop_up_avant_impact ** 2)) *
                    (temps_restant - temps_milieu) ** 2 + altitude_max
            )

    return y_missile, z_missile


# Paramètres pour la simulation
pas_temps = 0.1  # Pas de temps en secondes
mode_vol = "Vol avec Zigzag et Pop-up"

# Initialisation des listes pour la visualisation
temps_list = []
x_pos_list = []
y_pos_list = []
z_pos_list = []

# Boucle de simulation
temps = 0
distance_missile = distance_initiale

while distance_missile > 0:
    # Calcul de la position du missile selon le temps et le mode de vol
    y_missile, z_missile = position_missile(
        temps, amplitude_zigzag, period_distance, temps_pop_up_avant_impact, temps_total, mode_vol
    )

    # Stockage des données pour le graphique
    temps_list.append(temps)
    x_pos_list.append(distance_missile)
    y_pos_list.append(y_missile)
    z_pos_list.append(z_missile)

    # Mise à jour de la distance et du temps
    distance_missile -= vitesse_missile_m_s * pas_temps
    temps += pas_temps

# Affichage des résultats
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(temps_list, x_pos_list, label="Position x (distance)", color='green')
plt.xlabel('Temps (s)')
plt.ylabel('Position x (m)')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(temps_list, y_pos_list, label="Position y (zigzag)", color='orange')
plt.xlabel('Temps (s)')
plt.ylabel('Position y (m)')
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(temps_list, z_pos_list, label="Position z (pop-up)", color='purple')
plt.xlabel('Temps (s)')
plt.ylabel('Position z (m)')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.show()

# Graphique 3D de la trajectoire avec axes ajustés pour la même échelle
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_pos_list, y_pos_list, z_pos_list, label='Trajectoire du missile', color='blue')

# Définir les limites pour chaque axe
ax.set_xlim(0, distance_initiale)   # Limites pour l'axe X
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

