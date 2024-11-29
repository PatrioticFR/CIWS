import math

# Données du CIWS
cadence_tir = 200  # Cadence de tir en coups par minute
cadence_tir_par_seconde = cadence_tir / 60  # Cadence de tir en coups par seconde
distance_min = 50  # Distance minimale en mètres
distance_max = 4000  # Distance maximale en mètres

# Données du missile
mach_vitesse = {0.8: 0.27, 1.6: 0.53, 2.0: 0.61}  # Vitesse du missile en km/s (Mach 0.8, Mach 1.6, Mach 2.0)

# Données sur l'obus
explosion_distance = 20  # Distance de l'explosion en mètres
angle_total = 40  # Angle total de dispersion (en degrés)


# Calcul de la vitesse du missile en m/s
def vitesse_missile(mach, vitesse_acoustique=343):
    # Vitesse du missile en m/s en fonction de Mach et de la vitesse du son
    return mach * vitesse_acoustique


# Calcul du nombre de coups tirés avant l'impact
def nombre_coups(vitesse_missile, distance_max, distance_min, pas_temps=0.1):
    # Temps total pour que le missile atteigne la cible
    temps_total = (distance_max - distance_min) / vitesse_missile
    # Nombre de coups tirés pendant ce temps (en secondes)
    return cadence_tir_par_seconde * temps_total


# Calcul du rayon de dispersion des billes
def rayon_dispersion(distance, angle_deg):
    # Calcul du rayon de dispersion en fonction de la distance et de l'angle
    angle_rad = math.radians(angle_deg / 2)  # Demi-angle
    return math.tan(angle_rad) * distance


# Calcul du nombre de billes qui toucheront la cible
def nombre_billes_toucher(coups, rayon_dispersion):
    # Surface totale couverte par les billes à la distance d'explosion
    surface_total = math.pi * rayon_dispersion ** 2
    # Nombre de billes générées par explosion
    billes_par_explosion = 200  # Nombre de billes par explosion
    # Densité des billes par m²
    densite_billes = billes_par_explosion / surface_total
    # Surface de la cible (6m x 0.4m)
    surface_cible = 6 * 0.4  # Surface en mètres carrés
    # Nombre de billes touchant la cible
    return densite_billes * surface_cible * coups


# Calcul du nombre de coups pour chaque Mach
for mach in [0.8, 1.6, 2.0]:
    vitesse = vitesse_missile(mach)
    coups = nombre_coups(vitesse, distance_max, distance_min)
    rayon = rayon_dispersion(explosion_distance, angle_total)
    billes = nombre_billes_toucher(coups, rayon)

    # Affichage des résultats
    print(f"Pour un missile à Mach {mach}:")
    print(f"- Nombre de coups avant l'impact : {coups:.2f}")
    print(f"- Nombre de billes touchant la cible avant l'impact : {billes:.2f}")
    print("-" * 40)
