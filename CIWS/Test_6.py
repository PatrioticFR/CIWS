import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.stats import norm


class Missile:
    def __init__(self, name, speed, surface, range, maneuver_g, zigzag_start, zigzag_period, popup_time, popup_altitude,
                 base_altitude, impact_altitude):
        self.name = name
        self.speed = speed
        self.surface = surface
        self.range = range
        self.maneuver_g = maneuver_g
        self.zigzag_start = zigzag_start
        self.zigzag_period = zigzag_period
        self.popup_time = popup_time
        self.popup_altitude = popup_altitude
        self.base_altitude = base_altitude
        self.impact_altitude = impact_altitude
        self.amplitude = self.calculate_zigzag_amplitude()

    def calculate_zigzag_amplitude(self):
        T = self.zigzag_period / self.speed
        return (self.maneuver_g * 9.81 * T ** 2) / (4 * np.pi ** 2)

    def position(self, time, total_time, mode):
        if time > total_time:
            time = total_time
        distance_covered = self.speed * time
        distance = self.range - distance_covered

        if mode == 1:  # Vol direct
            y = 0
        elif mode == 2:  # Vol manœuvrant (zigzag)
            if distance <= self.zigzag_start:
                distance_zigzag = self.zigzag_start - distance
                y = self.amplitude * np.sin(2 * np.pi * distance_zigzag / self.zigzag_period)
            else:
                y = 0
        elif mode == 3:  # Vol avec pop-up
            y = 0
        elif mode == 4:  # Vol combiné (zigzag + pop-up)
            time_remaining = total_time - time
            if time_remaining > self.popup_time:
                if distance <= self.zigzag_start + (self.popup_time * self.speed):
                    distance_zigzag = self.zigzag_start + (self.popup_time * self.speed) - distance
                    y = self.amplitude * np.sin(2 * np.pi * distance_zigzag / self.zigzag_period)
                else:
                    y = 0
            else:
                y = 0

        if mode in [1, 2]:  # Altitude constante
            z = self.base_altitude
        elif mode in [3, 4]:  # Pop-up
            time_remaining = total_time - time
            if time_remaining <= self.popup_time:
                t_mid = self.popup_time / 2
                z = (self.popup_altitude - self.impact_altitude) * (-4 / (self.popup_time ** 2)) * (
                            time_remaining - t_mid) ** 2 + self.popup_altitude
            else:
                z = self.base_altitude
        return distance, y, z


class CIWS:
    def __init__(self, name, fire_rate, projectile_speed, max_range, min_range, dispersion_angle, base_tracking_factor,
                 kill_threshold, radar_local=True, eo_sensor=False, proximity_fuse=None, variable_rate=False):
        self.name = name
        self.fire_rate = fire_rate / 60  # RPM -> RPS
        self.projectile_speed = projectile_speed
        self.max_range = max_range
        self.min_range = min_range
        self.dispersion_angle = dispersion_angle
        self.base_tracking_factor = base_tracking_factor
        self.kill_threshold = kill_threshold
        self.radar_local = radar_local
        self.eo_sensor = eo_sensor
        self.proximity_fuse = proximity_fuse
        self.variable_rate = variable_rate

    def adjust_fire_rate(self, mode):
        if self.variable_rate and mode in [3, 4]:  # Réduction pour pop-up ou combiné
            return self.fire_rate * 0.5
        return self.fire_rate

    def dispersion_radius(self, distance):
        return distance * np.tan(np.radians(self.dispersion_angle))

    def adjust_tracking_factor(self, jamming_level):
        tracking = self.base_tracking_factor
        if self.radar_local:
            tracking *= (1 - jamming_level * 0.2)
        else:
            tracking *= (1 - jamming_level * 0.4)
        if self.eo_sensor:
            tracking = min(self.base_tracking_factor,
                           tracking + (self.base_tracking_factor - tracking) * 0.7 * (1 - jamming_level))
        return max(0.1, tracking)

    def simulate_intercept(self, time, missile_distance, missile, mode, dt, jamming_level=0.2):
        if self.min_range <= missile_distance <= self.max_range:
            flight_time = missile_distance / self.projectile_speed
            x_curr, y_curr, z_curr = missile.position(time, missile.range / missile.speed, mode)
            tracking_factor = self.adjust_tracking_factor(jamming_level)
            x_pred = x_curr - missile.speed * flight_time
            y_pred = y_curr * (0.8 if self.eo_sensor and mode != 1 else (1 - tracking_factor))
            z_pred = z_curr * (0.8 if self.eo_sensor and mode != 1 else (1 - tracking_factor))
            x_real, y_real, z_real = missile.position(time + flight_time, missile.range / missile.speed, mode)

            radius = self.dispersion_radius(missile_distance)
            shots_fired = self.adjust_fire_rate(mode) * dt

            if self.proximity_fuse:
                explosion_radius = self.proximity_fuse['explosion_distance'] * np.tan(
                    np.radians(self.proximity_fuse['dispersion_angle'] / 2))
                dispersion_area = np.pi * explosion_radius ** 2
                shot_density = (self.proximity_fuse['fragments'] * shots_fired) / dispersion_area
                error_distance = np.sqrt((x_pred - x_real) ** 2 + (y_pred - y_real) ** 2 + (z_pred - z_real) ** 2)

                if self.proximity_fuse['fragmentation_type'] == 'directional':
                    hit_prob = min(1.0, 0.7 * explosion_radius / (error_distance + 0.1))
                elif self.proximity_fuse['fragmentation_type'] == 'guided':
                    hit_prob = max(0, 0.95 - error_distance / (explosion_radius * 2))
                elif self.proximity_fuse['fragmentation_type'] == 'omnidirectional':
                    hit_prob = min(1.0, 0.6 * explosion_radius / (error_distance + 0.1))

                if self.variable_rate and mode in [3, 4]:
                    hit_prob = min(1.0, hit_prob * 1.2)

                expected_hits = shot_density * missile.surface * hit_prob
                expected_hits = min(expected_hits, shots_fired * self.proximity_fuse['fragments'] * 0.05)
            else:
                dispersion_area = np.pi * radius ** 2 * (1 - tracking_factor)
                shot_density = shots_fired / max(dispersion_area, missile.surface)
                error_distance = np.sqrt((x_pred - x_real) ** 2 + (y_pred - y_real) ** 2 + (z_pred - z_real) ** 2)
                hit_prob = norm.cdf(radius, loc=error_distance, scale=radius / 4) if mode != 1 else 0.95
                if self.variable_rate and mode in [3, 4]:
                    hit_prob = min(1.0, hit_prob * 1.2)
                expected_hits = shot_density * missile.surface * hit_prob * 2
                expected_hits = min(expected_hits, shots_fired * 1.5)

            return expected_hits
        return 0


exocet = Missile(
    name="Exocet MM40", speed=300, surface=2.0, range=5000,
    maneuver_g=5, zigzag_start=1000, zigzag_period=1000,
    popup_time=2, popup_altitude=10, base_altitude=3, impact_altitude=1
)

ciws_systems = [
    CIWS("AK-230", 2000, 1050, 2000, 400, 0.6, 0.5, 20, False, False),
    CIWS("Type 69", 2000, 1050, 2000, 400, 0.6, 0.5, 20, False, False),
    CIWS("AK-630", 4500, 880, 1500, 350, 0.4, 0.6, 20, False, True),
    CIWS("AK-630M", 4500, 880, 1500, 350, 0.4, 0.6, 20, False, True),
    CIWS("AK-630M2 Duet", 10000, 890, 2000, 300, 0.3, 0.8, 15, False, True),
    CIWS("H/PJ-13", 4500, 880, 1500, 350, 0.3, 0.75, 15, False, True),
    CIWS("Karmand", 4500, 880, 2000, 350, 0.3, 0.75, 15, False, True),
    CIWS("Kashtan CIWS", 9000, 880, 1500, 300, 0.4, 0.7, 20, False, True),
    CIWS("Kashtan-M", 10000, 960, 2000, 200, 0.3, 0.85, 15, False, True),
    CIWS("Pantsir-M", 10000, 960, 2000, 200, 0.2, 0.95, 15, False, True),
    CIWS("Palma / Palash", 10000, 960, 2000, 200, 0.2, 0.95, 15, True, True),
    CIWS("Phalanx Block 0", 3000, 1100, 1500, 200, 0.4, 0.6, 30, True, False),
    CIWS("Phalanx Block 1", 4500, 1100, 1500, 200, 0.4, 0.65, 30, True, False),
    CIWS("Phalanx Block 1A", 4500, 1100, 1500, 200, 0.3, 0.7, 25, True, False),
    CIWS("Phalanx Block 1B", 4500, 1100, 1500, 150, 0.3, 0.8, 25, True, True),
    CIWS("Phalanx Block 1B Baseline 2", 4500, 1100, 1500, 150, 0.3, 0.85, 25, True, True),
    CIWS("Type 76A", 750, 1000, 4500, 700, 0.5, 0.5, 30, False, False,
         {'explosion_distance': 10, 'dispersion_angle': 30, 'fragments': 20, 'fragmentation_type': 'omnidirectional'}),
    CIWS("Type 730 / H/PJ-12", 5800, 880, 1500, 200, 0.3, 0.8, 15, True, True),
    CIWS("Type 730B", 5800, 880, 1500, 350, 0.3, 0.8, 15, True, True),
    CIWS("Type 730C", 4000, 880, 2000, 150, 0.3, 0.85, 15, True, True),
    CIWS("Type 1130 / H/PJ-11", 11000, 880, 1500, 200, 0.2, 0.95, 15, True, True),
    CIWS("OTO Melara 76mm Strales", 120, 905, 8000, 500, 0.3, 0.9, 2, True, True,
         {'explosion_distance': 15, 'dispersion_angle': 20, 'fragments': 20, 'fragmentation_type': 'guided'}),
    CIWS("DARDO / 40L70 Compact", 600, 1025, 2000, 400, 0.4, 0.65, 10, True, True),
    CIWS("Single Fast Forty", 450, 1025, 2000, 400, 0.4, 0.65, 10, True, True),
    CIWS("Twin Fast Forty", 900, 1025, 2000, 400, 0.4, 0.65, 10, True, True),
    CIWS("GOKDENIZ", 1100, 1175, 2500, 150, 0.3, 0.9, 20, True, True,
         {'explosion_distance': 10, 'dispersion_angle': 30, 'fragments': 152, 'fragmentation_type': 'directional'}),
    CIWS("GOKDENIZ ER", 1100, 1175, 2500, 150, 0.3, 0.9, 20, True, True,
         {'explosion_distance': 10, 'dispersion_angle': 30, 'fragments': 152, 'fragmentation_type': 'directional'}),
    CIWS("Sea Zenith", 3200, 1100, 1500, 300, 0.4, 0.7, 25, True, True),
    CIWS("Oerlikon Millennium Gun", 1000, 1175, 2500, 300, 0.2, 0.9, 20, True, True,
         {'explosion_distance': 10, 'dispersion_angle': 30, 'fragments': 152, 'fragmentation_type': 'directional'}),
    CIWS("Sea Snake 30 mm", 1100, 1050, 2000, 150, 0.3, 0.85, 15, False, True),
    CIWS("RapidFire", 200, 1000, 2000, 50, 0.2, 0.95, 10, False, True,
         {'explosion_distance': 20, 'dispersion_angle': 40, 'fragments': 200, 'fragmentation_type': 'directional'}),
    CIWS("Denel 35 mm DPG", 1100, 1175, 2000, 300, 0.3, 0.85, 15, True, True),
    CIWS("Meroka CIWS", 1440, 1290, 1500, 250, 0.6, 0.5, 30, False, False),
    CIWS("OSU-35K", 550, 1440, 2000, 150, 0.3, 0.9, 15, False, True),
    CIWS("Goalkeeper CIWS", 4200, 1050, 2000, 300, 0.2, 0.9, 15, True, True),
    # Nouveaux systèmes à cadence variable
    CIWS("Phalanx Block 1B Baseline 2 (Low Rate)", 4500, 1100, 1500, 150, 0.3, 0.85, 25, True, True, None, True),
    CIWS("Oerlikon Millennium Gun (Low Rate)", 1000, 1175, 2500, 300, 0.2, 0.9, 20, True, True,
         {'explosion_distance': 10, 'dispersion_angle': 30, 'fragments': 152, 'fragmentation_type': 'directional'},
         True),
    CIWS("Goalkeeper CIWS (Low Rate)", 4200, 1050, 2000, 300, 0.2, 0.9, 15, True, True, None, True),
    CIWS("RAPIDSeaGuardian", 600, 1000, 2000, 100, 0.25, 0.9, 10, True, True,
         {'explosion_distance': 15, 'dispersion_angle': 40, 'fragments': 100, 'fragmentation_type': 'directional'},
         True),
    CIWS("Skyguard 35mm", 1000, 1175, 2500, 200, 0.2, 0.85, 15, True, True,
         {'explosion_distance': 10, 'dispersion_angle': 30, 'fragments': 152, 'fragmentation_type': 'directional'},
         True),
]

dt = 0.01
modes = [1, 2, 3, 4]
mode_labels = {1: "Vol direct", 2: "Vol manœuvrant", 3: "Vol pop-up", 4: "Vol combiné"}
total_time = exocet.range / exocet.speed
time_array = np.arange(0, total_time + dt, dt)
results = {ciws.name: {} for ciws in ciws_systems}
jamming_level = 0.2

print(f"Amplitude du zigzag : {exocet.amplitude:.2f} m")

for ciws in ciws_systems:
    for mode in modes:
        x_pos = exocet.range - exocet.speed * time_array
        y_pos = np.zeros_like(time_array)
        z_pos = np.zeros_like(time_array)
        hits_per_sec = np.zeros_like(time_array)
        cumulative_hits = np.zeros_like(time_array)
        total_hits = 0
        for i, t in enumerate(time_array):
            if x_pos[i] <= 0:
                x_pos[i] = 0
                _, y_pos[i], z_pos[i] = exocet.position(t, total_time, mode)
                break
            x_pos[i], y_pos[i], z_pos[i] = exocet.position(t, total_time, mode)
            hits = ciws.simulate_intercept(t, x_pos[i], exocet, mode, dt, jamming_level)
            total_hits += hits
            hits_per_sec[i] = hits / dt
            cumulative_hits[i] = total_hits
        results[ciws.name][mode] = {
            'x': x_pos, 'y': y_pos, 'z': z_pos,
            'hits': total_hits, 'hits_per_sec': hits_per_sec, 'cumulative_hits': cumulative_hits
        }

for ciws in ciws_systems:
    print(f"\nRésultats pour {ciws.name} (brouillage = {jamming_level}):")
    print("-" * 50)
    print(f"{'Mode de vol':<25} | {'Hits (obus/fragments)':<20} | {'Neutralisé':<10}")
    print("-" * 50)
    for mode in modes:
        hits = results[ciws.name][mode]['hits']
        neutralized = "Oui" if hits >= ciws.kill_threshold else "Non"
        hit_label = "Fragments" if ciws.proximity_fuse else "Obus"
        print(f"{mode_labels[mode]:<25} | {hits:<20.2f} ({hit_label}) | {neutralized:<10}")
    print("-" * 50)

selected_ciws = ["Phalanx Block 1B", "Goalkeeper CIWS", "Type 1130 / H/PJ-11", "RapidFire", "Oerlikon Millennium Gun",
                 "GOKDENIZ",
                 "Phalanx Block 1B Baseline 2 (Low Rate)", "Oerlikon Millennium Gun (Low Rate)",
                 "Goalkeeper CIWS (Low Rate)",
                 "RAPIDSeaGuardian", "Skyguard 35mm"]
for ciws_name in selected_ciws:
    ciws = next(c for c in ciws_systems if c.name == ciws_name)
    fig = plt.figure(figsize=(20, 10))
    for i, mode in enumerate(modes, 1):
        ax = fig.add_subplot(2, 2, i, projection='3d')
        ax.plot(results[ciws.name][mode]['x'], results[ciws.name][mode]['y'], results[ciws.name][mode]['z'],
                label=mode_labels[mode], color='blue')
        ax.set_xlabel("Distance (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Altitude (m)")
        ax.set_title(f"{ciws.name} - {mode_labels[mode]}")
        ax.legend()
    plt.tight_layout()
    #plt.show()

    fig, axes = plt.subplots(2, 4, figsize=(20, 8), sharex=True, sharey='row')
    for i, mode in enumerate(modes):
        axes[0, i].plot(time_array, results[ciws.name][mode]['hits_per_sec'], color='purple')
        axes[0, i].set_title(f"{ciws.name} - {mode_labels[mode]}")
        axes[0, i].set_ylabel("Hits/s (obus ou fragments)")
        axes[1, i].plot(time_array, results[ciws.name][mode]['cumulative_hits'], color='orange')
        axes[1, i].set_xlabel("Temps (s)")
        axes[1, i].set_ylabel("Hits cumulés")
    plt.tight_layout()
    #plt.show()