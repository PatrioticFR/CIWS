# Simulateur d'Engagement CIWS contre Missile Antinavire

## 📚 Description

Ce projet propose une série de scripts Python conçus pour **simuler l'engagement d'un système de défense rapprochée (CIWS - Close-In Weapon System) contre un missile antinavire manœuvrant**. L'objectif est d'évaluer l'efficacité de différents systèmes CIWS face à diverses trajectoires de missiles, en calculant la probabilité d'interception et le nombre total d'impacts (obus ou fragments) sur la cible.

La simulation modélise la trajectoire du missile en 3D, en intégrant plusieurs modes de vol possibles, et simule la réponse du CIWS, incluant le tir, la dispersion des projectiles, les erreurs de poursuite et l'effet du brouillage.

## ✨ Fonctionnalités Principales

* **Modélisation du Missile :**
    * Trajectoire 3D (Distance, Azimut/Zigzag, Altitude).
    * Plusieurs modes de vol simulés :
        * Vol direct à basse altitude.
        * Vol avec manœuvres évasives latérales (zigzag) basé sur l'accélération G maximale.
        * Vol avec manœuvre terminale "pop-up" (montée puis descente rapide).
        * Combinaison de zigzag et pop-up.
    * Paramètres configurables (vitesse, portée, capacité de manœuvre, altitude, dimensions).
* **Modélisation du CIWS :**
    * **Base de données étendue :** Inclut les caractéristiques de nombreux systèmes CIWS réels (Phalanx, Goalkeeper, AK-630, Type 1130, Kashtan, Millenium Gun, RapidFire, etc.).
    * **Paramètres clés :** Cadence de tir, vitesse des projectiles, portée minimale/maximale, angle de dispersion.
    * **Modélisation de la Poursuite :** Facteur de précision de base, affecté par :
        * Le type de radar (local ou déporté).
        * La présence de capteurs électro-optiques (EO).
        * Un niveau de **brouillage** configurable.
    * **Munitions Avancées :** Prise en compte optionnelle des obus à fragmentation avec fusée de proximité (type AHEAD), incluant rayon d'explosion, nombre de fragments et type de fragmentation (directionnelle, guidée, omnidirectionnelle).
    * **Cadence Variable :** Possibilité de simuler une cadence de tir réduite pour certains modes de vol ou systèmes.
* **Simulation de l'Interception :**
    * Calcul du temps de vol des projectiles.
    * Prédiction de la position future du missile par le CIWS (avec erreur de poursuite).
    * Calcul de la **probabilité d'impact** basée sur la distance entre la position prédite et la position réelle du missile, la dispersion des tirs et la surface du missile (utilisant une distribution Normale pour la dispersion).
    * Calcul du **nombre cumulé d'impacts** (obus directs ou fragments) au fil du temps.
    * Détermination de la **neutralisation** basée sur un seuil d'impacts configurable.
* **Visualisation :**
    * Graphiques **3D** des trajectoires de missile pour les différents modes de vol.
    * Graphiques **2D** montrant l'évolution temporelle de la distance, des coordonnées Y/Z, du nombre d'impacts par seconde et des impacts cumulés.
    * Graphiques comparatifs avancés (Bar plots, Heatmaps, Radar charts) pour évaluer et comparer l'efficacité de plusieurs systèmes CIWS dans différents scénarios.

## 🔧 Détails Techniques

* **Langage :** Python 3
* **Bibliothèques Principales :** NumPy (pour les calculs vectoriels et la génération de séquences temporelles), Matplotlib (pour la visualisation 2D et 3D), SciPy (pour la fonction de répartition Normale `norm.cdf` utilisée dans le calcul de probabilité d'impact).
* **Simulation :** Basée sur une discrétisation temporelle (pas de temps `dt`). La position du missile est calculée analytiquement à chaque pas de temps. L'interception est évaluée à chaque pas en calculant la probabilité d'impact pour les obus tirés pendant cet intervalle.

## 🚀 Utilisation (Exemple avec `Test_6.py`)

1.  **Configurer la Simulation :**
    * Modifiez les paramètres de l'objet `exocet` pour définir les caractéristiques du missile (vitesse, portée, manœuvres...).
    * Ajustez la liste `ciws_systems` si nécessaire, ou modifiez les paramètres d'un système existant.
    * Réglez le pas de temps `dt` (un `dt` plus petit augmente la précision mais ralentit la simulation).
    * Choisissez les `modes` de vol à simuler.
    * Ajustez le `jamming_level`.
2.  **Lancer le Script :**
    ```bash
    python Test_6.py
    ```
3.  **Analyser les Résultats :**
    * La console affichera les résultats synthétisés pour chaque système CIWS et chaque mode de vol, indiquant le nombre total d'impacts et si le seuil de neutralisation a été atteint.
    * Des fenêtres Matplotlib s'ouvriront (si l'affichage interactif est activé) pour montrer les graphiques 3D et 2D des trajectoires et des impacts pour les systèmes sélectionnés dans `selected_ciws`.
    * Le script `Visualisation.py` génère des graphiques comparatifs plus élaborés et les sauvegarde sous forme de fichiers PNG.

---
