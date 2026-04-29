# Robotran_Mazda_MX5

Modèle multicorps d’une **Mazda MX-5** réalisé avec **Robotran / MBSysPad / MBsysPy** dans le cadre du cours **LEPL1504 – Projet 4 en mécanique**.

L’objectif du projet est de construire un modèle véhicule cohérent permettant d’étudier la **cinématique**, la **fermeture géométrique**, l’**équilibre**, puis la **dynamique directe** d’une Mazda MX-5.

---

## Aperçu du projet

Ce dépôt contient :

- le modèle `.mbs` de la voiture
- les fichiers symboliques générés par Robotran
- les fonctions utilisateur modifiées pour le projet
- un script principal Python pour lancer les simulations
- les résultats et animations générés

Le projet sert principalement à analyser :

- la structure géométrique du véhicule
- le comportement des suspensions et des roues
- la fermeture des boucles cinématiques
- l’influence des efforts externes, notamment les efforts pneus/sol
- la stabilité de la simulation en dynamique directe

---

## Structure du dépôt

```text
Robotran_Mazda_MX5/
│
├── animationR/        # Fichiers d’animation générés par Robotran
├── dataR/             # Données du modèle
│   └── Robotran_Mazda_MX5.mbs
├── resultsR/          # Résultats de simulation
├── symbolicR/         # Fichiers symboliques générés par Robotran
├── userfctR/          # Fonctions utilisateur personnalisées
├── workR/             # Script principal et fichiers d’exécution
│   └── main.py
├── README.md
└── requirement.md