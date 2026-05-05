#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import MBsysPy as Robotran
import os
import numpy as np
import matplotlib.pyplot as plt

# =============================================================================
# 1. PARAMÈTRES DE LA SIMULATION
# =============================================================================
simulation = "virage"  # Options: "MRU", "acceleration", "freinage", "dos_d_ane", "virage", "evitement"
vitesse_kmh = {"MRU": 36, "acceleration": 7, "freinage": 70, "dos_d_ane": 60, "virage": 30, "evitement": 60}[simulation]

print(f"--- Démarrage du projet Mazda MX-5 : Mode {simulation} ---")

# Chargement du projet
work_dir = os.path.dirname(os.path.abspath(__file__))
mbs_file = os.path.normpath(os.path.join(work_dir, "..", "dataR", "Robotran_Mazda_MX5_transmition_integrale.mbs")) 
mbs_data = Robotran.MbsData(mbs_file)

# =============================================================================
# 2. INITIALISATION DU USER MODEL
# =============================================================================
um = {}

# --- Simulation ---
um['simulation']                        = simulation

# --- Paramètres des Pneumatiques et Suspensions ---
um['FrontTire']                         = {'R': 0.288, 'K': 180000.0}
um['RearTire']                          = {'R': 0.288, 'K': 180000.0}
um['FrontSuspension']                   = {'K': 27000.0, 'C': 2200.0, 'C_bar': 15000.0, 'Z0': 0.43}
um['RearSuspension']                    = {'K': 27000.0, 'C': 1800.0, 'C_bar': 12000.0, 'Z0': 0.43}

# --- Systèmes de Contrôle ---
um['enable_esp']                        = True       # Active l'ESP (contrôle de stabilité)
um['enable_abs']                        = False      # Active l'ABS (antiblocage des roues)

# --- Paramètres de Direction ---
um['K_steering']                        = 200000.0   # Raideur direction avant (N·m/rad)
um['D_steering']                        = 10000.0    # Amortissement direction avant (N·m·s/rad)
um['K_steering_AR']                     = 1e7        # Raideur direction arrière (très élevée)
um['D_steering_AR']                     = 1e4        # Amortissement direction arrière
um['enable_4ws']                        = True       # Direction 4-roues (False = 2-roues)
um['ratio_4ws']                         = 0.10       # Ratio direction arrière vs avant (4WS)

# --- Paramètres de Couple et Freinage ---
um['torque_rear']                       = 0.0        # Couple moteur roues arrière (N·m)
um['torque_front']                      = 0.0        # Couple moteur roues avant (N·m)
um['force_freinage']                    = 0.0        # Force de freinage (N)
um['couple_acceleration']               = 2800.0     # Couple d'accélération réduit (N·m)

# --- Paramètres de Virage ---
um['amplitude_virage']                  = 0.008      # Déplacement de crémaillère en virage (m) - réduit pour limiter le roulis
um['couple_virage']                     = 0.0        # Couple pour maintenir la vitesse en virage (N·m)

# --- Paramètres d'Évitement (Pure Pursuit) ---
um['L_visee']                           = 4.0        # Distance de lookahead (m)
um['Kp_volant']                         = 0.01       # Gain proportionnel du volant
um['q_target_max']                      = 0.5        # Limite de braquage (rad)
um['X_debut_decalage']                  = 5.0        # Début obstacle
um['X_fin_decalage']                    = 15.0       # Fin obstacle
um['X_debut_retour']                    = 60.0       # Début retour
um['X_fin_retour']                      = 80.0       # Fin retour
um['Y_decalage_max']                    = 3.0        # Décalage latéral max (m)

# --- Paramètres de Temps (Freinage d'urgence) ---
um['tf1']                               = 1.0        # Début freinage d'urgence (s)
um['tf2']                               = 3.0        # Fin freinage d'urgence (s)

# --- Paramètres ESP (Contrôle de Stabilité) ---
um['K_esp_base']                        = 50000.0    # Gain de base ESP (N·m)
um['v_esp_ref']                         = 16.67      # Vitesse de référence ESP (m/s = 60 km/h)
um['K_esp_max']                         = 100000.0   # Gain ESP maximal saturé (N·m)
um['v_esp_min']                         = 2.7        # Vitesse minimale pour ESP (m/s)

# --- Paramètres ABS (Antiblocage des Roues) ---
um['abs_slip_threshold']                = 0.80       # Seuil d'activation ABS (80% vitesse)
um['abs_recovery_threshold']            = 0.95       # Seuil de récupération ABS (95% vitesse)
um['v_abs_min']                         = 2.0        # Vitesse minimale pour ABS (m/s)

# --- Freinage ESP/ABS ---
um['frein_esp_max']                     = -4000.0    # Force max des freins (N·m)
um['frein_ratio_front']                 = 0.60       # Répartition avant freinage (%)
um['frein_ratio_rear']                  = 0.40       # Répartition arrière freinage (%)

# --- Vitesses de Sécurité ---
um['v_min_started']                     = 5.0        # Vitesse pour confirmer départ (m/s)
um['v_max_stopped']                     = 2.0        # Vitesse pour confirmer arrêt (m/s)

mbs_data.user_model = um

# Configuration initiale (Hauteur pour garantir le contact pneu/sol)
mbs_data.q[3] = 0.2 

# =============================================================================
# 3. PARTITIONNEMENT
# =============================================================================
print("\n>> PARTITIONNEMENT...")
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=0)
mbs_part.run()

# Forcer l'alignement de la direction après partitionnement
try:
    jid_dir = mbs_data.joint_id["T2_barre_direction"]
    jid_dir_ar = mbs_data.joint_id["T2_barre_direction_AR"]
    mbs_data.q[jid_dir] = 0.0
    mbs_data.q[jid_dir_ar] = 0.0
    mbs_data.qd[jid_dir] = 0.0
    mbs_data.qd[jid_dir_ar] = 0.0
    mbs_data.qdd[jid_dir] = 0.0
    mbs_data.qdd[jid_dir_ar] = 0.0
except KeyError:
    pass

# =============================================================================
# 4. PHASE DE TASSEMENT (Remplace MbsEquil)
# =============================================================================
mbs_data.process = 2
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
# On utilise un pas de temps fin (1e-3) pour stabiliser le modèle Bakker
print(">> Phase de tassement (2 secondes)...")
mbs_dirdyn.set_options(dt0=1e-2, tf=2.0, save2file=0) 
mbs_dirdyn.run()
mbs_data.q[43] = mbs_data.qd[43] = mbs_data.qdd[43] = 0 # joint 43 quand on est en transmission intégrale (Joint de la barre de direction avant) sinon c'est le joint 39
mbs_data.q[48] = mbs_data.qd[48] = mbs_data.qdd[48] = 0 # joint 48 quand on est en transmission intégrale (Joint de la barre de direction arrière)

# =============================================================================
# 5. INJECTION DES VITESSES ET SIMULATION DYNAMIQUE
# =============================================================================
print(f">> Injection de la vitesse : {vitesse_kmh} km/h")
mbs_data.process = 3
vitesse_ms = vitesse_kmh / 3.6
omega = vitesse_ms / 0.288 # Vitesse angulaire

# Application des vitesses
mbs_data.qd[1] = vitesse_ms  # Châssis (vitesse longitudinale X)

mbs_data.q[2]  = 0.0  # Y
mbs_data.q[4]  = 0.0  # Roulis (Nouveau !)
mbs_data.q[6]  = 0.0  # Lacet (Yaw)
mbs_data.qd[2] = 0.0  
mbs_data.qd[4] = 0.0  # Vitesse de Roulis (Nouveau !)
mbs_data.qd[6] = 0.0

mbs_data.qd[29] = omega      # Roue AV_G (indices à vérifier selon votre .mbs)
mbs_data.qd[35] = omega      # Roue AV_D
mbs_data.qd[18] = omega      # Roue AR_G
mbs_data.qd[12] = omega      # Roue AR_D
# Application des vitesses
mbs_data.qd[2] = 0.0         # Tuer le glissement latéral parasite (Y)
mbs_data.qd[6] = 0.0         # Tuer la rotation parasite (Yaw)

print(f">> Lancement de la simulation ({simulation})...")
mbs_dirdyn.set_options(dt0=1e-3, tf=6.0, save2file=1)

# =============================================================================
# 5. GESTION DU CRASH-TEST (Anti-arrêt de Python)
# =============================================================================
try:
    mbs_dirdyn.run()
except Exception as e:
    # Si la voiture se retourne, on atterrit ici au lieu de faire planter Python
    print("\n" + "="*50)
    print("CRASH PHYSIQUE DÉTECTÉ")
    print("La voiture a perdu le contrôle (tonneau ou géométrie cassée).")
    print("Génération des graphiques avec les données de la boîte noire...")
    print("="*50 + "\n")

# =============================================================================
# 6. GÉNÉRATION DES GRAPHES (PDF)
# =============================================================================
print("\n>> Récupération des données...")
results_dir = os.path.normpath(os.path.join(work_dir, "..", "resultsR"))
results_path = os.path.join(results_dir, "dirdyn_q.res")

try:
    results = np.loadtxt(results_path)
    time = results[:, 0]

    # --- Récupération dynamique des bonnes colonnes ---
    id_av_g = mbs_data.joint_id["R1_Bras_sup_AV_G"] 
    id_ar_g = mbs_data.joint_id["R1_Bras_inf_AR_G"] 

    q_av_g = results[:, id_av_g] * (180 / np.pi)
    q_ar_g = results[:, id_ar_g] * (180 / np.pi)

    fig, axs = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    fig.suptitle(f'Réponse des Suspensions (CRASH) - {simulation}', fontsize=12, fontweight='bold', color='red')

    # Avant Gauche
    axs[0].plot(time, q_av_g, color='darkblue', linewidth=1.5, label='Avant Gauche')
    axs[0].set_ylabel('Angle (°)')
    axs[0].grid(True, linestyle='--', alpha=0.7)
    axs[0].legend(loc='best')

    # Arrière Gauche
    axs[1].plot(time, q_ar_g, color='red', linewidth=1.5, label='Arrière Gauche')
    axs[1].set_ylabel('Angle (°)')
    axs[1].set_xlabel('Temps (s)')
    axs[1].grid(True, linestyle='--', alpha=0.7)
    axs[1].legend(loc='best')

    plt.tight_layout()

    plot_name = f"suspension_{simulation}.pdf"
    plot_save_path = os.path.join(results_dir, plot_name)
    
    plt.savefig(plot_save_path, format="pdf", bbox_inches='tight')
    print(f">> Graphique sauvegardé : {plot_save_path}")
#    plt.show()

except Exception as e:
    print(f"Impossible de lire le fichier de résultats (Fichier corrompu par le crash) : {e}")
# =============================================================================
# 7. GRAPHIQUE 2 : LES FORCES DE CONTACT PNEU/SOL (ABS en action)
# =============================================================================
print("\n>> Génération du graphique des Forces de Contact...")

# On cherche l'ID exact de ta roue arrière gauche (AR_G)
try:
    # ⚠️ Vérifie que le nom de ton capteur externe correspond bien à celui dans MBsysPad
    id_ext_ar_g = mbs_data.extforce_id["ExtForce_Roue_AR_G"] # Modifie le nom si besoin !
except KeyError:
    # Si le nom n'est pas trouvé, on suppose souvent que la roue AR_G est le capteur n°3
    id_ext_ar_g = 3 

nom_output_force = f"dirdyn_F_Longi_Roue_{id_ext_ar_g}.res"
results_path_force = os.path.join(results_dir, nom_output_force)

try:
    if os.path.exists(results_path_force):
        results_force = np.loadtxt(results_path_force)
        time_force = results_force[:, 0]
        force_valeur = results_force[:, 1] # La valeur de la force Fx

        fig_force, ax_force = plt.subplots(figsize=(8, 4))
        fig_force.suptitle(f'Force Longitudinale du Pneu (Friction) - {simulation}', fontsize=12, fontweight='bold')

        ax_force.plot(time_force, force_valeur, color='purple', linewidth=2.0, label='Force de freinage (N)')
        
        ax_force.set_ylabel('Force (N)')
        ax_force.set_xlabel('Temps (s)')
        ax_force.grid(True, linestyle='--', alpha=0.7)
        ax_force.legend(loc='best')

        plt.tight_layout()
        plot_save_path_force = os.path.join(results_dir, f"forces_contact_{simulation}.pdf")
        plt.savefig(plot_save_path_force, format="pdf", bbox_inches='tight')
        print(f">> Graphique des forces sauvegardé : {plot_save_path_force}")
    else:
        print(f"⚠️ Le fichier {nom_output_force} n'a pas été trouvé. As-tu bien mis le set_output dans user_ExtForces.py ?")

except Exception as e:
    print(f"Impossible de générer le graphique des forces : {e}")

# =============================================================================
# 8. GRAPHIQUE 3 : DYNAMIQUE DES PNEUS ET ABS (Vitesses et Glissement)
# =============================================================================
print("\n>> Génération du graphique ABS (Glissement Pneu)...")
results_path_qd = os.path.join(results_dir, "dirdyn_qd.res")

try:
    results_qd = np.loadtxt(results_path_qd)
    time_qd = results_qd[:, 0]

    # Récupération des identifiants (Vérifie les noms selon ton modèle)
    id_T1_chassis = mbs_data.joint_id["T1_chassis"]
    id_roue_ar_g = mbs_data.joint_id["R2_Roue_AR_G"]
    
    # Extraction des vitesses
    v_vehicule = results_qd[:, id_T1_chassis]
    omega_roue = results_qd[:, id_roue_ar_g]
    
    # Conversion de la rotation (rad/s) en vitesse linéaire (m/s)
    R_wheel = 0.288
    v_roue_tangentielle = np.abs(omega_roue * R_wheel) 

    # Création du graphique
    fig_abs, ax_abs = plt.subplots(figsize=(8, 5))
    fig_abs.suptitle(f'Action de l\'ABS (Analyse du Glissement) - {simulation}', fontsize=12, fontweight='bold')

    # La Courbe Noire (Vitesse de la voiture)
    ax_abs.plot(time_qd, v_vehicule, color='black', linewidth=2.0, label='Vitesse Châssis (m/s)')
    
    # La Courbe Orange (Vitesse de la roue)
    ax_abs.plot(time_qd, v_roue_tangentielle, color='orange', linewidth=1.5, label='Vitesse Roue (m/s)')
    
    # Le Remplissage Rouge (Zone de glissement)
    ax_abs.fill_between(time_qd, v_vehicule, v_roue_tangentielle, 
                        where=(v_vehicule > v_roue_tangentielle), 
                        color='red', alpha=0.3, label='Glissement du pneu')

    ax_abs.set_ylabel('Vitesse (m/s)')
    ax_abs.set_xlabel('Temps (s)')
    ax_abs.grid(True, linestyle='--', alpha=0.7)
    ax_abs.legend(loc='upper right')

    plt.tight_layout()
    plot_save_path_abs = os.path.join(results_dir, f"abs_glissement_{simulation}.pdf")
    plt.savefig(plot_save_path_abs, format="pdf", bbox_inches='tight')
    print(f">> Graphique ABS sauvegardé : {plot_save_path_abs}")
    
#    plt.show() # Ceci doit TOUJOURS rester la toute dernière ligne de ton main.py !

except Exception as e:
    print(f"Impossible de générer le graphique ABS : {e}")
print("\n--- Simulation terminée ---")