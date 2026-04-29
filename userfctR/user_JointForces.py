# -*- coding: utf-8 -*-
import numpy as np

def user_JointForces(mbs_data, tsim):
    
    # TABLEAU DE BORD
    enable_esp = mbs_data.user_model['enable_esp'] 
    enable_abs = mbs_data.user_model['enable_abs']   

    mbs_data.Qq[:] = 0.0
    um = mbs_data.user_model
    mode = um.get('simulation', 'MRU')
    jid_X = mbs_data.joint_id["T1_chassis"]
    v_veh = mbs_data.qd[jid_X]
    
    jid_dir = mbs_data.joint_id["T2_barre_direction"]
    jid_dir_ar = mbs_data.joint_id["T2_barre_direction_AR"]
    
    wheels_rear = [mbs_data.joint_id["R2_Roue_AR_G"], mbs_data.joint_id["R2_Roue_AR_D"]]
    try:
        wheels_front = [mbs_data.joint_id["R2_Roue_AV_G"], mbs_data.joint_id["R2_Roue_AV_D"]]
    except KeyError:
        wheels_front = []

    q_target = 0.0
    torque_rear = 0.0
    torque_front = 0.0
    force_freinage = -200.0 

    
    # =========================================================
    #  MÉMOIRE DU PILOTE
    # =========================================================
    # Si la variable n'existe pas encore, on la crée à False
    if 'has_rolled' not in um:
        um['has_rolled'] = False  
    if 'is_stopped' not in um:
        um['is_stopped'] = False  

    # Étape 1 : On confirme que la vraie simulation a commencé
    if v_veh > 5.0:
        um['has_rolled'] = True

    # Étape 2 : Si la voiture est passée sous les 2 m/s, on coupe tout 
    if um['has_rolled'] and v_veh < 2.0:
        um['is_stopped'] = True
        
        # LA SOLUTION : On force Robotran à arrêter la simulation proprement
        mbs_data.flag_stop = 1 
        
        # Petit message pour prévenir dans la console
        if 'stop_msg_printed' not in um:
            print(f"\n Véhicule arrêté à t = {tsim:.2f}s. Fin de la simulation ordonnée. Has_rolled = {um['has_rolled']}, Is_stopped = {um['is_stopped']}")
            um['stop_msg_printed'] = True

    # =========================================================
    #  CAPTEUR DE BLOCAGE DE ROUES (Pour ton diagnostic)
    # =========================================================
    if 'warning_printed' not in um:
        um['warning_printed'] = False

    # On écoute la roue arrière gauche
    jid_wheel_ar_g = mbs_data.joint_id["R2_Roue_AR_G"]
    omega_roue = mbs_data.qd[jid_wheel_ar_g]
    
    # Règle : Si la voiture roule à plus de 5 m/s MAIS que la roue 
    # tourne à moins de 2 rad/s (elle est quasi figée), on glisse !
    if v_veh > 5.0 and abs(omega_roue) < 2.0:
        if not um['warning_printed']:
            print(f"\n⚠ DANGER : Blocage des roues arrière détecté à t = {tsim:.2f}s !")
            um['warning_printed'] = True


    # =========================================================
    # SCÉNARIO : VIRAGE 
    # =========================================================
    if mode == "virage":
        if 2.0 <= tsim < 4.0:
            q_target = 0.02 * np.sin(np.pi * (tsim - 2.0) / 2.0)
        elif 4.0 <= tsim < 6.0:
            q_target = -0.02 * np.sin(np.pi * (tsim - 4.0) / 2.0)

    # =========================================================
    # SCÉNARIO : ÉVITEMENT (LOGIQUE SPATIALE / "VRAI PILOTE")
    # =========================================================
    if mode == "evitement":
        # 1. On récupère la position et l'angle du châssis
        jid_X = mbs_data.joint_id["T1_chassis"] # Distance parcourue (X)
        jid_Y = mbs_data.joint_id["T2_chassis"] # Décalage latéral (Y)
        jid_Yaw = mbs_data.joint_id["R3_chassis"] # Angle de lacet (Yaw)
        
        X_actuel = mbs_data.q[jid_X]
        Y_actuel = mbs_data.q[jid_Y]
        Yaw_actuel = mbs_data.q[jid_Yaw]

        # 2. On définit le tracé de la chicane en MÈTRES sur la route
        # (Peu importe si on va vite ou si on freine, la chicane ne bouge pas)
        X_debut_decalage = 10.0  # La voiture commence à tourner à 30m
        X_fin_decalage   = 40.0  # Elle doit être sur la voie de gauche à 50m
        X_debut_retour   = 80.0  # Elle commence à revenir à 90m
        X_fin_retour     = 110.0 # Elle est revenue à droite à 120m
        
        Y_cible = 0.0
        if X_actuel < X_debut_decalage:
            Y_cible = 0.0
        elif X_debut_decalage <= X_actuel < X_fin_decalage:
            progression = (X_actuel - X_debut_decalage) / (X_fin_decalage - X_debut_decalage)
            Y_cible = 3.0 * (0.5 - 0.5 * np.cos(np.pi * progression))
        elif X_fin_decalage <= X_actuel < X_debut_retour:
            Y_cible = 3.0  
        elif X_debut_retour <= X_actuel < X_fin_retour:
            progression = (X_actuel - X_debut_retour) / (X_fin_retour - X_debut_retour)
            Y_cible = 3.0 * (0.5 + 0.5 * np.cos(np.pi * progression))
        else:
            Y_cible = 0.0

        # 3. Calcul de l'erreur pour le contrôleur PD
        erreur_Y = Y_cible - Y_actuel
        erreur_Yaw = 0.0 - Yaw_actuel # On veut que la voiture reste bien droite (Yaw=0)
        
        # Gains du pilote (ajustés pour le 4WS)
        Kp_Y = 0.012  
        Kd_Yaw = 0.1 if enable_esp else 0.0
        
        # Calcul de la commande de direction finale
        raw_q_target = (Kp_Y * erreur_Y) + (Kd_Yaw * erreur_Yaw)
        q_target = np.clip(raw_q_target, -0.025, 0.025)

        # --- GESTION DES FREINS PENDANT L'ÉVITEMENT ---
        # On freine entre 2.0s et 4.0s (on garde le temps ici car c'est une action de pied)
        if 2.0 <= tsim <= 4.0 and not um['is_stopped']:
            torque_front = force_freinage * 0.50
            torque_rear  = force_freinage * 0.50

    # =========================================================
    # SCÉNARIO : ACCELERATION OU FREINAGE  
    # =========================================================
    if mode == "acceleration" and tsim > 1.0:
        # RÉDUCTION DU COUPLE (Anti-Burnout) : 150 Nm au lieu de 600 Nm
        torque_rear = 150.0  
        
    elif mode == "freinage" and tsim > 1.0:
        # Même logique de sécurité que pour l'évitement
        if not um['is_stopped']: 
            torque_front = force_freinage * 0.60
            torque_rear  = force_freinage * 0.40

    # =========================================================
    # 1. APPLICATION DU CONTRÔLE DE DIRECTION (AV + AR)
    # =========================================================
    K_steering, D_steering = 1e6, 1e4
    
    # --- Direction AVANT ---
    mbs_data.Qq[jid_dir] = -K_steering * (mbs_data.q[jid_dir] - q_target) - D_steering * mbs_data.qd[jid_dir]

    # --- Direction ARRIÈRE (4WS Activé !) ---
    # Règle des 10% : L'arrière braque dans le MÊME SENS que l'avant 
    # (Idéal pour la stabilité à haute vitesse lors d'un évitement)
    if mode == "evitement":
        q_target_AR = 0.10 * q_target 
    else:
        q_target_AR = 0.0 # On bloque l'arrière pour les autres modes (accélération/freinage)
        
    mbs_data.Qq[jid_dir_ar] = -K_steering * (mbs_data.q[jid_dir_ar] - q_target_AR) - D_steering * mbs_data.qd[jid_dir_ar]
    
    # =========================================================
    # 2. APPLICATION DES COUPLES & SYSTÈME ABS à hystérésis
    # =========================================================

    if 'abs_state' not in um:
        # Mémoire individuelle pour chaque roue (False = Frein normal, True = ABS relâche)
        um['abs_state'] = {}
        
    R_wheel = 0.288
    all_wheels = wheels_rear + wheels_front
    
    for jid in all_wheels:
        # On initialise la mémoire de la roue si elle n'existe pas
        if jid not in um['abs_state']:
            um['abs_state'][jid] = False 
            
        # On détermine si on regarde une roue arrière ou avant
        couple_base = torque_rear if jid in wheels_rear else torque_front
        
        # Logique ABS
        if enable_abs and couple_base < 0 and um['has_rolled'] and v_veh > 2.0:
            omega = mbs_data.qd[jid]
            v_ideale = v_veh / R_wheel
            
            # --- L'HYSTÉRÉSIS (Le secret anti-tremblement) ---
            if not um['abs_state'][jid] and abs(omega) < (v_ideale * 0.80):
                um['abs_state'][jid] = True  # La roue bloque -> On coupe les freins
            elif um['abs_state'][jid] and abs(omega) > (v_ideale * 0.95):
                um['abs_state'][jid] = False # La roue a repris sa vitesse -> On refreine
            # --------------------------------------------------
            
            # Application du couple selon l'état de l'ABS
            mbs_data.Qq[jid] = 0.0 if um['abs_state'][jid] else couple_base
        else:
            # Freinage normal (ou accélération) si l'ABS n'est pas nécessaire
            mbs_data.Qq[jid] = couple_base
    
    # === BLOC DE DEBUG ===
    if tsim % 0.5 < 0.001: # On affiche toutes les 0.5s pour ne pas spammer
        print(f"DEBUG: Position barre AV = {mbs_data.q[jid_dir]:.4f} | Cible AV = {q_target:.4f}")
        print(f"DEBUG: Position barre AR = {mbs_data.q[jid_dir_ar]:.4f} | Cible AR = {q_target_AR:.4f}")
    # =====================
    
    return mbs_data.Qq