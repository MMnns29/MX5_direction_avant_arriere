# -*- coding: utf-8 -*-
import numpy as np

def user_JointForces(mbs_data, tsim):
    
    # TABLEAU DE BORD : Donnée du vehicule
    enable_esp = mbs_data.user_model['enable_esp'] 
    enable_abs = mbs_data.user_model['enable_abs']   

    mbs_data.Qq[:] = 0.0
    um = mbs_data.user_model
    mode = um.get('simulation', 'MRU')
    jid_X = mbs_data.joint_id["T1_chassis"]
    jid_Y = mbs_data.joint_id["T2_chassis"]
    jid_Yaw = mbs_data.joint_id["R3_chassis"]
    v_veh = mbs_data.qd[jid_X]
    
    jid_dir = mbs_data.joint_id["T2_barre_direction"]
    jid_dir_ar = mbs_data.joint_id["T2_barre_direction_AR"]
    
    wheels_rear = [mbs_data.joint_id["R2_Roue_AR_G"], mbs_data.joint_id["R2_Roue_AR_D"]]
    try:
        wheels_front = [mbs_data.joint_id["R2_Roue_AV_G"], mbs_data.joint_id["R2_Roue_AV_D"]]
    except KeyError:
        wheels_front = []

    # Récupération des paramètres depuis um
    q_target = 0.0
    torque_rear = um['torque_rear']
    torque_front = um['torque_front']
    force_freinage = um['force_freinage']
    tf1 = um['tf1']
    tf2 = um['tf2']
    K_steering = um['K_steering']
    D_steering = um['D_steering']
    K_steering_AR = um['K_steering_AR']
    D_steering_AR = um['D_steering_AR']
    ratio_4ws = um['ratio_4ws']
    
    # =========================================================
    #  MÉMOIRE DU PILOTE
    # =========================================================
    # Si la variable n'existe pas encore, on la crée à False
    if 'has_rolled' not in um:
        um['has_rolled'] = False  
    if 'is_stopped' not in um:
        um['is_stopped'] = False  

    # Étape 1 : On confirme que la vraie simulation a commencé
    if v_veh > um['v_min_started']:
        um['has_rolled'] = True

    # Étape 2 : Si la voiture est passée sous les X m/s, on coupe tout 
    if um['has_rolled'] and v_veh < um['v_max_stopped']:
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
    
    # Règle : Si la voiture roule à plus de X m/s MAIS que la roue 
    # tourne à moins de 2 rad/s (elle est quasi figée), on glisse !
    if v_veh > um['v_min_started'] and abs(omega_roue) < 2.0:
        if not um['warning_printed']:
            print(f"\n⚠ DANGER : Blocage des roues arrière détecté à t = {tsim:.2f}s !")
            um['warning_printed'] = True


    # =========================================================
    # SCÉNARIO : VIRAGE 
    # =========================================================
    if mode == "virage":
        amplitude = um['amplitude_virage']
        if 1.0 <= tsim < 3.0:
            phase = (tsim - 1.0) / 3.0
            # Virage gauche continu
            q_target = amplitude * (1.0 - np.cos(np.pi * phase)) / 2.0
        elif 3.0 <= tsim < 3.5:
            # Transition douce vers la ligne droite
            phase = (tsim - 3.0) / 0.5
            q_target = amplitude * (1.0 - phase)
        else:
            q_target = 0.0

        # Maintenir la vitesse en virage
        torque_rear = um['couple_virage']

    # =========================================================
    # SCÉNARIO : ÉVITEMENT (PILOTE "LOOK-AHEAD" / PURE PURSUIT)
    # =========================================================
    if mode == "evitement":
        # Récupération des positions et angles du châssis
        X_actuel = mbs_data.q[jid_X]
        Y_actuel = mbs_data.q[jid_Y]
        Yaw_actuel = mbs_data.q[jid_Yaw]

        # 2. Chicane (récupération depuis um)
        X_debut_decalage = um['X_debut_decalage']
        X_fin_decalage = um['X_fin_decalage']
        X_debut_retour = um['X_debut_retour']
        X_fin_retour = um['X_fin_retour']
        Y_decalage_max = um['Y_decalage_max']
        
        # -----------------------------------------------------
        # LE CERVEAU DU PILOTE (L'anticipation Visuelle)
        # -----------------------------------------------------
        L_visee = um['L_visee']  # Le pilote regarde X mètres devant la voiture
        X_visee = X_actuel + L_visee

        # A. Où sera la ligne blanche cible dans L_visee mètres ?
        if X_visee < X_debut_decalage:
            Y_cible_visee = 0.0
        elif X_debut_decalage <= X_visee < X_fin_decalage:
            progression = (X_visee - X_debut_decalage) / (X_fin_decalage - X_debut_decalage)
            Y_cible_visee = Y_decalage_max * (0.5 - 0.5 * np.cos(np.pi * progression))
        elif X_fin_decalage <= X_visee < X_debut_retour:
            Y_cible_visee = Y_decalage_max  
        elif X_debut_retour <= X_visee < X_fin_retour:
            progression = (X_visee - X_debut_retour) / (X_fin_retour - X_debut_retour)
            Y_cible_visee = Y_decalage_max * (0.5 + 0.5 * np.cos(np.pi * progression))
        else:
            Y_cible_visee = 0.0

        # B. Où sera la voiture dans L_visee mètres si elle garde son angle de braquage actuel ?
        # (Formule géométrique simple : Y_actuel + Distance * Angle_Lacet)
        Y_futur_voiture = Y_actuel + (L_visee * Yaw_actuel)

        # C. Le coup de volant est proportionnel à l'écart entre ces deux futurs
        erreur_visee = Y_cible_visee - Y_futur_voiture
        
        Kp_volant = um['Kp_volant']  # Force du coup de volant
        raw_q_target = Kp_volant * erreur_visee # raw_q_target est la consigne de direction brute, sans limitation
        q_target = np.clip(raw_q_target, -um['q_target_max'], um['q_target_max'])  # Butée de sécurité, on ne veut pas que le pilote braque à fond d'un coup !

        # 3. ON SAUVEGARDE L'ERREUR DE LACET POUR L'ESP (Lui, il veut empêcher de tourner sur soi-même !)
        um['erreur_Yaw'] = 0.0 - Yaw_actuel 

        # --- GESTION DE LA PÉDALE DE FREIN (Évitement d'urgence) ---
        if tf1 <= tsim <= tf2 and not um['is_stopped']:
            torque_front = force_freinage * um['frein_ratio_front']
            torque_rear  = force_freinage * um['frein_ratio_rear']  
    
    # =========================================================
    # SCÉNARIO : ACCELERATION OU FREINAGE  
    # =========================================================
    if mode == "acceleration" and tsim > 1.0 and mbs_data.process != 2:  # On attend 1 seconde pour être sûr que la simulation a démarré, et on vérifie que le processus n'est pas déjà arrêté (sécurité anti-burnout)
        # RÉDUCTION DU COUPLE (Anti-Burnout)
        torque_rear = um['couple_acceleration']
        
    elif mode == "freinage" and tsim > 1.0:
        # Même logique de sécurité que pour l'évitement
        if not um['is_stopped']: 
            torque_front = force_freinage * um['frein_ratio_front']
            torque_rear  = force_freinage * um['frein_ratio_rear']

    # =========================================================
    # 1. APPLICATION DU CONTRÔLE DE DIRECTION (AV + AR)
    # =========================================================
    # Muscles humains / Direction assistée
    
    # --- DIRECTION AVANT (Inchangée) ---
    mbs_data.Qq[jid_dir] = -K_steering * (mbs_data.q[jid_dir] - q_target) - D_steering * mbs_data.qd[jid_dir]

    # --- DIRECTION ARRIÈRE (Le point de soudure en titane) ---
    # On met un K gigantesque (1e7 = 10 millions) pour être sûr que la route 
    # ne puisse JAMAIS arracher la direction arrière.
    
    if um['enable_4ws']:
        q_target_AR = ratio_4ws * q_target 
    else:
        q_target_AR = 0.0  # La direction arrière reste centrée, elle ne suit pas la direction avant 

    mbs_data.Qq[jid_dir_ar] = -K_steering_AR * (mbs_data.q[jid_dir_ar] - q_target_AR) - D_steering_AR * mbs_data.qd[jid_dir_ar]
    
    # =========================================================
    # 2. APPLICATION DES COUPLES, VRAI ESP & SYSTÈME ABS
    # =========================================================

    if 'abs_state' not in um:
        um['abs_state'] = {}
        
    R_wheel = 0.288
    all_wheels = wheels_rear + wheels_front
    
    # --- CALCUL DU CORRECTIF ESP DYNAMIQUE (Gain Scheduling) ---
    delta_esp = 0.0
    if enable_esp and 'erreur_Yaw' in um:
        
        # Point de référence
        v_ref = um['v_esp_ref']
        K_base = um['K_esp_base']
        
        # Sécurité basse vitesse
        if v_veh < um['v_esp_min']:
            K_esp = 0.0
        else:
            # Évolution au carré de la vitesse
            K_esp = K_base * ((v_veh / v_ref) ** 2)
            K_esp = min(K_esp, um['K_esp_max'])  # Saturation
            
        delta_esp = K_esp * um['erreur_Yaw']

    # Variables pour le radar de debug
    frein_applique_G = 0.0
    frein_applique_D = 0.0
    
    for jid in all_wheels:
        if jid not in um['abs_state']:
            um['abs_state'][jid] = False 
            
        couple_base = torque_rear if jid in wheels_rear else torque_front
        
        # --- APPLICATION DE L'ESP (Freinage asymétrique sur l'arrière) ---
        if enable_esp and jid in wheels_rear:
            if jid == mbs_data.joint_id["R2_Roue_AR_G"]:
                couple_base -= delta_esp # Ajoute ou retire du frein à gauche
            elif jid == mbs_data.joint_id["R2_Roue_AR_D"]:
                couple_base += delta_esp # Inversement à droite
                
            # Sécurité 1 : L'ESP ne peut QUE freiner, il ne peut pas accélérer
            couple_base = min(0.0, couple_base)
            
            # Sécurité 2 : L'ESP ne peut pas dépasser la force maximale des freins
            couple_base = max(um['frein_esp_max'], couple_base)

            # Mémorisation pour le radar
            if jid == mbs_data.joint_id["R2_Roue_AR_G"]: frein_applique_G = couple_base
            if jid == mbs_data.joint_id["R2_Roue_AR_D"]: frein_applique_D = couple_base
        
        # --- LOGIQUE ABS (Le boss final qui a le dernier mot) ---
        if enable_abs and couple_base < 0 and um['has_rolled'] and v_veh > um['v_abs_min']:
            omega = mbs_data.qd[jid]
            v_ideale = v_veh / R_wheel
            
            if not um['abs_state'][jid] and abs(omega) < (v_ideale * um['abs_slip_threshold']):
                um['abs_state'][jid] = True  
            elif um['abs_state'][jid] and abs(omega) > (v_ideale * um['abs_recovery_threshold']):
                um['abs_state'][jid] = False 
            
            # L'ABS coupe la pression de freinage (même celle demandée par l'ESP) si ça glisse
            mbs_data.Qq[jid] = 0.0 if um['abs_state'][jid] else couple_base
        else:
            mbs_data.Qq[jid] = couple_base
            
    # === TRACEUR RADAR ESP ===
    if enable_esp and tsim > 1.0 and tsim % 0.1 < 0.001 and 'erreur_yaw' in um and abs(um['erreur_Yaw']) > 0.01:
        print(f"t={tsim:.1f}s | Dérapage: {np.degrees(um['erreur_Yaw']):.1f}° | Frein AR_G: {frein_applique_G:.0f} Nm | Frein AR_D: {frein_applique_D:.0f} Nm")
    # =========================================================
    # === DEBUG : VÉRIFICATION DE LA DIRECTION (4WS & ACKERMAN) ===
    # =========================================================
    # On affiche les valeurs toutes les 0.2 secondes
    if tsim > 0.6 and tsim % 0.2 < 0.001 and mbs_data.process != 2:
        
        # 1. Lecture des crémaillères (Conversion en millimètres pour que ça soit lisible)
        crem_AV = mbs_data.q[jid_dir] * 1000.0
        crem_AR = mbs_data.q[jid_dir_ar] * 1000.0
        
        # 2. Approximation de l'angle global (Bras de levier de 60 mm de la MX-5)
        # Formule : Angle = arc sinus(déplacement / bras de levier)
        angle_AV_moyen = np.degrees(np.arcsin(mbs_data.q[jid_dir] / 0.06))
        angle_AR_moyen = np.degrees(np.arcsin(mbs_data.q[jid_dir_ar] / 0.06))
        
        # 3. Calcul du ratio réel appliqué par le contrôleur
        ratio_crem = (crem_AR / crem_AV) * 100 if abs(crem_AV) > 0.1 else 0.0

        print(f"\n--- t = {tsim:.2f} s ---")
        print(f"Crémaillères (mm)  : AV = {crem_AV:+.2f} | AR = {crem_AR:+.2f}  (Ratio: {ratio_crem:+.1f}%)")
        print(f"Angle Moyen (deg)  : AV = {angle_AV_moyen:+.2f}° | AR = {angle_AR_moyen:+.2f}°")

        # 4. VÉRIFICATION GAUCHE/DROITE (GÉOMÉTRIE D'ACKERMAN)
        # Robotran calcule la vraie géométrie 3D des biellettes. 
        # Remplacer les noms entre guillemets par ceux de tes vrais pivots si Robotran râle.
        try:
            jid_pivot_av_g = mbs_data.joint_id["R3_fusee_AV_G"] 
            jid_pivot_av_d = mbs_data.joint_id["R3_fusee_AV_D"] 
            
            angle_G = np.degrees(mbs_data.q[jid_pivot_av_g])
            angle_D = np.degrees(mbs_data.q[jid_pivot_av_d])
            
            print(f"Ackerman AV (deg)  : Roue Gauche = {angle_G:+.2f}° | Roue Droite = {angle_D:+.2f}°")
            
            # Dans un virage, la roue intérieure doit braquer PLUS FORT que la roue extérieure
            ecart_ackerman = abs(angle_G) - abs(angle_D)
            print(f"Différence (G-D)   : {ecart_ackerman:+.2f}°")
        except KeyError:
            print("Info: Noms des joints de pivot (R3) introuvables pour l'Ackerman.")

        # position du vehicule pour le debug
        print (f"Position (m)       : X = {mbs_data.q[jid_X]:.2f} | Y = {mbs_data.q[jid_Y]:.2f} | Yaw = {np.degrees(mbs_data.q[jid_Yaw]):.1f}°"
               f" | Vitesse = {v_veh:.2f} m/s")
        print("-" * 35)
        

    return mbs_data.Qq