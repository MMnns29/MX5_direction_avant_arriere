# -*- coding : utf-8 -*-

import numpy as np

def user_JointForces(mbs_data, tsim):
    """
    This function defines the user function for the joint forces. It is called by the main program at each time step.

    Parameters
    ----------
    mbs_data : object
        The multibody system data.
    tsim : float
        The current simulation time.

    Returns
    -------
    None
        The function does not return anything, but it modifies the mbs_data object to include the joint forces.

    """
    # Get the simulation type from the user model
    um = mbs_data.user_model
    simulation_type = um['simulation']
    enable_4ws = um['enable_4ws']
    ratio_4ws = um['ratio_4ws']
    enable_esp = um['enable_esp']
    enable_abs = um['enable_abs']

    # Get the joint id's
    jid_dir_av = mbs_data.joint_id['T2_barre_direction']
    jid_dir_ar = mbs_data.joint_id['T2_barre_direction_AR']

    jid_wheels_rear = [mbs_data.joint_id['R2_Roue_AR_G'], mbs_data.joint_id['R2_Roue_AR_D']]
    jid_wheels_front = [mbs_data.joint_id['R2_Roue_AV_G'], mbs_data.joint_id['R2_Roue_AV_D']]

    # Get the vehicule speed
    jid_X = mbs_data.joint_id["T1_chassis"]
    v_veh = mbs_data.qd[jid_X]

    # Get the K and D of the direction control
    K_steering_AV = um['K_steering']
    D_steering_AV = um['D_steering']
    K_steering_AR = um['K_steering_AR']
    D_steering_AR = um['D_steering_AR']

    # 
    torque_rear = 0.0   # Couple moteur roues arrière (N·m)
    torque_front = 0.0  # Couple moteur roues avant (N·m)



    # =============================================================================================
    # ------------------------------------- Memoire du pilote -------------------------------------
    # =============================================================================================

    # Visée du pilote 
    q_target = 0.0  # Angle de direction cible (en radians)



    # =============================================================================================
    # ---------------------------------- Verification de l'etat -----------------------------------
    # =============================================================================================
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



    # =============================================================================================
    # ------------------------------------ Simulation : Virage ------------------------------------
    # =============================================================================================
    if simulation_type == 'virage':
        # Recuperation des donnée de simulation
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

    # =============================================================================================
    # --------------------------------- Simulation : Acceleration ---------------------------------
    # =============================================================================================
    if simulation_type == 'acceleration':
        # Recuperation des donnée de simulation
        if tsim > 1.0 and mbs_data.process != 2:
            torque_rear = um['couple_acceleration']


    # =============================================================================================
    # --------------------------- Application du contrôle de direction ----------------------------
    # =============================================================================================

    # --- DIRECTION AVANT (Inchangée) ---
    mbs_data.Qq[jid_dir_av] = -K_steering_AV * (mbs_data.q[jid_dir_av] - q_target) - D_steering_AV * mbs_data.qd[jid_dir_av]

    if um['enable_4ws']:
        q_target_AR = ratio_4ws * q_target 
    else:
        q_target_AR = 0.0  # La direction arrière reste centrée, elle ne suit pas la direction avant 

    mbs_data.Qq[jid_dir_ar] = -K_steering_AR * (mbs_data.q[jid_dir_ar] - q_target_AR) - D_steering_AR * mbs_data.qd[jid_dir_ar]



    # =============================================================================================
    # ----------------------- Application des couples + système ESP et ABS ------------------------
    # =============================================================================================

    R_wheel = um['FrontTire']['R']
    jid_wheels_all = jid_wheels_rear + jid_wheels_front

    # Application des couples
    for jid in jid_wheels_all:
        couple_base = torque_rear if jid in jid_wheels_rear else torque_front

        if enable_esp and jid in jid_wheels_rear:
            pass
        if enable_abs and couple_base < 0:
            pass
        else:
            mbs_data.Qq[jid] = couple_base

    # =============================================================================================
    # ------------------------------------------- Debug -------------------------------------------
    # =============================================================================================
    if tsim > 0.6 and tsim % 0.2 < 0.001 and mbs_data.process != 2:
        
        # 1. Lecture des crémaillères (Conversion en millimètres pour que ça soit lisible)
        crem_AV = mbs_data.q[jid_dir_av] * 1000.0
        crem_AR = mbs_data.q[jid_dir_ar] * 1000.0
        
        # 2. Approximation de l'angle global (Bras de levier de 60 mm de la MX-5)
        # Formule : Angle = arc sinus(déplacement / bras de levier)
        angle_AV_moyen = np.degrees(np.arcsin(mbs_data.q[jid_dir_av] / 0.06))
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
        print (f"Position (m)       : X = {mbs_data.q[jid_X]:.2f}"
               f" | Vitesse = {v_veh:.2f} m/s")
        print("-" * 35)