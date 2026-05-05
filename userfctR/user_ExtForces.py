# -*- coding: utf-8 -*-
import numpy as np
from tgc_py import tgc_car_kine_wheel, tgc_bakker_contact
from MBsysPy import matrix_vector_product

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):
    """
    Cette fonction calcule les forces et moments extérieurs appliqués aux roues du véhicule en fonction de la cinématique du contact sol-roue et des paramètres du pneu.
    Les étapes principales sont les suivantes :
    PxF : Position du point d'application de la force (en coordonnées globales)
    RxF : Orientation du point d'application de la force (matrice de rotation)
    VxF : Vitesse du point d'application de la force
    OMxF : Vitesse angulaire du point d'application de la force
    AxF : Accélération du point d'application de la force
    OMPxF : Accélération angulaire du point d'application de la force
    mbs_data : Structure de données de Robotran contenant les paramètres du modèle et les variables d'état
    tsim : Temps de simulation actuel
    ixF : Indice de la force extérieure (correspondant à une roue spécifique)
    """

    Fx = Fy = Fz = Mx = My = Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]   
    dxF = mbs_data.dpt[1:, idpt]
    dxF_tgc = np.zeros(4)  # Initialisation du vecteur de déplacement pour tgc_car_kine_wheel
    dxF_tgc[1:4] = dxF  # Copie des composantes de déplacement dans le format attendu par tgc_car_kine_wheel

    wheel_names = ['F_pneu_av_g', 'F_pneu_av_d', 'F_pneu_ar_g', 'F_pneu_ar_d']
    wheel_ids = [mbs_data.extforce_id.get(n) for n in wheel_names]

    if ixF in wheel_ids:
        # 1. PARAMÈTRES PNEUS (Récupérés du user_model)
        if ixF == wheel_ids[0] or ixF == wheel_ids[1]:
            K_tire = mbs_data.user_model['FrontTire']['K']
            R_tire = mbs_data.user_model['FrontTire']['R']
        else:
            K_tire = mbs_data.user_model['RearTire']['K']
            R_tire = mbs_data.user_model['RearTire']['R']

        # 2. CINÉMATIQUE (Calcul du contact sol plat)
        pen, rz, angslip, angcamb, slip, Pct, Vmct, Rt_ground, dxF_tgc = \
            tgc_car_kine_wheel(PxF, RxF, VxF, OMxF, R_tire)

        # 3. CALCUL DES FORCES DYNAMIQUES
        if mbs_data.process == 3: # Simulation dynamique
            T_F = np.zeros(4)
            T_M = np.zeros(4)

            if pen > 0:
                # Force normale (Sécurité anti-crash Bakker)
                T_F[3] = max(0.12, K_tire * pen)

                # APPEL BAKKER : Calcule Fx, Fy et Mz
                # IMPORTANT : On ne force plus T_F[2] à 0 pour permettre le virage !
                tgc_bakker_contact(T_F, T_M, angslip, angcamb, slip)

                # TRANSFORMATION VERS LE REPÈRE INERTIEL [I]
                F_inertial = matrix_vector_product(Rt_ground, T_F)
                M_inertial = matrix_vector_product(Rt_ground, T_M)

                Fx, Fy, Fz = F_inertial[1], F_inertial[2], F_inertial[3]
                Mx, My, Mz = M_inertial[1], M_inertial[2], M_inertial[3]
            
        elif mbs_data.process == 2: # Équilibre (Tassement)
            if pen > 0:
                Fz = K_tire * pen
            Fx = Fy = Mx = My = Mz = 0.0

    # Remplissage du vecteur de sortie Swr
    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF_tgc[1], dxF_tgc[2], dxF_tgc[3]]
    # =========================================================
    # SAUVEGARDE DES 3 COMPOSANTES DE LA FORCE DE CONTACT
    # (ixF correspond au numéro de la roue : 1, 2, 3 ou 4)
    # =========================================================
    
    # Swr[1] : Force longitudinale (Accélération / Freinage)
    mbs_data.set_output(Swr[1], f"F_Longi_Roue_{int(ixF)}")
    
    # Swr[2] : Force latérale (Adhérence en virage / Dérapage)
    mbs_data.set_output(Swr[2], f"F_Lat_Roue_{int(ixF)}")
    
    # Swr[3] : Force normale (Poids de la voiture sur cette roue)
    mbs_data.set_output(Swr[3], f"F_Norm_Roue_{int(ixF)}")
    return Swr