# -*- coding: utf-8 -*-
"""Module for the definition of user links forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2020

def user_LinkForces(Z, Zd, mbs_data, tsim, identity):
    """Compute the force in the given link.

    Parameters
    ----------
    Z : float
        The distance between the two anchor points of the link.
    Zd : float
        The relative velocity between the two anchor points of the link.
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    identity : int
        The identity of the computed link.

    Returns
    -------
    Flink : float
        The force in the current link.

    """

    Flink = 0.0

    if identity in [1, 2]:  # Ressorts AR
        um = mbs_data.user_model
        K = um['RearSuspension']['K']    # 22000 N/m
        C = um['RearSuspension']['C']    # 2000 N.s/m
        Z0 = um['RearSuspension']['Z0']  # à activer quand MRU / Accélération / Freinage : ressort ARRIÈRE allongé artificiellement

        Flink = K * (Z - Z0) + C * Zd
    
    if identity in [3, 4]:  # Ressorts AV
        um = mbs_data.user_model
        K = um['FrontSuspension']['K']   # 25000 N/m
        C = um['FrontSuspension']['C']   # 2000 N.s/m
        Z0 = um['FrontSuspension']['Z0']  # longueur naturelle [m] pour l'AVANT (on ne touche pas)

        Flink = K * (Z - Z0) + C * Zd

    return Flink
