#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Mon May  4 16:27:45 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Robotran_Mazda_MX5
#
#	==> Number of joints: 52
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S7 = sin(q[7])
    C7 = cos(q[7])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S36 = sin(q[36])
    C36 = cos(q[36])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_22 = s.dpt[2,18]*C7-s.dpt[3,18]*S7
    RLlnk2_32 = s.dpt[2,18]*S7+s.dpt[3,18]*C7
    POlnk2_22 = RLlnk2_22+s.dpt[2,1]
    POlnk2_32 = RLlnk2_32+s.dpt[3,1]
    ORlnk2_22 = -qd[7]*RLlnk2_32
    ORlnk2_32 = qd[7]*RLlnk2_22
    Plnk11 = s.dpt[1,1]-s.dpt[1,3]
    Plnk21 = POlnk2_22-s.dpt[2,3]
    Plnk31 = POlnk2_32-s.dpt[3,3]
    PPlnk1 = Plnk11*Plnk11+Plnk21*Plnk21+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e21 = Plnk21/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_22*e21+ORlnk2_32*e31
    RLlnk3_22 = s.dpt[2,26]*C13-s.dpt[3,26]*S13
    RLlnk3_32 = s.dpt[2,26]*S13+s.dpt[3,26]*C13
    POlnk3_22 = RLlnk3_22+s.dpt[2,4]
    POlnk3_32 = RLlnk3_32+s.dpt[3,4]
    ORlnk3_22 = -qd[13]*RLlnk3_32
    ORlnk3_32 = qd[13]*RLlnk3_22
    Plnk12 = -s.dpt[1,4]+s.dpt[1,6]
    Plnk22 = -POlnk3_22+s.dpt[2,6]
    Plnk32 = -POlnk3_32+s.dpt[3,6]
    PPlnk2 = Plnk12*Plnk12+Plnk22*Plnk22+Plnk32*Plnk32
    Z2 = sqrt(PPlnk2)
    e12 = Plnk12/Z2
    e22 = Plnk22/Z2
    e32 = Plnk32/Z2
    Zd2 = -ORlnk3_22*e22-ORlnk3_32*e32
    RLlnk6_22 = s.dpt[2,43]*C30-s.dpt[3,43]*S30
    RLlnk6_32 = s.dpt[2,43]*S30+s.dpt[3,43]*C30
    POlnk6_22 = RLlnk6_22+s.dpt[2,10]
    POlnk6_32 = RLlnk6_32+s.dpt[3,10]
    ORlnk6_22 = -qd[30]*RLlnk6_32
    ORlnk6_32 = qd[30]*RLlnk6_22
    Plnk13 = s.dpt[1,10]-s.dpt[1,8]
    Plnk23 = POlnk6_22-s.dpt[2,8]
    Plnk33 = POlnk6_32-s.dpt[3,8]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = ORlnk6_22*e23+ORlnk6_32*e33
    RLlnk8_22 = s.dpt[2,51]*C36-s.dpt[3,51]*S36
    RLlnk8_32 = s.dpt[2,51]*S36+s.dpt[3,51]*C36
    POlnk8_22 = RLlnk8_22+s.dpt[2,13]
    POlnk8_32 = RLlnk8_32+s.dpt[3,13]
    ORlnk8_22 = -qd[36]*RLlnk8_32
    ORlnk8_32 = qd[36]*RLlnk8_22
    Plnk14 = -s.dpt[1,11]+s.dpt[1,13]
    Plnk24 = POlnk8_22-s.dpt[2,11]
    Plnk34 = POlnk8_32-s.dpt[3,11]
    PPlnk4 = Plnk14*Plnk14+Plnk24*Plnk24+Plnk34*Plnk34
    Z4 = sqrt(PPlnk4)
    e14 = Plnk14/Z4
    e24 = Plnk24/Z4
    e34 = Plnk34/Z4
    Zd4 = ORlnk8_22*e24+ORlnk8_32*e34

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
    Flink4 = s.user_LinkForces(Z4,Zd4,s,tsim,4)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk21 = Flink1*e21
    fPlnk31 = Flink1*e31
    trqlnk6_1_1 = -fPlnk21*(s.dpt[3,3]-s.l[3,6])+fPlnk31*s.dpt[2,3]
    trqlnk6_1_2 = fPlnk11*(s.dpt[3,3]-s.l[3,6])-fPlnk31*(s.dpt[1,3]-s.l[1,6])
    trqlnk6_1_3 = -fPlnk11*s.dpt[2,3]+fPlnk21*(s.dpt[1,3]-s.l[1,6])
    fSlnk11 = Flink1*e11
    fSlnk21 = Flink1*(e21*C7+e31*S7)
    fSlnk31 = Flink1*(-e21*S7+e31*C7)
    trqlnk7_1_1 = fSlnk21*s.dpt[3,18]-fSlnk31*(s.dpt[2,18]-s.l[2,7])
    trqlnk7_1_2 = -fSlnk11*s.dpt[3,18]
    trqlnk7_1_3 = fSlnk11*(s.dpt[2,18]-s.l[2,7])
    fPlnk12 = Flink2*e12
    fPlnk22 = Flink2*(e22*C13+e32*S13)
    fPlnk32 = Flink2*(-e22*S13+e32*C13)
    trqlnk13_2_1 = -fPlnk22*s.dpt[3,26]+fPlnk32*(s.dpt[2,26]-s.l[2,13])
    trqlnk13_2_2 = fPlnk12*s.dpt[3,26]
    trqlnk13_2_3 = -fPlnk12*(s.dpt[2,26]-s.l[2,13])
    fSlnk12 = Flink2*e12
    fSlnk22 = Flink2*e22
    fSlnk32 = Flink2*e32
    frclnk6_2_1 = fPlnk11-fSlnk12
    frclnk6_2_2 = fPlnk21-fSlnk22
    frclnk6_2_3 = fPlnk31-fSlnk32
    trqlnk6_2_1 = trqlnk6_1_1+fSlnk22*(s.dpt[3,6]-s.l[3,6])-fSlnk32*s.dpt[2,6]
    trqlnk6_2_2 = trqlnk6_1_2-fSlnk12*(s.dpt[3,6]-s.l[3,6])+fSlnk32*(s.dpt[1,6]-s.l[1,6])
    trqlnk6_2_3 = trqlnk6_1_3+fSlnk12*s.dpt[2,6]-fSlnk22*(s.dpt[1,6]-s.l[1,6])
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*e23
    fPlnk33 = Flink3*e33
    frclnk6_3_1 = fPlnk13+frclnk6_2_1
    frclnk6_3_2 = fPlnk23+frclnk6_2_2
    frclnk6_3_3 = fPlnk33+frclnk6_2_3
    trqlnk6_3_1 = trqlnk6_2_1-fPlnk23*(s.dpt[3,8]-s.l[3,6])+fPlnk33*s.dpt[2,8]
    trqlnk6_3_2 = trqlnk6_2_2+fPlnk13*(s.dpt[3,8]-s.l[3,6])-fPlnk33*(s.dpt[1,8]-s.l[1,6])
    trqlnk6_3_3 = trqlnk6_2_3-fPlnk13*s.dpt[2,8]+fPlnk23*(s.dpt[1,8]-s.l[1,6])
    fSlnk13 = Flink3*e13
    fSlnk23 = Flink3*(e23*C30+e33*S30)
    fSlnk33 = Flink3*(-e23*S30+e33*C30)
    trqlnk30_3_1 = fSlnk23*s.dpt[3,43]-fSlnk33*(s.dpt[2,43]-s.l[2,30])
    trqlnk30_3_2 = -fSlnk13*s.dpt[3,43]
    trqlnk30_3_3 = fSlnk13*(s.dpt[2,43]-s.l[2,30])
    fPlnk14 = Flink4*e14
    fPlnk24 = Flink4*e24
    fPlnk34 = Flink4*e34
    frclnk6_4_1 = fPlnk14+frclnk6_3_1
    frclnk6_4_2 = fPlnk24+frclnk6_3_2
    frclnk6_4_3 = fPlnk34+frclnk6_3_3
    trqlnk6_4_1 = trqlnk6_3_1-fPlnk24*(s.dpt[3,11]-s.l[3,6])+fPlnk34*s.dpt[2,11]
    trqlnk6_4_2 = trqlnk6_3_2+fPlnk14*(s.dpt[3,11]-s.l[3,6])-fPlnk34*(s.dpt[1,11]-s.l[1,6])
    trqlnk6_4_3 = trqlnk6_3_3-fPlnk14*s.dpt[2,11]+fPlnk24*(s.dpt[1,11]-s.l[1,6])
    fSlnk14 = Flink4*e14
    fSlnk24 = Flink4*(e24*C36+e34*S36)
    fSlnk34 = Flink4*(-e24*S36+e34*C36)
    trqlnk36_4_1 = fSlnk24*s.dpt[3,51]-fSlnk34*(s.dpt[2,51]-s.l[2,36])
    trqlnk36_4_2 = -fSlnk14*s.dpt[3,51]
    trqlnk36_4_3 = fSlnk14*(s.dpt[2,51]-s.l[2,36])
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_4_1
    frc[2,6] = s.frc[2,6]+frclnk6_4_2
    frc[3,6] = s.frc[3,6]+frclnk6_4_3
    trq[1,6] = s.trq[1,6]+trqlnk6_4_1
    trq[2,6] = s.trq[2,6]+trqlnk6_4_2
    trq[3,6] = s.trq[3,6]+trqlnk6_4_3
    frc[1,7] = s.frc[1,7]-fSlnk11
    frc[2,7] = s.frc[2,7]-fSlnk21
    frc[3,7] = s.frc[3,7]-fSlnk31
    trq[1,7] = s.trq[1,7]+trqlnk7_1_1
    trq[2,7] = s.trq[2,7]+trqlnk7_1_2
    trq[3,7] = s.trq[3,7]+trqlnk7_1_3
    frc[1,13] = s.frc[1,13]+fPlnk12
    frc[2,13] = s.frc[2,13]+fPlnk22
    frc[3,13] = s.frc[3,13]+fPlnk32
    trq[1,13] = s.trq[1,13]+trqlnk13_2_1
    trq[2,13] = s.trq[2,13]+trqlnk13_2_2
    trq[3,13] = s.trq[3,13]+trqlnk13_2_3
    frc[1,30] = s.frc[1,30]-fSlnk13
    frc[2,30] = s.frc[2,30]-fSlnk23
    frc[3,30] = s.frc[3,30]-fSlnk33
    trq[1,30] = s.trq[1,30]+trqlnk30_3_1
    trq[2,30] = s.trq[2,30]+trqlnk30_3_2
    trq[3,30] = s.trq[3,30]+trqlnk30_3_3
    frc[1,36] = s.frc[1,36]-fSlnk14
    frc[2,36] = s.frc[2,36]-fSlnk24
    frc[3,36] = s.frc[3,36]-fSlnk34
    trq[1,36] = s.trq[1,36]+trqlnk36_4_1
    trq[2,36] = s.trq[2,36]+trqlnk36_4_2
    trq[3,36] = s.trq[3,36]+trqlnk36_4_3
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2
    Z[3] = Z3
    Zd[3] = Zd3
    Flink[3] = Flink3
    Z[4] = Z4
    Zd[4] = Zd4
    Flink[4] = Flink4

# Number of continuation lines = 0


