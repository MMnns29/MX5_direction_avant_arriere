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
#	==> Generation Date: Wed Apr  8 16:14:13 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: car
#
#	==> Number of joints: 43
#
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S18 = sin(q[18])
    C18 = cos(q[18])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S35 = sin(q[35])
    C35 = cos(q[35])
    S36 = sin(q[36])
    C36 = cos(q[36])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S32 = sin(q[32])
    C32 = cos(q[32])
    S33 = sin(q[33])
    C33 = cos(q[33])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S25 = sin(q[25])
    C25 = cos(q[25])
    S26 = sin(q[26])
    C26 = cos(q[26])
    S38 = sin(q[38])
    C38 = cos(q[38])
    S42 = sin(q[42])
    C42 = cos(q[42])
    S43 = sin(q[43])
    C43 = cos(q[43])
    S39 = sin(q[39])
    C39 = cos(q[39])
    S40 = sin(q[40])
    C40 = cos(q[40])
    S41 = sin(q[41])
    C41 = cos(q[41])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S31 = sin(q[31])
    C31 = cos(q[31])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjdqd1_22 = S12*S13
    ROjdqd1_32 = -C12*S13
    ROjdqd1_82 = -S12*C13
    ROjdqd1_92 = C12*C13
    ROjdqd1_43 = S13*S14
    ROjdqd1_53 = ROjdqd1_82*S14+C12*C14
    ROjdqd1_63 = ROjdqd1_92*S14+S12*C14
    ROjdqd1_73 = S13*C14
    ROjdqd1_83 = ROjdqd1_82*C14-C12*S14
    ROjdqd1_93 = ROjdqd1_92*C14-S12*S14
    ROjdqd1_14 = ROjdqd1_43*S15+C13*C15
    ROjdqd1_24 = ROjdqd1_22*C15+ROjdqd1_53*S15
    ROjdqd1_34 = ROjdqd1_32*C15+ROjdqd1_63*S15
    RLjdqd1_22 = s.dpt[2,22]*C12
    RLjdqd1_32 = s.dpt[2,22]*S12
    OMjdqd1_22 = qd[13]*C12
    OMjdqd1_32 = qd[13]*S12
    ORjdqd1_22 = -RLjdqd1_32*qd[12]
    ORjdqd1_32 = RLjdqd1_22*qd[12]
    Ompqpjdqd1_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd1_32 = qd[12]*qd[13]*C12
    Apqpjdqd1_22 = -ORjdqd1_32*qd[12]
    Apqpjdqd1_32 = ORjdqd1_22*qd[12]
    OMjdqd1_13 = qd[12]+qd[14]*C13
    OMjdqd1_23 = OMjdqd1_22+ROjdqd1_22*qd[14]
    OMjdqd1_33 = OMjdqd1_32+ROjdqd1_32*qd[14]
    Ompqpjdqd1_13 = qd[14]*(OMjdqd1_22*ROjdqd1_32-OMjdqd1_32*ROjdqd1_22)
    Ompqpjdqd1_23 = Ompqpjdqd1_22+qd[14]*(OMjdqd1_32*C13-ROjdqd1_32*qd[12])
    Ompqpjdqd1_33 = Ompqpjdqd1_32+qd[14]*(-OMjdqd1_22*C13+ROjdqd1_22*qd[12])
    OMjdqd1_14 = OMjdqd1_13+ROjdqd1_73*qd[15]
    OMjdqd1_24 = OMjdqd1_23+ROjdqd1_83*qd[15]
    OMjdqd1_34 = OMjdqd1_33+ROjdqd1_93*qd[15]
    Ompqpjdqd1_14 = Ompqpjdqd1_13+qd[15]*(OMjdqd1_23*ROjdqd1_93-OMjdqd1_33*ROjdqd1_83)
    Ompqpjdqd1_24 = Ompqpjdqd1_23+qd[15]*(-OMjdqd1_13*ROjdqd1_93+OMjdqd1_33*ROjdqd1_73)
    Ompqpjdqd1_34 = Ompqpjdqd1_33+qd[15]*(OMjdqd1_13*ROjdqd1_83-OMjdqd1_23*ROjdqd1_73)
    RLjdqd1_15 = ROjdqd1_14*s.dpt[1,24]+ROjdqd1_73*s.dpt[3,24]
    RLjdqd1_25 = ROjdqd1_24*s.dpt[1,24]+ROjdqd1_83*s.dpt[3,24]
    RLjdqd1_35 = ROjdqd1_34*s.dpt[1,24]+ROjdqd1_93*s.dpt[3,24]
    ORjdqd1_15 = OMjdqd1_24*RLjdqd1_35-OMjdqd1_34*RLjdqd1_25
    ORjdqd1_25 = -OMjdqd1_14*RLjdqd1_35+OMjdqd1_34*RLjdqd1_15
    ORjdqd1_35 = OMjdqd1_14*RLjdqd1_25-OMjdqd1_24*RLjdqd1_15
    Apqpjdqd1_15 = OMjdqd1_24*ORjdqd1_35-OMjdqd1_34*ORjdqd1_25+Ompqpjdqd1_24*RLjdqd1_35-Ompqpjdqd1_34*RLjdqd1_25
    Apqpjdqd1_25 = Apqpjdqd1_22-OMjdqd1_14*ORjdqd1_35+OMjdqd1_34*ORjdqd1_15-Ompqpjdqd1_14*RLjdqd1_35+Ompqpjdqd1_34* \
 	  RLjdqd1_15
    Apqpjdqd1_35 = Apqpjdqd1_32+OMjdqd1_14*ORjdqd1_25-OMjdqd1_24*ORjdqd1_15+Ompqpjdqd1_14*RLjdqd1_25-Ompqpjdqd1_24* \
 	  RLjdqd1_15
    RLjdqd2_22 = s.dpt[2,28]*C17
    RLjdqd2_32 = s.dpt[2,28]*S17
    ORjdqd2_22 = -RLjdqd2_32*qd[17]
    ORjdqd2_32 = RLjdqd2_22*qd[17]
    Apqpjdqd2_22 = -ORjdqd2_32*qd[17]
    Apqpjdqd2_32 = ORjdqd2_22*qd[17]
    jdqd2 = Apqpjdqd1_25-Apqpjdqd2_22
    jdqd3 = Apqpjdqd1_35-Apqpjdqd2_32
    ROjdqd3_22 = S7*S8
    ROjdqd3_32 = -C7*S8
    ROjdqd3_82 = -S7*C8
    ROjdqd3_92 = C7*C8
    ROjdqd3_43 = S8*S9
    ROjdqd3_53 = ROjdqd3_82*S9+C7*C9
    ROjdqd3_63 = ROjdqd3_92*S9+S7*C9
    ROjdqd3_73 = S8*C9
    ROjdqd3_83 = ROjdqd3_82*C9-C7*S9
    ROjdqd3_93 = ROjdqd3_92*C9-S7*S9
    ROjdqd3_14 = ROjdqd3_43*S10+C10*C8
    ROjdqd3_24 = ROjdqd3_22*C10+ROjdqd3_53*S10
    ROjdqd3_34 = ROjdqd3_32*C10+ROjdqd3_63*S10
    RLjdqd3_22 = s.dpt[2,16]*C7
    RLjdqd3_32 = s.dpt[2,16]*S7
    OMjdqd3_22 = qd[8]*C7
    OMjdqd3_32 = qd[8]*S7
    ORjdqd3_22 = -RLjdqd3_32*qd[7]
    ORjdqd3_32 = RLjdqd3_22*qd[7]
    Ompqpjdqd3_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd3_32 = qd[7]*qd[8]*C7
    Apqpjdqd3_22 = -ORjdqd3_32*qd[7]
    Apqpjdqd3_32 = ORjdqd3_22*qd[7]
    OMjdqd3_13 = qd[7]+qd[9]*C8
    OMjdqd3_23 = OMjdqd3_22+ROjdqd3_22*qd[9]
    OMjdqd3_33 = OMjdqd3_32+ROjdqd3_32*qd[9]
    Ompqpjdqd3_13 = qd[9]*(OMjdqd3_22*ROjdqd3_32-OMjdqd3_32*ROjdqd3_22)
    Ompqpjdqd3_23 = Ompqpjdqd3_22+qd[9]*(OMjdqd3_32*C8-ROjdqd3_32*qd[7])
    Ompqpjdqd3_33 = Ompqpjdqd3_32+qd[9]*(-OMjdqd3_22*C8+ROjdqd3_22*qd[7])
    OMjdqd3_14 = OMjdqd3_13+ROjdqd3_73*qd[10]
    OMjdqd3_24 = OMjdqd3_23+ROjdqd3_83*qd[10]
    OMjdqd3_34 = OMjdqd3_33+ROjdqd3_93*qd[10]
    Ompqpjdqd3_14 = Ompqpjdqd3_13+qd[10]*(OMjdqd3_23*ROjdqd3_93-OMjdqd3_33*ROjdqd3_83)
    Ompqpjdqd3_24 = Ompqpjdqd3_23+qd[10]*(-OMjdqd3_13*ROjdqd3_93+OMjdqd3_33*ROjdqd3_73)
    Ompqpjdqd3_34 = Ompqpjdqd3_33+qd[10]*(OMjdqd3_13*ROjdqd3_83-OMjdqd3_23*ROjdqd3_73)
    RLjdqd3_15 = ROjdqd3_14*s.dpt[1,19]+ROjdqd3_73*s.dpt[3,19]
    RLjdqd3_25 = ROjdqd3_24*s.dpt[1,19]+ROjdqd3_83*s.dpt[3,19]
    RLjdqd3_35 = ROjdqd3_34*s.dpt[1,19]+ROjdqd3_93*s.dpt[3,19]
    ORjdqd3_15 = OMjdqd3_24*RLjdqd3_35-OMjdqd3_34*RLjdqd3_25
    ORjdqd3_25 = -OMjdqd3_14*RLjdqd3_35+OMjdqd3_34*RLjdqd3_15
    ORjdqd3_35 = OMjdqd3_14*RLjdqd3_25-OMjdqd3_24*RLjdqd3_15
    Apqpjdqd3_15 = OMjdqd3_24*ORjdqd3_35-OMjdqd3_34*ORjdqd3_25+Ompqpjdqd3_24*RLjdqd3_35-Ompqpjdqd3_34*RLjdqd3_25
    Apqpjdqd3_25 = Apqpjdqd3_22-OMjdqd3_14*ORjdqd3_35+OMjdqd3_34*ORjdqd3_15-Ompqpjdqd3_14*RLjdqd3_35+Ompqpjdqd3_34* \
 	  RLjdqd3_15
    Apqpjdqd3_35 = Apqpjdqd3_32+OMjdqd3_14*ORjdqd3_25-OMjdqd3_24*ORjdqd3_15+Ompqpjdqd3_14*RLjdqd3_25-Ompqpjdqd3_24* \
 	  RLjdqd3_15
    RLjdqd4_22 = s.dpt[2,30]*C18
    RLjdqd4_32 = s.dpt[2,30]*S18
    ORjdqd4_22 = -RLjdqd4_32*qd[18]
    ORjdqd4_32 = RLjdqd4_22*qd[18]
    Apqpjdqd4_22 = -ORjdqd4_32*qd[18]
    Apqpjdqd4_32 = ORjdqd4_22*qd[18]
    jdqd5 = Apqpjdqd3_25-Apqpjdqd4_22
    jdqd6 = Apqpjdqd3_35-Apqpjdqd4_32
    RLjdqd5_22 = s.dpt[2,32]*C19
    RLjdqd5_32 = s.dpt[2,32]*S19
    ORjdqd5_22 = -RLjdqd5_32*qd[19]
    ORjdqd5_32 = RLjdqd5_22*qd[19]
    Apqpjdqd5_22 = -ORjdqd5_32*qd[19]
    Apqpjdqd5_32 = ORjdqd5_22*qd[19]
    ROjdqd6_82 = -C35*S36-S35*C36
    ROjdqd6_92 = C35*C36-S35*S36
    RLjdqd6_22 = s.dpt[2,50]*C35
    RLjdqd6_32 = s.dpt[2,50]*S35
    OMjdqd6_12 = qd[35]+qd[36]
    ORjdqd6_22 = -RLjdqd6_32*qd[35]
    ORjdqd6_32 = RLjdqd6_22*qd[35]
    Apqpjdqd6_22 = -ORjdqd6_32*qd[35]
    Apqpjdqd6_32 = ORjdqd6_22*qd[35]
    RLjdqd6_23 = ROjdqd6_82*s.dpt[3,53]
    RLjdqd6_33 = ROjdqd6_92*s.dpt[3,53]
    ORjdqd6_23 = -OMjdqd6_12*RLjdqd6_33
    ORjdqd6_33 = OMjdqd6_12*RLjdqd6_23
    Apqpjdqd6_23 = Apqpjdqd6_22-OMjdqd6_12*ORjdqd6_33
    Apqpjdqd6_33 = Apqpjdqd6_32+OMjdqd6_12*ORjdqd6_23
    jdqd8 = Apqpjdqd5_22-Apqpjdqd6_23
    jdqd9 = Apqpjdqd5_32-Apqpjdqd6_33
    RLjdqd7_22 = s.dpt[2,34]*C20
    RLjdqd7_32 = s.dpt[2,34]*S20
    ORjdqd7_22 = -RLjdqd7_32*qd[20]
    ORjdqd7_32 = RLjdqd7_22*qd[20]
    Apqpjdqd7_22 = -ORjdqd7_32*qd[20]
    Apqpjdqd7_32 = ORjdqd7_22*qd[20]
    ROjdqd8_82 = -C32*S33-S32*C33
    ROjdqd8_92 = C32*C33-S32*S33
    RLjdqd8_22 = s.dpt[2,45]*C32
    RLjdqd8_32 = s.dpt[2,45]*S32
    OMjdqd8_12 = qd[32]+qd[33]
    ORjdqd8_22 = -RLjdqd8_32*qd[32]
    ORjdqd8_32 = RLjdqd8_22*qd[32]
    Apqpjdqd8_22 = -ORjdqd8_32*qd[32]
    Apqpjdqd8_32 = ORjdqd8_22*qd[32]
    RLjdqd8_23 = ROjdqd8_82*s.dpt[3,47]
    RLjdqd8_33 = ROjdqd8_92*s.dpt[3,47]
    ORjdqd8_23 = -OMjdqd8_12*RLjdqd8_33
    ORjdqd8_33 = OMjdqd8_12*RLjdqd8_23
    Apqpjdqd8_23 = Apqpjdqd8_22-OMjdqd8_12*ORjdqd8_33
    Apqpjdqd8_33 = Apqpjdqd8_32+OMjdqd8_12*ORjdqd8_23
    jdqd11 = Apqpjdqd7_22-Apqpjdqd8_23
    jdqd12 = Apqpjdqd7_32-Apqpjdqd8_33
    ROjdqd9_42 = S21*S22
    ROjdqd9_62 = C21*S22
    ROjdqd9_72 = S21*C22
    ROjdqd9_92 = C21*C22
    ROjdqd9_73 = ROjdqd9_72*C23+C21*S23
    ROjdqd9_83 = -S22*C23
    ROjdqd9_93 = ROjdqd9_92*C23-S21*S23
    RLjdqd9_12 = s.dpt[1,36]*C21
    RLjdqd9_32 = -s.dpt[1,36]*S21
    OMjdqd9_12 = qd[22]*C21
    OMjdqd9_32 = -qd[22]*S21
    ORjdqd9_12 = RLjdqd9_32*qd[21]
    ORjdqd9_32 = -RLjdqd9_12*qd[21]
    Ompqpjdqd9_12 = -qd[21]*qd[22]*S21
    Ompqpjdqd9_32 = -qd[21]*qd[22]*C21
    Apqpjdqd9_12 = ORjdqd9_32*qd[21]
    Apqpjdqd9_32 = -ORjdqd9_12*qd[21]
    OMjdqd9_13 = OMjdqd9_12+ROjdqd9_42*qd[23]
    OMjdqd9_23 = qd[21]+qd[23]*C22
    OMjdqd9_33 = OMjdqd9_32+ROjdqd9_62*qd[23]
    Ompqpjdqd9_13 = Ompqpjdqd9_12+qd[23]*(-OMjdqd9_32*C22+ROjdqd9_62*qd[21])
    Ompqpjdqd9_23 = qd[23]*(-OMjdqd9_12*ROjdqd9_62+OMjdqd9_32*ROjdqd9_42)
    Ompqpjdqd9_33 = Ompqpjdqd9_32+qd[23]*(OMjdqd9_12*C22-ROjdqd9_42*qd[21])
    RLjdqd9_14 = ROjdqd9_73*s.dpt[3,38]
    RLjdqd9_24 = ROjdqd9_83*s.dpt[3,38]
    RLjdqd9_34 = ROjdqd9_93*s.dpt[3,38]
    ORjdqd9_14 = OMjdqd9_23*RLjdqd9_34-OMjdqd9_33*RLjdqd9_24
    ORjdqd9_24 = -OMjdqd9_13*RLjdqd9_34+OMjdqd9_33*RLjdqd9_14
    ORjdqd9_34 = OMjdqd9_13*RLjdqd9_24-OMjdqd9_23*RLjdqd9_14
    Apqpjdqd9_14 = Apqpjdqd9_12+OMjdqd9_23*ORjdqd9_34-OMjdqd9_33*ORjdqd9_24+Ompqpjdqd9_23*RLjdqd9_34-Ompqpjdqd9_33* \
 	  RLjdqd9_24
    Apqpjdqd9_24 = -OMjdqd9_13*ORjdqd9_34+OMjdqd9_33*ORjdqd9_14-Ompqpjdqd9_13*RLjdqd9_34+Ompqpjdqd9_33*RLjdqd9_14
    Apqpjdqd9_34 = Apqpjdqd9_32+OMjdqd9_13*ORjdqd9_24-OMjdqd9_23*ORjdqd9_14+Ompqpjdqd9_13*RLjdqd9_24-Ompqpjdqd9_23* \
 	  RLjdqd9_14
    RLjdqd10_22 = s.dpt[2,23]*C12
    RLjdqd10_32 = s.dpt[2,23]*S12
    ORjdqd10_22 = -RLjdqd10_32*qd[12]
    ORjdqd10_32 = RLjdqd10_22*qd[12]
    Apqpjdqd10_22 = -ORjdqd10_32*qd[12]
    Apqpjdqd10_32 = ORjdqd10_22*qd[12]
    jdqd14 = -Apqpjdqd10_22+Apqpjdqd9_24
    jdqd15 = -Apqpjdqd10_32+Apqpjdqd9_34
    ROjdqd11_12 = C21*C24-S21*S24
    ROjdqd11_32 = -C21*S24-S21*C24
    ROjdqd11_72 = C21*S24+S21*C24
    ROjdqd11_92 = C21*C24-S21*S24
    ROjdqd11_43 = ROjdqd11_72*S25
    ROjdqd11_63 = ROjdqd11_92*S25
    ROjdqd11_73 = ROjdqd11_72*C25
    ROjdqd11_93 = ROjdqd11_92*C25
    ROjdqd11_74 = ROjdqd11_12*S26+ROjdqd11_73*C26
    ROjdqd11_84 = -S25*C26
    ROjdqd11_94 = ROjdqd11_32*S26+ROjdqd11_93*C26
    OMjdqd11_22 = qd[21]+qd[24]
    RLjdqd11_13 = ROjdqd11_12*s.dpt[1,39]
    RLjdqd11_33 = ROjdqd11_32*s.dpt[1,39]
    OMjdqd11_13 = ROjdqd11_12*qd[25]
    OMjdqd11_33 = ROjdqd11_32*qd[25]
    ORjdqd11_13 = OMjdqd11_22*RLjdqd11_33
    ORjdqd11_33 = -OMjdqd11_22*RLjdqd11_13
    Ompqpjdqd11_13 = OMjdqd11_22*ROjdqd11_32*qd[25]
    Ompqpjdqd11_33 = -OMjdqd11_22*ROjdqd11_12*qd[25]
    Apqpjdqd11_13 = OMjdqd11_22*ORjdqd11_33
    Apqpjdqd11_33 = -OMjdqd11_22*ORjdqd11_13
    OMjdqd11_14 = OMjdqd11_13+ROjdqd11_43*qd[26]
    OMjdqd11_24 = OMjdqd11_22+qd[26]*C25
    OMjdqd11_34 = OMjdqd11_33+ROjdqd11_63*qd[26]
    Ompqpjdqd11_14 = Ompqpjdqd11_13+qd[26]*(OMjdqd11_22*ROjdqd11_63-OMjdqd11_33*C25)
    Ompqpjdqd11_24 = qd[26]*(-OMjdqd11_13*ROjdqd11_63+OMjdqd11_33*ROjdqd11_43)
    Ompqpjdqd11_34 = Ompqpjdqd11_33+qd[26]*(OMjdqd11_13*C25-OMjdqd11_22*ROjdqd11_43)
    RLjdqd11_15 = ROjdqd11_74*s.dpt[3,40]
    RLjdqd11_25 = ROjdqd11_84*s.dpt[3,40]
    RLjdqd11_35 = ROjdqd11_94*s.dpt[3,40]
    ORjdqd11_15 = OMjdqd11_24*RLjdqd11_35-OMjdqd11_34*RLjdqd11_25
    ORjdqd11_25 = -OMjdqd11_14*RLjdqd11_35+OMjdqd11_34*RLjdqd11_15
    ORjdqd11_35 = OMjdqd11_14*RLjdqd11_25-OMjdqd11_24*RLjdqd11_15
    Apqpjdqd11_15 = Apqpjdqd11_13+OMjdqd11_24*ORjdqd11_35-OMjdqd11_34*ORjdqd11_25+Ompqpjdqd11_24*RLjdqd11_35- \
 	  Ompqpjdqd11_34*RLjdqd11_25
    Apqpjdqd11_25 = -OMjdqd11_14*ORjdqd11_35+OMjdqd11_34*ORjdqd11_15-Ompqpjdqd11_14*RLjdqd11_35+Ompqpjdqd11_34* \
 	  RLjdqd11_15
    Apqpjdqd11_35 = Apqpjdqd11_33+OMjdqd11_14*ORjdqd11_25-OMjdqd11_24*ORjdqd11_15+Ompqpjdqd11_14*RLjdqd11_25- \
 	  Ompqpjdqd11_24*RLjdqd11_15
    RLjdqd12_22 = s.dpt[2,17]*C7
    RLjdqd12_32 = s.dpt[2,17]*S7
    ORjdqd12_22 = -RLjdqd12_32*qd[7]
    ORjdqd12_32 = RLjdqd12_22*qd[7]
    Apqpjdqd12_22 = -ORjdqd12_32*qd[7]
    Apqpjdqd12_32 = ORjdqd12_22*qd[7]
    jdqd17 = Apqpjdqd11_25-Apqpjdqd12_22
    jdqd18 = Apqpjdqd11_35-Apqpjdqd12_32
    RLjdqd13_22 = s.dpt[2,46]*C32
    RLjdqd13_32 = s.dpt[2,46]*S32
    ORjdqd13_22 = -RLjdqd13_32*qd[32]
    ORjdqd13_32 = RLjdqd13_22*qd[32]
    Apqpjdqd13_22 = -ORjdqd13_32*qd[32]
    Apqpjdqd13_32 = ORjdqd13_22*qd[32]
    ROjdqd14_42 = S38*S42
    ROjdqd14_62 = C38*S42
    ROjdqd14_72 = S38*C42
    ROjdqd14_92 = C38*C42
    ROjdqd14_73 = ROjdqd14_72*C43+C38*S43
    ROjdqd14_83 = -S42*C43
    ROjdqd14_93 = ROjdqd14_92*C43-S38*S43
    RLjdqd14_12 = s.dpt[1,56]*C38
    RLjdqd14_32 = -s.dpt[1,56]*S38
    OMjdqd14_12 = qd[42]*C38
    OMjdqd14_32 = -qd[42]*S38
    ORjdqd14_12 = RLjdqd14_32*qd[38]
    ORjdqd14_32 = -RLjdqd14_12*qd[38]
    Ompqpjdqd14_12 = -qd[38]*qd[42]*S38
    Ompqpjdqd14_32 = -qd[38]*qd[42]*C38
    Apqpjdqd14_12 = ORjdqd14_32*qd[38]
    Apqpjdqd14_32 = -ORjdqd14_12*qd[38]
    OMjdqd14_13 = OMjdqd14_12+ROjdqd14_42*qd[43]
    OMjdqd14_23 = qd[38]+qd[43]*C42
    OMjdqd14_33 = OMjdqd14_32+ROjdqd14_62*qd[43]
    Ompqpjdqd14_13 = Ompqpjdqd14_12+qd[43]*(-OMjdqd14_32*C42+ROjdqd14_62*qd[38])
    Ompqpjdqd14_23 = qd[43]*(-OMjdqd14_12*ROjdqd14_62+OMjdqd14_32*ROjdqd14_42)
    Ompqpjdqd14_33 = Ompqpjdqd14_32+qd[43]*(OMjdqd14_12*C42-ROjdqd14_42*qd[38])
    RLjdqd14_14 = ROjdqd14_73*s.dpt[3,59]
    RLjdqd14_24 = ROjdqd14_83*s.dpt[3,59]
    RLjdqd14_34 = ROjdqd14_93*s.dpt[3,59]
    ORjdqd14_14 = OMjdqd14_23*RLjdqd14_34-OMjdqd14_33*RLjdqd14_24
    ORjdqd14_24 = -OMjdqd14_13*RLjdqd14_34+OMjdqd14_33*RLjdqd14_14
    ORjdqd14_34 = OMjdqd14_13*RLjdqd14_24-OMjdqd14_23*RLjdqd14_14
    Apqpjdqd14_14 = Apqpjdqd14_12+OMjdqd14_23*ORjdqd14_34-OMjdqd14_33*ORjdqd14_24+Ompqpjdqd14_23*RLjdqd14_34- \
 	  Ompqpjdqd14_33*RLjdqd14_24
    Apqpjdqd14_24 = -OMjdqd14_13*ORjdqd14_34+OMjdqd14_33*ORjdqd14_14-Ompqpjdqd14_13*RLjdqd14_34+Ompqpjdqd14_33* \
 	  RLjdqd14_14
    Apqpjdqd14_34 = Apqpjdqd14_32+OMjdqd14_13*ORjdqd14_24-OMjdqd14_23*ORjdqd14_14+Ompqpjdqd14_13*RLjdqd14_24- \
 	  Ompqpjdqd14_23*RLjdqd14_14
    jdqd20 = Apqpjdqd13_22-Apqpjdqd14_24
    jdqd21 = Apqpjdqd13_32-Apqpjdqd14_34
    ROjdqd15_12 = C38*C39-S38*S39
    ROjdqd15_32 = -C38*S39-S38*C39
    ROjdqd15_72 = C38*S39+S38*C39
    ROjdqd15_92 = C38*C39-S38*S39
    ROjdqd15_43 = ROjdqd15_72*S40
    ROjdqd15_63 = ROjdqd15_92*S40
    ROjdqd15_73 = ROjdqd15_72*C40
    ROjdqd15_93 = ROjdqd15_92*C40
    ROjdqd15_74 = ROjdqd15_12*S41+ROjdqd15_73*C41
    ROjdqd15_84 = -S40*C41
    ROjdqd15_94 = ROjdqd15_32*S41+ROjdqd15_93*C41
    OMjdqd15_22 = qd[38]+qd[39]
    RLjdqd15_13 = ROjdqd15_12*s.dpt[1,57]
    RLjdqd15_33 = ROjdqd15_32*s.dpt[1,57]
    OMjdqd15_13 = ROjdqd15_12*qd[40]
    OMjdqd15_33 = ROjdqd15_32*qd[40]
    ORjdqd15_13 = OMjdqd15_22*RLjdqd15_33
    ORjdqd15_33 = -OMjdqd15_22*RLjdqd15_13
    Ompqpjdqd15_13 = OMjdqd15_22*ROjdqd15_32*qd[40]
    Ompqpjdqd15_33 = -OMjdqd15_22*ROjdqd15_12*qd[40]
    Apqpjdqd15_13 = OMjdqd15_22*ORjdqd15_33
    Apqpjdqd15_33 = -OMjdqd15_22*ORjdqd15_13
    OMjdqd15_14 = OMjdqd15_13+ROjdqd15_43*qd[41]
    OMjdqd15_24 = OMjdqd15_22+qd[41]*C40
    OMjdqd15_34 = OMjdqd15_33+ROjdqd15_63*qd[41]
    Ompqpjdqd15_14 = Ompqpjdqd15_13+qd[41]*(OMjdqd15_22*ROjdqd15_63-OMjdqd15_33*C40)
    Ompqpjdqd15_24 = qd[41]*(-OMjdqd15_13*ROjdqd15_63+OMjdqd15_33*ROjdqd15_43)
    Ompqpjdqd15_34 = Ompqpjdqd15_33+qd[41]*(OMjdqd15_13*C40-OMjdqd15_22*ROjdqd15_43)
    RLjdqd15_15 = ROjdqd15_74*s.dpt[3,58]
    RLjdqd15_25 = ROjdqd15_84*s.dpt[3,58]
    RLjdqd15_35 = ROjdqd15_94*s.dpt[3,58]
    ORjdqd15_15 = OMjdqd15_24*RLjdqd15_35-OMjdqd15_34*RLjdqd15_25
    ORjdqd15_25 = -OMjdqd15_14*RLjdqd15_35+OMjdqd15_34*RLjdqd15_15
    ORjdqd15_35 = OMjdqd15_14*RLjdqd15_25-OMjdqd15_24*RLjdqd15_15
    Apqpjdqd15_15 = Apqpjdqd15_13+OMjdqd15_24*ORjdqd15_35-OMjdqd15_34*ORjdqd15_25+Ompqpjdqd15_24*RLjdqd15_35- \
 	  Ompqpjdqd15_34*RLjdqd15_25
    Apqpjdqd15_25 = -OMjdqd15_14*ORjdqd15_35+OMjdqd15_34*ORjdqd15_15-Ompqpjdqd15_14*RLjdqd15_35+Ompqpjdqd15_34* \
 	  RLjdqd15_15
    Apqpjdqd15_35 = Apqpjdqd15_33+OMjdqd15_14*ORjdqd15_25-OMjdqd15_24*ORjdqd15_15+Ompqpjdqd15_14*RLjdqd15_25- \
 	  Ompqpjdqd15_24*RLjdqd15_15
    RLjdqd16_22 = s.dpt[2,51]*C35
    RLjdqd16_32 = s.dpt[2,51]*S35
    ORjdqd16_22 = -RLjdqd16_32*qd[35]
    ORjdqd16_32 = RLjdqd16_22*qd[35]
    Apqpjdqd16_22 = -ORjdqd16_32*qd[35]
    Apqpjdqd16_32 = ORjdqd16_22*qd[35]
    jdqd23 = Apqpjdqd15_25-Apqpjdqd16_22
    jdqd24 = Apqpjdqd15_35-Apqpjdqd16_32
    ROjdqd17_22 = S7*S8
    ROjdqd17_32 = -C7*S8
    ROjdqd17_82 = -S7*C8
    ROjdqd17_92 = C7*C8
    ROjdqd17_43 = S8*S9
    ROjdqd17_53 = ROjdqd17_82*S9+C7*C9
    ROjdqd17_63 = ROjdqd17_92*S9+S7*C9
    ROjdqd17_73 = S8*C9
    ROjdqd17_83 = ROjdqd17_82*C9-C7*S9
    ROjdqd17_93 = ROjdqd17_92*C9-S7*S9
    ROjdqd17_14 = ROjdqd17_43*S10+C10*C8
    ROjdqd17_24 = ROjdqd17_22*C10+ROjdqd17_53*S10
    ROjdqd17_34 = ROjdqd17_32*C10+ROjdqd17_63*S10
    RLjdqd17_22 = s.dpt[2,16]*C7
    RLjdqd17_32 = s.dpt[2,16]*S7
    OMjdqd17_22 = qd[8]*C7
    OMjdqd17_32 = qd[8]*S7
    ORjdqd17_22 = -RLjdqd17_32*qd[7]
    ORjdqd17_32 = RLjdqd17_22*qd[7]
    Ompqpjdqd17_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd17_32 = qd[7]*qd[8]*C7
    Apqpjdqd17_22 = -ORjdqd17_32*qd[7]
    Apqpjdqd17_32 = ORjdqd17_22*qd[7]
    OMjdqd17_13 = qd[7]+qd[9]*C8
    OMjdqd17_23 = OMjdqd17_22+ROjdqd17_22*qd[9]
    OMjdqd17_33 = OMjdqd17_32+ROjdqd17_32*qd[9]
    Ompqpjdqd17_13 = qd[9]*(OMjdqd17_22*ROjdqd17_32-OMjdqd17_32*ROjdqd17_22)
    Ompqpjdqd17_23 = Ompqpjdqd17_22+qd[9]*(OMjdqd17_32*C8-ROjdqd17_32*qd[7])
    Ompqpjdqd17_33 = Ompqpjdqd17_32+qd[9]*(-OMjdqd17_22*C8+ROjdqd17_22*qd[7])
    OMjdqd17_14 = OMjdqd17_13+ROjdqd17_73*qd[10]
    OMjdqd17_24 = OMjdqd17_23+ROjdqd17_83*qd[10]
    OMjdqd17_34 = OMjdqd17_33+ROjdqd17_93*qd[10]
    Ompqpjdqd17_14 = Ompqpjdqd17_13+qd[10]*(OMjdqd17_23*ROjdqd17_93-OMjdqd17_33*ROjdqd17_83)
    Ompqpjdqd17_24 = Ompqpjdqd17_23+qd[10]*(-OMjdqd17_13*ROjdqd17_93+OMjdqd17_33*ROjdqd17_73)
    Ompqpjdqd17_34 = Ompqpjdqd17_33+qd[10]*(OMjdqd17_13*ROjdqd17_83-OMjdqd17_23*ROjdqd17_73)
    RLjdqd17_15 = ROjdqd17_14*s.dpt[1,18]+ROjdqd17_73*s.dpt[3,18]
    RLjdqd17_25 = ROjdqd17_24*s.dpt[1,18]+ROjdqd17_83*s.dpt[3,18]
    RLjdqd17_35 = ROjdqd17_34*s.dpt[1,18]+ROjdqd17_93*s.dpt[3,18]
    ORjdqd17_15 = OMjdqd17_24*RLjdqd17_35-OMjdqd17_34*RLjdqd17_25
    ORjdqd17_25 = -OMjdqd17_14*RLjdqd17_35+OMjdqd17_34*RLjdqd17_15
    ORjdqd17_35 = OMjdqd17_14*RLjdqd17_25-OMjdqd17_24*RLjdqd17_15
    Apqpjdqd17_15 = OMjdqd17_24*ORjdqd17_35-OMjdqd17_34*ORjdqd17_25+Ompqpjdqd17_24*RLjdqd17_35-Ompqpjdqd17_34* \
 	  RLjdqd17_25
    Apqpjdqd17_25 = Apqpjdqd17_22-OMjdqd17_14*ORjdqd17_35+OMjdqd17_34*ORjdqd17_15-Ompqpjdqd17_14*RLjdqd17_35+ \
 	  Ompqpjdqd17_34*RLjdqd17_15
    Apqpjdqd17_35 = Apqpjdqd17_32+OMjdqd17_14*ORjdqd17_25-OMjdqd17_24*ORjdqd17_15+Ompqpjdqd17_14*RLjdqd17_25- \
 	  Ompqpjdqd17_24*RLjdqd17_15
    ROjdqd18_23 = C28*S29
    ROjdqd18_33 = S28*S29
    ROjdqd18_53 = C28*C29
    ROjdqd18_63 = S28*C29
    OMjdqd18_23 = -qd[29]*S28
    OMjdqd18_33 = qd[29]*C28
    Ompqpjdqd18_23 = -qd[28]*qd[29]*C28
    Ompqpjdqd18_33 = -qd[28]*qd[29]*S28
    RLjdqd18_14 = s.dpt[1,43]*C29-s.dpt[2,43]*S29
    RLjdqd18_24 = ROjdqd18_23*s.dpt[1,43]+ROjdqd18_53*s.dpt[2,43]
    RLjdqd18_34 = ROjdqd18_33*s.dpt[1,43]+ROjdqd18_63*s.dpt[2,43]
    ORjdqd18_14 = OMjdqd18_23*RLjdqd18_34-OMjdqd18_33*RLjdqd18_24
    ORjdqd18_24 = OMjdqd18_33*RLjdqd18_14-RLjdqd18_34*qd[28]
    ORjdqd18_34 = -OMjdqd18_23*RLjdqd18_14+RLjdqd18_24*qd[28]
    Apqpjdqd18_14 = OMjdqd18_23*ORjdqd18_34-OMjdqd18_33*ORjdqd18_24+Ompqpjdqd18_23*RLjdqd18_34-Ompqpjdqd18_33* \
 	  RLjdqd18_24
    Apqpjdqd18_24 = OMjdqd18_33*ORjdqd18_14-ORjdqd18_34*qd[28]+Ompqpjdqd18_33*RLjdqd18_14
    Apqpjdqd18_34 = -OMjdqd18_23*ORjdqd18_14+ORjdqd18_24*qd[28]-Ompqpjdqd18_23*RLjdqd18_14
    jdqd25 = Apqpjdqd17_15-Apqpjdqd18_14
    jdqd26 = Apqpjdqd17_25-Apqpjdqd18_24
    jdqd27 = Apqpjdqd17_35-Apqpjdqd18_34
    ROjdqd19_22 = S12*S13
    ROjdqd19_32 = -C12*S13
    ROjdqd19_82 = -S12*C13
    ROjdqd19_92 = C12*C13
    ROjdqd19_43 = S13*S14
    ROjdqd19_53 = ROjdqd19_82*S14+C12*C14
    ROjdqd19_63 = ROjdqd19_92*S14+S12*C14
    ROjdqd19_73 = S13*C14
    ROjdqd19_83 = ROjdqd19_82*C14-C12*S14
    ROjdqd19_93 = ROjdqd19_92*C14-S12*S14
    ROjdqd19_14 = ROjdqd19_43*S15+C13*C15
    ROjdqd19_24 = ROjdqd19_22*C15+ROjdqd19_53*S15
    ROjdqd19_34 = ROjdqd19_32*C15+ROjdqd19_63*S15
    RLjdqd19_22 = s.dpt[2,22]*C12
    RLjdqd19_32 = s.dpt[2,22]*S12
    OMjdqd19_22 = qd[13]*C12
    OMjdqd19_32 = qd[13]*S12
    ORjdqd19_22 = -RLjdqd19_32*qd[12]
    ORjdqd19_32 = RLjdqd19_22*qd[12]
    Ompqpjdqd19_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd19_32 = qd[12]*qd[13]*C12
    Apqpjdqd19_22 = -ORjdqd19_32*qd[12]
    Apqpjdqd19_32 = ORjdqd19_22*qd[12]
    OMjdqd19_13 = qd[12]+qd[14]*C13
    OMjdqd19_23 = OMjdqd19_22+ROjdqd19_22*qd[14]
    OMjdqd19_33 = OMjdqd19_32+ROjdqd19_32*qd[14]
    Ompqpjdqd19_13 = qd[14]*(OMjdqd19_22*ROjdqd19_32-OMjdqd19_32*ROjdqd19_22)
    Ompqpjdqd19_23 = Ompqpjdqd19_22+qd[14]*(OMjdqd19_32*C13-ROjdqd19_32*qd[12])
    Ompqpjdqd19_33 = Ompqpjdqd19_32+qd[14]*(-OMjdqd19_22*C13+ROjdqd19_22*qd[12])
    OMjdqd19_14 = OMjdqd19_13+ROjdqd19_73*qd[15]
    OMjdqd19_24 = OMjdqd19_23+ROjdqd19_83*qd[15]
    OMjdqd19_34 = OMjdqd19_33+ROjdqd19_93*qd[15]
    Ompqpjdqd19_14 = Ompqpjdqd19_13+qd[15]*(OMjdqd19_23*ROjdqd19_93-OMjdqd19_33*ROjdqd19_83)
    Ompqpjdqd19_24 = Ompqpjdqd19_23+qd[15]*(-OMjdqd19_13*ROjdqd19_93+OMjdqd19_33*ROjdqd19_73)
    Ompqpjdqd19_34 = Ompqpjdqd19_33+qd[15]*(OMjdqd19_13*ROjdqd19_83-OMjdqd19_23*ROjdqd19_73)
    RLjdqd19_15 = ROjdqd19_14*s.dpt[1,26]+ROjdqd19_73*s.dpt[3,26]
    RLjdqd19_25 = ROjdqd19_24*s.dpt[1,26]+ROjdqd19_83*s.dpt[3,26]
    RLjdqd19_35 = ROjdqd19_34*s.dpt[1,26]+ROjdqd19_93*s.dpt[3,26]
    ORjdqd19_15 = OMjdqd19_24*RLjdqd19_35-OMjdqd19_34*RLjdqd19_25
    ORjdqd19_25 = -OMjdqd19_14*RLjdqd19_35+OMjdqd19_34*RLjdqd19_15
    ORjdqd19_35 = OMjdqd19_14*RLjdqd19_25-OMjdqd19_24*RLjdqd19_15
    Apqpjdqd19_15 = OMjdqd19_24*ORjdqd19_35-OMjdqd19_34*ORjdqd19_25+Ompqpjdqd19_24*RLjdqd19_35-Ompqpjdqd19_34* \
 	  RLjdqd19_25
    Apqpjdqd19_25 = Apqpjdqd19_22-OMjdqd19_14*ORjdqd19_35+OMjdqd19_34*ORjdqd19_15-Ompqpjdqd19_14*RLjdqd19_35+ \
 	  Ompqpjdqd19_34*RLjdqd19_15
    Apqpjdqd19_35 = Apqpjdqd19_32+OMjdqd19_14*ORjdqd19_25-OMjdqd19_24*ORjdqd19_15+Ompqpjdqd19_14*RLjdqd19_25- \
 	  Ompqpjdqd19_24*RLjdqd19_15
    ROjdqd20_23 = C30*S31
    ROjdqd20_33 = S30*S31
    ROjdqd20_53 = C30*C31
    ROjdqd20_63 = S30*C31
    OMjdqd20_23 = -qd[31]*S30
    OMjdqd20_33 = qd[31]*C30
    Ompqpjdqd20_23 = -qd[30]*qd[31]*C30
    Ompqpjdqd20_33 = -qd[30]*qd[31]*S30
    RLjdqd20_14 = s.dpt[1,44]*C31-s.dpt[2,44]*S31
    RLjdqd20_24 = ROjdqd20_23*s.dpt[1,44]+ROjdqd20_53*s.dpt[2,44]
    RLjdqd20_34 = ROjdqd20_33*s.dpt[1,44]+ROjdqd20_63*s.dpt[2,44]
    ORjdqd20_14 = OMjdqd20_23*RLjdqd20_34-OMjdqd20_33*RLjdqd20_24
    ORjdqd20_24 = OMjdqd20_33*RLjdqd20_14-RLjdqd20_34*qd[30]
    ORjdqd20_34 = -OMjdqd20_23*RLjdqd20_14+RLjdqd20_24*qd[30]
    Apqpjdqd20_14 = OMjdqd20_23*ORjdqd20_34-OMjdqd20_33*ORjdqd20_24+Ompqpjdqd20_23*RLjdqd20_34-Ompqpjdqd20_33* \
 	  RLjdqd20_24
    Apqpjdqd20_24 = OMjdqd20_33*ORjdqd20_14-ORjdqd20_34*qd[30]+Ompqpjdqd20_33*RLjdqd20_14
    Apqpjdqd20_34 = -OMjdqd20_23*ORjdqd20_14+ORjdqd20_24*qd[30]-Ompqpjdqd20_23*RLjdqd20_14
    jdqd28 = Apqpjdqd19_15-Apqpjdqd20_14
    jdqd29 = Apqpjdqd19_25-Apqpjdqd20_24
    jdqd30 = Apqpjdqd19_35-Apqpjdqd20_34
    Jdqd[1] = Apqpjdqd1_15
    Jdqd[2] = jdqd2
    Jdqd[3] = jdqd3
    Jdqd[4] = Apqpjdqd3_15
    Jdqd[5] = jdqd5
    Jdqd[6] = jdqd6
    Jdqd[7] = jdqd8
    Jdqd[8] = jdqd9
    Jdqd[9] = jdqd11
    Jdqd[10] = jdqd12
    Jdqd[11] = Apqpjdqd9_14
    Jdqd[12] = jdqd14
    Jdqd[13] = jdqd15
    Jdqd[14] = Apqpjdqd11_15
    Jdqd[15] = jdqd17
    Jdqd[16] = jdqd18
    Jdqd[17] = -Apqpjdqd14_14
    Jdqd[18] = jdqd20
    Jdqd[19] = jdqd21
    Jdqd[20] = Apqpjdqd15_15
    Jdqd[21] = jdqd23
    Jdqd[22] = jdqd24
    Jdqd[23] = jdqd25
    Jdqd[24] = jdqd26
    Jdqd[25] = jdqd27
    Jdqd[26] = jdqd28
    Jdqd[27] = jdqd29
    Jdqd[28] = jdqd30

# Number of continuation lines = 1


