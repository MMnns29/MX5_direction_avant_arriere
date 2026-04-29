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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
# Trigonometric functions

    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
    S6 = sin(q[6])
    C6 = cos(q[6])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S11 = sin(q[11])
    C11 = cos(q[11])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S16 = sin(q[16])
    C16 = cos(q[16])
    S17 = sin(q[17])
    C17 = cos(q[17])
    S18 = sin(q[18])
    C18 = cos(q[18])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
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
    S28 = sin(q[28])
    C28 = cos(q[28])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S31 = sin(q[31])
    C31 = cos(q[31])
    S32 = sin(q[32])
    C32 = cos(q[32])
    S33 = sin(q[33])
    C33 = cos(q[33])
    S34 = sin(q[34])
    C34 = cos(q[34])
    S35 = sin(q[35])
    C35 = cos(q[35])
    S36 = sin(q[36])
    C36 = cos(q[36])
    S37 = sin(q[37])
    C37 = cos(q[37])
    S38 = sin(q[38])
    C38 = cos(q[38])
    S39 = sin(q[39])
    C39 = cos(q[39])
    S40 = sin(q[40])
    C40 = cos(q[40])
    S41 = sin(q[41])
    C41 = cos(q[41])
    S42 = sin(q[42])
    C42 = cos(q[42])
    S43 = sin(q[43])
    C43 = cos(q[43])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    ALPHA33 = qdd[3]-s.g[3]
    ALPHA14 = qdd[1]*C4+qdd[2]*S4
    ALPHA24 = -qdd[1]*S4+qdd[2]*C4
    OM25 = qd[4]*S5
    OM35 = qd[4]*C5
    OMp25 = qd[4]*qd[5]*C5+qdd[4]*S5
    OMp35 = -qd[4]*qd[5]*S5+qdd[4]*C5
    ALPHA25 = ALPHA24*C5+ALPHA33*S5
    ALPHA35 = -ALPHA24*S5+ALPHA33*C5
    OM16 = qd[5]*C6-OM35*S6
    OM26 = qd[6]+OM25
    OM36 = qd[5]*S6+OM35*C6
    OMp16 = C6*(qdd[5]-qd[6]*OM35)-S6*(OMp35+qd[5]*qd[6])
    OMp26 = qdd[6]+OMp25
    OMp36 = C6*(OMp35+qd[5]*qd[6])+S6*(qdd[5]-qd[6]*OM35)
    BS16 = -OM26*OM26-OM36*OM36
    BS26 = OM16*OM26
    BS36 = OM16*OM36
    BS56 = -OM16*OM16-OM36*OM36
    BS66 = OM26*OM36
    BS96 = -OM16*OM16-OM26*OM26
    BETA26 = BS26-OMp36
    BETA36 = BS36+OMp26
    BETA46 = BS26+OMp36
    BETA66 = BS66-OMp16
    BETA76 = BS36-OMp26
    BETA86 = BS66+OMp16
    ALPHA16 = ALPHA14*C6-ALPHA35*S6
    ALPHA36 = ALPHA14*S6+ALPHA35*C6
    OM17 = qd[7]+OM16
    OM27 = OM26*C7+OM36*S7
    OM37 = -OM26*S7+OM36*C7
    OMp17 = qdd[7]+OMp16
    OMp27 = C7*(OMp26+qd[7]*OM36)+S7*(OMp36-qd[7]*OM26)
    OMp37 = C7*(OMp36-qd[7]*OM26)-S7*(OMp26+qd[7]*OM36)
    BS27 = OM17*OM27
    BS57 = -OM17*OM17-OM37*OM37
    BS67 = OM27*OM37
    BETA27 = BS27-OMp37
    BETA87 = BS67+OMp17
    ALPHA17 = ALPHA16+BETA26*s.dpt[2,1]+BETA36*s.dpt[3,1]+BS16*s.dpt[1,1]
    ALPHA27 = C7*(ALPHA25+BETA46*s.dpt[1,1]+BETA66*s.dpt[3,1]+BS56*s.dpt[2,1])+S7*(ALPHA36+BETA76*s.dpt[1,1]+BETA86* \
 	  s.dpt[2,1]+BS96*s.dpt[3,1])
    ALPHA37 = C7*(ALPHA36+BETA76*s.dpt[1,1]+BETA86*s.dpt[2,1]+BS96*s.dpt[3,1])-S7*(ALPHA25+BETA46*s.dpt[1,1]+BETA66* \
 	  s.dpt[3,1]+BS56*s.dpt[2,1])
    OM18 = OM17*C8-OM37*S8
    OM28 = qd[8]+OM27
    OM38 = OM17*S8+OM37*C8
    OMp18 = C8*(OMp17-qd[8]*OM37)-S8*(OMp37+qd[8]*OM17)
    OMp28 = qdd[8]+OMp27
    OMp38 = C8*(OMp37+qd[8]*OM17)+S8*(OMp17-qd[8]*OM37)
    ALPHA18 = C8*(ALPHA17+BETA27*s.dpt[2,16])-S8*(ALPHA37+BETA87*s.dpt[2,16])
    ALPHA28 = ALPHA27+BS57*s.dpt[2,16]
    ALPHA38 = C8*(ALPHA37+BETA87*s.dpt[2,16])+S8*(ALPHA17+BETA27*s.dpt[2,16])
    OM19 = qd[9]+OM18
    OM29 = OM28*C9+OM38*S9
    OM39 = -OM28*S9+OM38*C9
    OMp19 = qdd[9]+OMp18
    OMp29 = C9*(OMp28+qd[9]*OM38)+S9*(OMp38-qd[9]*OM28)
    OMp39 = C9*(OMp38-qd[9]*OM28)-S9*(OMp28+qd[9]*OM38)
    ALPHA29 = ALPHA28*C9+ALPHA38*S9
    ALPHA39 = -ALPHA28*S9+ALPHA38*C9
    OM110 = OM19*C10+OM29*S10
    OM210 = -OM19*S10+OM29*C10
    OM310 = qd[10]+OM39
    OMp110 = C10*(OMp19+qd[10]*OM29)+S10*(OMp29-qd[10]*OM19)
    OMp210 = C10*(OMp29-qd[10]*OM19)-S10*(OMp19+qd[10]*OM29)
    OMp310 = qdd[10]+OMp39
    BS310 = OM110*OM310
    BS610 = OM210*OM310
    BS910 = -OM110*OM110-OM210*OM210
    BETA310 = BS310+OMp210
    BETA610 = BS610-OMp110
    ALPHA110 = ALPHA18*C10+ALPHA29*S10
    ALPHA210 = -ALPHA18*S10+ALPHA29*C10
    OM111 = OM110*C11-OM310*S11
    OM211 = qd[11]+OM210
    OM311 = OM110*S11+OM310*C11
    OMp111 = C11*(OMp110-qd[11]*OM310)-S11*(OMp310+qd[11]*OM110)
    OMp211 = qdd[11]+OMp210
    OMp311 = C11*(OMp310+qd[11]*OM110)+S11*(OMp110-qd[11]*OM310)
    ALPHA111 = C11*(ALPHA110+BETA310*s.dpt[3,20])-S11*(ALPHA39+BS910*s.dpt[3,20])
    ALPHA211 = ALPHA210+BETA610*s.dpt[3,20]
    ALPHA311 = C11*(ALPHA39+BS910*s.dpt[3,20])+S11*(ALPHA110+BETA310*s.dpt[3,20])
    OM112 = qd[12]+OM16
    OM212 = OM26*C12+OM36*S12
    OM312 = -OM26*S12+OM36*C12
    OMp112 = qdd[12]+OMp16
    OMp212 = C12*(OMp26+qd[12]*OM36)+S12*(OMp36-qd[12]*OM26)
    OMp312 = C12*(OMp36-qd[12]*OM26)-S12*(OMp26+qd[12]*OM36)
    BS212 = OM112*OM212
    BS512 = -OM112*OM112-OM312*OM312
    BS612 = OM212*OM312
    BETA212 = BS212-OMp312
    BETA812 = BS612+OMp112
    ALPHA112 = ALPHA16+BETA26*s.dpt[2,2]+BETA36*s.dpt[3,2]+BS16*s.dpt[1,2]
    ALPHA212 = C12*(ALPHA25+BETA46*s.dpt[1,2]+BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])+S12*(ALPHA36+BETA76*s.dpt[1,2]+ \
 	  BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])
    ALPHA312 = C12*(ALPHA36+BETA76*s.dpt[1,2]+BETA86*s.dpt[2,2]+BS96*s.dpt[3,2])-S12*(ALPHA25+BETA46*s.dpt[1,2]+ \
 	  BETA66*s.dpt[3,2]+BS56*s.dpt[2,2])
    OM113 = OM112*C13-OM312*S13
    OM213 = qd[13]+OM212
    OM313 = OM112*S13+OM312*C13
    OMp113 = C13*(OMp112-qd[13]*OM312)-S13*(OMp312+qd[13]*OM112)
    OMp213 = qdd[13]+OMp212
    OMp313 = C13*(OMp312+qd[13]*OM112)+S13*(OMp112-qd[13]*OM312)
    ALPHA113 = C13*(ALPHA112+BETA212*s.dpt[2,22])-S13*(ALPHA312+BETA812*s.dpt[2,22])
    ALPHA213 = ALPHA212+BS512*s.dpt[2,22]
    ALPHA313 = C13*(ALPHA312+BETA812*s.dpt[2,22])+S13*(ALPHA112+BETA212*s.dpt[2,22])
    OM114 = qd[14]+OM113
    OM214 = OM213*C14+OM313*S14
    OM314 = -OM213*S14+OM313*C14
    OMp114 = qdd[14]+OMp113
    OMp214 = C14*(OMp213+qd[14]*OM313)+S14*(OMp313-qd[14]*OM213)
    OMp314 = C14*(OMp313-qd[14]*OM213)-S14*(OMp213+qd[14]*OM313)
    ALPHA214 = ALPHA213*C14+ALPHA313*S14
    ALPHA314 = -ALPHA213*S14+ALPHA313*C14
    OM115 = OM114*C15+OM214*S15
    OM215 = -OM114*S15+OM214*C15
    OM315 = qd[15]+OM314
    OMp115 = C15*(OMp114+qd[15]*OM214)+S15*(OMp214-qd[15]*OM114)
    OMp215 = C15*(OMp214-qd[15]*OM114)-S15*(OMp114+qd[15]*OM214)
    OMp315 = qdd[15]+OMp314
    BS315 = OM115*OM315
    BS615 = OM215*OM315
    BS915 = -OM115*OM115-OM215*OM215
    BETA315 = BS315+OMp215
    BETA615 = BS615-OMp115
    ALPHA115 = ALPHA113*C15+ALPHA214*S15
    ALPHA215 = -ALPHA113*S15+ALPHA214*C15
    OM116 = OM115*C16-OM315*S16
    OM216 = qd[16]+OM215
    OM316 = OM115*S16+OM315*C16
    OMp116 = C16*(OMp115-qd[16]*OM315)-S16*(OMp315+qd[16]*OM115)
    OMp216 = qdd[16]+OMp215
    OMp316 = C16*(OMp315+qd[16]*OM115)+S16*(OMp115-qd[16]*OM315)
    ALPHA116 = C16*(ALPHA115+BETA315*s.dpt[3,25])-S16*(ALPHA314+BS915*s.dpt[3,25])
    ALPHA216 = ALPHA215+BETA615*s.dpt[3,25]
    ALPHA316 = C16*(ALPHA314+BS915*s.dpt[3,25])+S16*(ALPHA115+BETA315*s.dpt[3,25])
    ALPHA127 = ALPHA16+q[27]*BETA26-(2.0)*qd[27]*OM36+BETA36*s.dpt[3,9]+BS16*s.dpt[1,9]
    ALPHA227 = qdd[27]+ALPHA25+q[27]*BS56+BETA46*s.dpt[1,9]+BETA66*s.dpt[3,9]
    ALPHA327 = ALPHA36+q[27]*BETA86+(2.0)*qd[27]*OM16+BETA76*s.dpt[1,9]+BS96*s.dpt[3,9]
    OM132 = qd[32]+OM16
    OM232 = OM26*C32+OM36*S32
    OM332 = -OM26*S32+OM36*C32
    OMp132 = qdd[32]+OMp16
    OMp232 = C32*(OMp26+qd[32]*OM36)+S32*(OMp36-qd[32]*OM26)
    OMp332 = C32*(OMp36-qd[32]*OM26)-S32*(OMp26+qd[32]*OM36)
    BS232 = OM132*OM232
    BS532 = -OM132*OM132-OM332*OM332
    BS632 = OM232*OM332
    BETA232 = BS232-OMp332
    BETA832 = BS632+OMp132
    ALPHA132 = ALPHA16+BETA26*s.dpt[2,12]+BETA36*s.dpt[3,12]+BS16*s.dpt[1,12]
    ALPHA232 = C32*(ALPHA25+BETA46*s.dpt[1,12]+BETA66*s.dpt[3,12]+BS56*s.dpt[2,12])+S32*(ALPHA36+BETA76*s.dpt[1,12]+ \
 	  BETA86*s.dpt[2,12]+BS96*s.dpt[3,12])
    ALPHA332 = C32*(ALPHA36+BETA76*s.dpt[1,12]+BETA86*s.dpt[2,12]+BS96*s.dpt[3,12])-S32*(ALPHA25+BETA46*s.dpt[1,12]+ \
 	  BETA66*s.dpt[3,12]+BS56*s.dpt[2,12])
    OM133 = qd[33]+OM132
    OM233 = OM232*C33+OM332*S33
    OM333 = -OM232*S33+OM332*C33
    OMp133 = qdd[33]+OMp132
    OMp233 = C33*(OMp232+qd[33]*OM332)+S33*(OMp332-qd[33]*OM232)
    OMp333 = C33*(OMp332-qd[33]*OM232)-S33*(OMp232+qd[33]*OM332)
    BS333 = OM133*OM333
    BS633 = OM233*OM333
    BS933 = -OM133*OM133-OM233*OM233
    BETA333 = BS333+OMp233
    BETA633 = BS633-OMp133
    ALPHA133 = ALPHA132+BETA232*s.dpt[2,45]
    ALPHA233 = C33*(ALPHA232+BS532*s.dpt[2,45])+S33*(ALPHA332+BETA832*s.dpt[2,45])
    ALPHA333 = C33*(ALPHA332+BETA832*s.dpt[2,45])-S33*(ALPHA232+BS532*s.dpt[2,45])
    OM134 = OM133*C34-OM333*S34
    OM234 = qd[34]+OM233
    OM334 = OM133*S34+OM333*C34
    OMp134 = C34*(OMp133-qd[34]*OM333)-S34*(OMp333+qd[34]*OM133)
    OMp234 = qdd[34]+OMp233
    OMp334 = C34*(OMp333+qd[34]*OM133)+S34*(OMp133-qd[34]*OM333)
    ALPHA134 = C34*(ALPHA133+BETA333*s.dpt[3,48])-S34*(ALPHA333+BS933*s.dpt[3,48])
    ALPHA234 = ALPHA233+BETA633*s.dpt[3,48]
    ALPHA334 = C34*(ALPHA333+BS933*s.dpt[3,48])+S34*(ALPHA133+BETA333*s.dpt[3,48])
    OM135 = qd[35]+OM16
    OM235 = OM26*C35+OM36*S35
    OM335 = -OM26*S35+OM36*C35
    OMp135 = qdd[35]+OMp16
    OMp235 = C35*(OMp26+qd[35]*OM36)+S35*(OMp36-qd[35]*OM26)
    OMp335 = C35*(OMp36-qd[35]*OM26)-S35*(OMp26+qd[35]*OM36)
    BS235 = OM135*OM235
    BS535 = -OM135*OM135-OM335*OM335
    BS635 = OM235*OM335
    BETA235 = BS235-OMp335
    BETA835 = BS635+OMp135
    ALPHA135 = ALPHA16+BETA26*s.dpt[2,13]+BETA36*s.dpt[3,13]+BS16*s.dpt[1,13]
    ALPHA235 = C35*(ALPHA25+BETA46*s.dpt[1,13]+BETA66*s.dpt[3,13]+BS56*s.dpt[2,13])+S35*(ALPHA36+BETA76*s.dpt[1,13]+ \
 	  BETA86*s.dpt[2,13]+BS96*s.dpt[3,13])
    ALPHA335 = C35*(ALPHA36+BETA76*s.dpt[1,13]+BETA86*s.dpt[2,13]+BS96*s.dpt[3,13])-S35*(ALPHA25+BETA46*s.dpt[1,13]+ \
 	  BETA66*s.dpt[3,13]+BS56*s.dpt[2,13])
    OM136 = qd[36]+OM135
    OM236 = OM235*C36+OM335*S36
    OM336 = -OM235*S36+OM335*C36
    OMp136 = qdd[36]+OMp135
    OMp236 = C36*(OMp235+qd[36]*OM335)+S36*(OMp335-qd[36]*OM235)
    OMp336 = C36*(OMp335-qd[36]*OM235)-S36*(OMp235+qd[36]*OM335)
    BS336 = OM136*OM336
    BS636 = OM236*OM336
    BS936 = -OM136*OM136-OM236*OM236
    BETA336 = BS336+OMp236
    BETA636 = BS636-OMp136
    ALPHA136 = ALPHA135+BETA235*s.dpt[2,50]
    ALPHA236 = C36*(ALPHA235+BS535*s.dpt[2,50])+S36*(ALPHA335+BETA835*s.dpt[2,50])
    ALPHA336 = C36*(ALPHA335+BETA835*s.dpt[2,50])-S36*(ALPHA235+BS535*s.dpt[2,50])
    OM137 = OM136*C37-OM336*S37
    OM237 = qd[37]+OM236
    OM337 = OM136*S37+OM336*C37
    OMp137 = C37*(OMp136-qd[37]*OM336)-S37*(OMp336+qd[37]*OM136)
    OMp237 = qdd[37]+OMp236
    OMp337 = C37*(OMp336+qd[37]*OM136)+S37*(OMp136-qd[37]*OM336)
    ALPHA137 = C37*(ALPHA136+BETA336*s.dpt[3,52])-S37*(ALPHA336+BS936*s.dpt[3,52])
    ALPHA237 = ALPHA236+BETA636*s.dpt[3,52]
    ALPHA337 = C37*(ALPHA336+BS936*s.dpt[3,52])+S37*(ALPHA136+BETA336*s.dpt[3,52])
 
# Backward Dynamics

    Fq142 = -s.frc[1,43]*C43-s.frc[3,43]*S43
    Fq342 = s.frc[1,43]*S43-s.frc[3,43]*C43
    Cq142 = -s.trq[1,43]*C43-s.trq[3,43]*S43
    Cq342 = s.trq[1,43]*S43-s.trq[3,43]*C43
    Fq140 = -s.frc[1,41]*C41-s.frc[3,41]*S41
    Fq340 = s.frc[1,41]*S41-s.frc[3,41]*C41
    Cq140 = -s.trq[1,41]*C41-s.trq[3,41]*S41
    Cq340 = s.trq[1,41]*S41-s.trq[3,41]*C41
    Fq139 = -s.frc[1,39]+Fq140
    Fq239 = -s.frc[2,39]-s.frc[2,41]*C40-Fq340*S40
    Fq339 = -s.frc[3,39]-s.frc[2,41]*S40+Fq340*C40
    Cq139 = -s.trq[1,39]+Cq140+s.dpt[2,57]*(-s.frc[2,41]*S40+Fq340*C40)
    Cq239 = -s.trq[2,39]-s.trq[2,41]*C40-Cq340*S40-s.dpt[1,57]*(-s.frc[2,41]*S40+Fq340*C40)
    Cq339 = -s.trq[3,39]-s.trq[2,41]*S40+Cq340*C40-Fq140*s.dpt[2,57]+s.dpt[1,57]*(-s.frc[2,41]*C40-Fq340*S40)
    Fq138 = -s.frc[1,38]+Fq142+Fq139*C39+Fq339*S39
    Fq238 = -s.frc[2,38]+Fq239-s.frc[2,43]*C42-Fq342*S42
    Fq338 = -s.frc[3,38]-s.frc[2,43]*S42-Fq139*S39+Fq339*C39+Fq342*C42
    Cq138 = -s.trq[1,38]+Cq142+Cq139*C39+Cq339*S39+s.dpt[2,56]*(-s.frc[2,43]*S42+Fq342*C42)
    Cq238 = -s.trq[2,38]+Cq239-s.trq[2,43]*C42-Cq342*S42-s.dpt[1,56]*(-s.frc[2,43]*S42+Fq342*C42)
    Cq338 = -s.trq[3,38]-s.trq[2,43]*S42-Cq139*S39+Cq339*C39+Cq342*C42-Fq142*s.dpt[2,56]+s.dpt[1,56]*(-s.frc[2,43]* \
 	  C42-Fq342*S42)
    Fs137 = -s.frc[1,37]+s.m[37]*ALPHA137
    Fs237 = -s.frc[2,37]+s.m[37]*ALPHA237
    Fs337 = -s.frc[3,37]+s.m[37]*ALPHA337
    Cq137 = -s.trq[1,37]+s.In[1,37]*OMp137-s.In[5,37]*OM237*OM337+s.In[9,37]*OM237*OM337
    Cq237 = -s.trq[2,37]+s.In[1,37]*OM137*OM337+s.In[5,37]*OMp237-s.In[9,37]*OM137*OM337
    Cq337 = -s.trq[3,37]-s.In[1,37]*OM137*OM237+s.In[5,37]*OM137*OM237+s.In[9,37]*OMp337
    Fs136 = -s.frc[1,36]+s.m[36]*ALPHA136
    Fs236 = -s.frc[2,36]+s.m[36]*ALPHA236
    Fs336 = -s.frc[3,36]+s.m[36]*ALPHA336
    Fq136 = Fs136+Fs137*C37+Fs337*S37
    Fq236 = Fs236+Fs237
    Fq336 = Fs336-Fs137*S37+Fs337*C37
    Cq136 = -s.trq[1,36]+Cq137*C37+Cq337*S37-Fs237*s.dpt[3,52]
    Cq236 = -s.trq[2,36]+Cq237+s.dpt[3,52]*(Fs137*C37+Fs337*S37)
    Cq336 = -s.trq[3,36]-Cq137*S37+Cq337*C37
    Fq135 = -s.frc[1,35]+Fq136
    Fq235 = -s.frc[2,35]+Fq236*C36-Fq336*S36
    Fq335 = -s.frc[3,35]+Fq236*S36+Fq336*C36
    Cq135 = -s.trq[1,35]+Cq136+s.dpt[2,50]*(Fq236*S36+Fq336*C36)
    Cq235 = -s.trq[2,35]+Cq236*C36-Cq336*S36
    Cq335 = -s.trq[3,35]+Cq236*S36+Cq336*C36-Fq136*s.dpt[2,50]
    Fs134 = -s.frc[1,34]+s.m[34]*ALPHA134
    Fs234 = -s.frc[2,34]+s.m[34]*ALPHA234
    Fs334 = -s.frc[3,34]+s.m[34]*ALPHA334
    Cq134 = -s.trq[1,34]+s.In[1,34]*OMp134-s.In[5,34]*OM234*OM334+s.In[9,34]*OM234*OM334
    Cq234 = -s.trq[2,34]+s.In[1,34]*OM134*OM334+s.In[5,34]*OMp234-s.In[9,34]*OM134*OM334
    Cq334 = -s.trq[3,34]-s.In[1,34]*OM134*OM234+s.In[5,34]*OM134*OM234+s.In[9,34]*OMp334
    Fs133 = -s.frc[1,33]+s.m[33]*ALPHA133
    Fs233 = -s.frc[2,33]+s.m[33]*ALPHA233
    Fs333 = -s.frc[3,33]+s.m[33]*ALPHA333
    Fq133 = Fs133+Fs134*C34+Fs334*S34
    Fq233 = Fs233+Fs234
    Fq333 = Fs333-Fs134*S34+Fs334*C34
    Cq133 = -s.trq[1,33]+Cq134*C34+Cq334*S34-Fs234*s.dpt[3,48]
    Cq233 = -s.trq[2,33]+Cq234+s.dpt[3,48]*(Fs134*C34+Fs334*S34)
    Cq333 = -s.trq[3,33]-Cq134*S34+Cq334*C34
    Fq132 = -s.frc[1,32]+Fq133
    Fq232 = -s.frc[2,32]+Fq233*C33-Fq333*S33
    Fq332 = -s.frc[3,32]+Fq233*S33+Fq333*C33
    Cq132 = -s.trq[1,32]+Cq133+s.dpt[2,45]*(Fq233*S33+Fq333*C33)
    Cq232 = -s.trq[2,32]+Cq233*C33-Cq333*S33
    Cq332 = -s.trq[3,32]+Cq233*S33+Cq333*C33-Fq133*s.dpt[2,45]
    Fq130 = -s.frc[1,31]*C31+s.frc[2,31]*S31
    Fq230 = -s.frc[1,31]*S31-s.frc[2,31]*C31
    Cq130 = -s.trq[1,31]*C31+s.trq[2,31]*S31
    Cq230 = -s.trq[1,31]*S31-s.trq[2,31]*C31
    Fq128 = -s.frc[1,29]*C29+s.frc[2,29]*S29
    Fq228 = -s.frc[1,29]*S29-s.frc[2,29]*C29
    Cq128 = -s.trq[1,29]*C29+s.trq[2,29]*S29
    Cq228 = -s.trq[1,29]*S29-s.trq[2,29]*C29
    Fs127 = -s.frc[1,27]+s.m[27]*ALPHA127
    Fs227 = -s.frc[2,27]+s.m[27]*ALPHA227
    Fs327 = -s.frc[3,27]+s.m[27]*ALPHA327
    Fq127 = Fq128+Fq130+Fs127
    Fq227 = Fs227+s.frc[3,29]*S28+s.frc[3,31]*S30+Fq228*C28+Fq230*C30
    Fq327 = Fs327-s.frc[3,29]*C28-s.frc[3,31]*C30+Fq228*S28+Fq230*S30
    Cq127 = -s.trq[1,27]+Cq128+Cq130+s.dpt[2,41]*(-s.frc[3,29]*C28+Fq228*S28)+s.dpt[2,42]*(-s.frc[3,31]*C30+Fq230*S30 \
 	  )
    Cq227 = -s.trq[2,27]+s.trq[3,29]*S28+s.trq[3,31]*S30+Cq228*C28+Cq230*C30
    Cq327 = -s.trq[3,27]-s.trq[3,29]*C28-s.trq[3,31]*C30+Cq228*S28+Cq230*S30-Fq128*s.dpt[2,41]-Fq130*s.dpt[2,42]
    Fq125 = -s.frc[1,26]*C26-s.frc[3,26]*S26
    Fq325 = s.frc[1,26]*S26-s.frc[3,26]*C26
    Cq125 = -s.trq[1,26]*C26-s.trq[3,26]*S26
    Cq325 = s.trq[1,26]*S26-s.trq[3,26]*C26
    Fq124 = -s.frc[1,24]+Fq125
    Fq224 = -s.frc[2,24]-s.frc[2,26]*C25-Fq325*S25
    Fq324 = -s.frc[3,24]-s.frc[2,26]*S25+Fq325*C25
    Cq124 = -s.trq[1,24]+Cq125+s.dpt[2,39]*(-s.frc[2,26]*S25+Fq325*C25)
    Cq224 = -s.trq[2,24]-s.trq[2,26]*C25-Cq325*S25-s.dpt[1,39]*(-s.frc[2,26]*S25+Fq325*C25)
    Cq324 = -s.trq[3,24]-s.trq[2,26]*S25+Cq325*C25-Fq125*s.dpt[2,39]+s.dpt[1,39]*(-s.frc[2,26]*C25-Fq325*S25)
    Fq122 = -s.frc[1,23]*C23-s.frc[3,23]*S23
    Fq322 = s.frc[1,23]*S23-s.frc[3,23]*C23
    Cq122 = -s.trq[1,23]*C23-s.trq[3,23]*S23
    Cq322 = s.trq[1,23]*S23-s.trq[3,23]*C23
    Fq121 = -s.frc[1,21]+Fq122+Fq124*C24+Fq324*S24
    Fq221 = -s.frc[2,21]+Fq224-s.frc[2,23]*C22-Fq322*S22
    Fq321 = -s.frc[3,21]-s.frc[2,23]*S22-Fq124*S24+Fq322*C22+Fq324*C24
    Cq121 = -s.trq[1,21]+Cq122+Cq124*C24+Cq324*S24+s.dpt[2,36]*(-s.frc[2,23]*S22+Fq322*C22)
    Cq221 = -s.trq[2,21]+Cq224-s.trq[2,23]*C22-Cq322*S22-s.dpt[1,36]*(-s.frc[2,23]*S22+Fq322*C22)
    Cq321 = -s.trq[3,21]-s.trq[2,23]*S22-Cq124*S24+Cq322*C22+Cq324*C24-Fq122*s.dpt[2,36]+s.dpt[1,36]*(-s.frc[2,23]* \
 	  C22-Fq322*S22)
    Fs116 = -s.frc[1,16]+s.m[16]*ALPHA116
    Fs216 = -s.frc[2,16]+s.m[16]*ALPHA216
    Fs316 = -s.frc[3,16]+s.m[16]*ALPHA316
    Cq116 = -s.trq[1,16]+s.In[1,16]*OMp116-s.In[5,16]*OM216*OM316+s.In[9,16]*OM216*OM316
    Cq216 = -s.trq[2,16]+s.In[1,16]*OM116*OM316+s.In[5,16]*OMp216-s.In[9,16]*OM116*OM316
    Cq316 = -s.trq[3,16]-s.In[1,16]*OM116*OM216+s.In[5,16]*OM116*OM216+s.In[9,16]*OMp316
    Fs115 = -s.frc[1,15]+s.m[15]*ALPHA115
    Fs215 = -s.frc[2,15]+s.m[15]*ALPHA215
    Fs315 = -s.frc[3,15]+s.m[15]*ALPHA314
    Fq115 = Fs115+Fs116*C16+Fs316*S16
    Fq215 = Fs215+Fs216
    Fq315 = Fs315-Fs116*S16+Fs316*C16
    Cq115 = -s.trq[1,15]+Cq116*C16+Cq316*S16-Fs216*s.dpt[3,25]
    Cq215 = -s.trq[2,15]+Cq216+s.dpt[3,25]*(Fs116*C16+Fs316*S16)
    Cq315 = -s.trq[3,15]-Cq116*S16+Cq316*C16
    Fq114 = Fq115*C15-Fq215*S15
    Fq214 = Fq115*S15+Fq215*C15
    Cq114 = Cq115*C15-Cq215*S15
    Cq214 = Cq115*S15+Cq215*C15
    Fq213 = Fq214*C14-Fq315*S14
    Fq313 = Fq214*S14+Fq315*C14
    Cq213 = Cq214*C14-Cq315*S14
    Cq313 = Cq214*S14+Cq315*C14
    Fq112 = -s.frc[1,12]+Fq114*C13+Fq313*S13
    Fq212 = -s.frc[2,12]+Fq213
    Fq312 = -s.frc[3,12]-Fq114*S13+Fq313*C13
    Cq112 = -s.trq[1,12]+Cq114*C13+Cq313*S13+s.dpt[2,22]*(-Fq114*S13+Fq313*C13)
    Cq212 = -s.trq[2,12]+Cq213
    Cq312 = -s.trq[3,12]-Cq114*S13+Cq313*C13-s.dpt[2,22]*(Fq114*C13+Fq313*S13)
    Fs111 = -s.frc[1,11]+s.m[11]*ALPHA111
    Fs211 = -s.frc[2,11]+s.m[11]*ALPHA211
    Fs311 = -s.frc[3,11]+s.m[11]*ALPHA311
    Cq111 = -s.trq[1,11]+s.In[1,11]*OMp111-s.In[5,11]*OM211*OM311+s.In[9,11]*OM211*OM311
    Cq211 = -s.trq[2,11]+s.In[1,11]*OM111*OM311+s.In[5,11]*OMp211-s.In[9,11]*OM111*OM311
    Cq311 = -s.trq[3,11]-s.In[1,11]*OM111*OM211+s.In[5,11]*OM111*OM211+s.In[9,11]*OMp311
    Fs110 = -s.frc[1,10]+s.m[10]*ALPHA110
    Fs210 = -s.frc[2,10]+s.m[10]*ALPHA210
    Fs310 = -s.frc[3,10]+s.m[10]*ALPHA39
    Fq110 = Fs110+Fs111*C11+Fs311*S11
    Fq210 = Fs210+Fs211
    Fq310 = Fs310-Fs111*S11+Fs311*C11
    Cq110 = -s.trq[1,10]+Cq111*C11+Cq311*S11-Fs211*s.dpt[3,20]
    Cq210 = -s.trq[2,10]+Cq211+s.dpt[3,20]*(Fs111*C11+Fs311*S11)
    Cq310 = -s.trq[3,10]-Cq111*S11+Cq311*C11
    Fq19 = Fq110*C10-Fq210*S10
    Fq29 = Fq110*S10+Fq210*C10
    Cq19 = Cq110*C10-Cq210*S10
    Cq29 = Cq110*S10+Cq210*C10
    Fq28 = Fq29*C9-Fq310*S9
    Fq38 = Fq29*S9+Fq310*C9
    Cq28 = Cq29*C9-Cq310*S9
    Cq38 = Cq29*S9+Cq310*C9
    Fq17 = -s.frc[1,7]+Fq19*C8+Fq38*S8
    Fq27 = -s.frc[2,7]+Fq28
    Fq37 = -s.frc[3,7]-Fq19*S8+Fq38*C8
    Cq17 = -s.trq[1,7]+Cq19*C8+Cq38*S8+s.dpt[2,16]*(-Fq19*S8+Fq38*C8)
    Cq27 = -s.trq[2,7]+Cq28
    Cq37 = -s.trq[3,7]-Cq19*S8+Cq38*C8-s.dpt[2,16]*(Fq19*C8+Fq38*S8)
    Fs16 = -s.frc[1,6]+s.m[6]*(ALPHA16+BETA36*s.l[3,6]+BS16*s.l[1,6])
    Fs26 = -s.frc[2,6]+s.m[6]*(ALPHA25+BETA46*s.l[1,6]+BETA66*s.l[3,6])
    Fs36 = -s.frc[3,6]+s.m[6]*(ALPHA36+BETA76*s.l[1,6]+BS96*s.l[3,6])
    Fq16 = -s.frc[1,17]-s.frc[1,18]-s.frc[1,19]-s.frc[1,20]+Fq112+Fq127+Fq132+Fq135+Fq17+Fs16+Fq121*C21+Fq138*C38+ \
 	  Fq321*S21+Fq338*S38
    Fq26 = Fq221+Fq227+Fq238+Fs26-s.frc[2,17]*C17-s.frc[2,18]*C18-s.frc[2,19]*C19-s.frc[2,20]*C20+s.frc[3,17]*S17+ \
 	  s.frc[3,18]*S18+s.frc[3,19]*S19+s.frc[3,20]*S20+Fq212*C12+Fq232*C32+Fq235*C35+Fq27*C7-Fq312*S12-Fq332*S32-Fq335*S35- \
 	  Fq37*S7
    Fq36 = Fq327+Fs36-s.frc[2,17]*S17-s.frc[2,18]*S18-s.frc[2,19]*S19-s.frc[2,20]*S20-s.frc[3,17]*C17-s.frc[3,18]*C18 \
 	  -s.frc[3,19]*C19-s.frc[3,20]*C20-Fq121*S21-Fq138*S38+Fq212*S12+Fq232*S32+Fq235*S35+Fq27*S7+Fq312*C12+Fq321*C21+Fq332* \
 	  C32+Fq335*C35+Fq338*C38+Fq37*C7
    Cq16 = -s.trq[1,17]-s.trq[1,18]-s.trq[1,19]-s.trq[1,20]-s.trq[1,6]+Cq112+Cq127+Cq132+Cq135+Cq17+q[27]*Fq327+ \
 	  s.In[1,6]*OMp16-s.In[5,6]*OM26*OM36+s.In[9,6]*OM26*OM36+Cq121*C21+Cq138*C38+Cq321*S21+Cq338*S38-Fq221*s.dpt[3,8]-Fq227 \
 	  *s.dpt[3,9]-Fq238*s.dpt[3,15]-Fs26*s.l[3,6]+s.dpt[2,12]*(Fq232*S32+Fq332*C32)+s.dpt[2,13]*(Fq235*S35+Fq335*C35)+ \
 	  s.dpt[2,1]*(Fq27*S7+Fq37*C7)+s.dpt[2,2]*(Fq212*S12+Fq312*C12)+s.dpt[2,3]*(-s.frc[2,17]*S17-s.frc[3,17]*C17)+s.dpt[2,4] \
 	  *(-s.frc[2,18]*S18-s.frc[3,18]*C18)+s.dpt[2,5]*(-s.frc[2,19]*S19-s.frc[3,19]*C19)+s.dpt[2,7]*(-s.frc[2,20]*S20- \
 	  s.frc[3,20]*C20)-s.dpt[3,12]*(Fq232*C32-Fq332*S32)-s.dpt[3,13]*(Fq235*C35-Fq335*S35)-s.dpt[3,1]*(Fq27*C7-Fq37*S7)- \
 	  s.dpt[3,2]*(Fq212*C12-Fq312*S12)
    Cq26 = -s.trq[2,6]+Cq221+Cq227+Cq238+s.In[1,6]*OM16*OM36+s.In[5,6]*OMp26-s.In[9,6]*OM16*OM36-s.trq[2,17]*C17- \
 	  s.trq[2,18]*C18-s.trq[2,19]*C19-s.trq[2,20]*C20+s.trq[3,17]*S17+s.trq[3,18]*S18+s.trq[3,19]*S19+s.trq[3,20]*S20+Cq212* \
 	  C12+Cq232*C32+Cq235*C35+Cq27*C7-Cq312*S12-Cq332*S32-Cq335*S35-Cq37*S7+Fq112*s.dpt[3,2]+Fq127*s.dpt[3,9]+Fq132* \
 	  s.dpt[3,12]+Fq135*s.dpt[3,13]+Fq17*s.dpt[3,1]-Fq327*s.dpt[1,9]+Fs16*s.l[3,6]-Fs36*s.l[1,6]-s.dpt[1,12]*(Fq232*S32+ \
 	  Fq332*C32)-s.dpt[1,13]*(Fq235*S35+Fq335*C35)-s.dpt[1,15]*(-Fq138*S38+Fq338*C38)-s.dpt[1,1]*(Fq27*S7+Fq37*C7)- \
 	  s.dpt[1,2]*(Fq212*S12+Fq312*C12)-s.dpt[1,5]*(-s.frc[2,19]*S19-s.frc[3,19]*C19)-s.dpt[1,7]*(-s.frc[2,20]*S20- \
 	  s.frc[3,20]*C20)-s.dpt[1,8]*(-Fq121*S21+Fq321*C21)+s.dpt[3,15]*(Fq138*C38+Fq338*S38)+s.dpt[3,8]*(Fq121*C21+Fq321*S21)
    Cq36 = -s.trq[3,6]+Cq327-q[27]*Fq127-s.In[1,6]*OM16*OM26+s.In[5,6]*OM16*OM26+s.In[9,6]*OMp36+s.frc[1,17]* \
 	  s.dpt[2,3]+s.frc[1,18]*s.dpt[2,4]+s.frc[1,19]*s.dpt[2,5]+s.frc[1,20]*s.dpt[2,7]-s.trq[2,17]*S17-s.trq[2,18]*S18- \
 	  s.trq[2,19]*S19-s.trq[2,20]*S20-s.trq[3,17]*C17-s.trq[3,18]*C18-s.trq[3,19]*C19-s.trq[3,20]*C20-Cq121*S21-Cq138*S38+ \
 	  Cq212*S12+Cq232*S32+Cq235*S35+Cq27*S7+Cq312*C12+Cq321*C21+Cq332*C32+Cq335*C35+Cq338*C38+Cq37*C7-Fq112*s.dpt[2,2]-Fq132 \
 	  *s.dpt[2,12]-Fq135*s.dpt[2,13]-Fq17*s.dpt[2,1]+Fq221*s.dpt[1,8]+Fq227*s.dpt[1,9]+Fq238*s.dpt[1,15]+Fs26*s.l[1,6]+ \
 	  s.dpt[1,12]*(Fq232*C32-Fq332*S32)+s.dpt[1,13]*(Fq235*C35-Fq335*S35)+s.dpt[1,1]*(Fq27*C7-Fq37*S7)+s.dpt[1,2]*(Fq212*C12 \
 	  -Fq312*S12)+s.dpt[1,5]*(-s.frc[2,19]*C19+s.frc[3,19]*S19)+s.dpt[1,7]*(-s.frc[2,20]*C20+s.frc[3,20]*S20)
    Fq15 = Fq16*C6+Fq36*S6
    Fq35 = -Fq16*S6+Fq36*C6
    Cq15 = Cq16*C6+Cq36*S6
    Cq35 = -Cq16*S6+Cq36*C6
    Fq24 = Fq26*C5-Fq35*S5
    Fq34 = Fq26*S5+Fq35*C5
    Cq34 = Cq26*S5+Cq35*C5
    Fq13 = Fq15*C4-Fq24*S4
    Fq23 = Fq15*S4+Fq24*C4
 
# Symbolic model output

    Qq[1] = Fq13
    Qq[2] = Fq23
    Qq[3] = Fq34
    Qq[4] = Cq34
    Qq[5] = Cq15
    Qq[6] = Cq26
    Qq[7] = Cq17
    Qq[8] = Cq28
    Qq[9] = Cq19
    Qq[10] = Cq310
    Qq[11] = Cq211
    Qq[12] = Cq112
    Qq[13] = Cq213
    Qq[14] = Cq114
    Qq[15] = Cq315
    Qq[16] = Cq216
    Qq[17] = -s.trq[1,17]
    Qq[18] = -s.trq[1,18]
    Qq[19] = -s.trq[1,19]
    Qq[20] = -s.trq[1,20]
    Qq[21] = Cq221
    Qq[22] = Cq122
    Qq[23] = -s.trq[2,23]
    Qq[24] = Cq224
    Qq[25] = Cq125
    Qq[26] = -s.trq[2,26]
    Qq[27] = Fq227
    Qq[28] = Cq128
    Qq[29] = -s.trq[3,29]
    Qq[30] = Cq130
    Qq[31] = -s.trq[3,31]
    Qq[32] = Cq132
    Qq[33] = Cq133
    Qq[34] = Cq234
    Qq[35] = Cq135
    Qq[36] = Cq136
    Qq[37] = Cq237
    Qq[38] = Cq238
    Qq[39] = Cq239
    Qq[40] = Cq140
    Qq[41] = -s.trq[2,41]
    Qq[42] = Cq142
    Qq[43] = -s.trq[2,43]

# Number of continuation lines = 6


