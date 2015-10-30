#GRADIENT 2 - Parameters Below

#h = 3.0 # meters, initial height of f
#vx0 = 7.5 # initial horizontal velocity
#p0 = 0.02
#pd0 = 0.1

data1 = [1.1224, 1.0761, 1.0679, 1.0583]
changes1 = [0.1, 1.0, 0.001, 0.001] 
orig1 = 1.0499
gradient1 = [(dat1-orig1)/change1 for (dat1,change1) in zip(data1, changes1)]
print gradient1

data2 = [0.9539, 0.9656, 0.9594, 0.9575]
changes2 = [0.01, 0.1, 0.001, 0.0001] 
orig2 = 0.9557
gradient2 = [(dat2-orig2)/change2 for (dat2,change2) in zip(data2, changes2)]
print gradient2

data3 = [1.0326, 1.0314, 1.0166, 1.0171]
changes3 = [0.1, 0.1, 0.001, 0.001] 
orig3 = 1.0168
gradient3 = [(dat3-orig3)/change3 for (dat3,change3) in zip(data3, changes3)]
print gradient3

data4 = [1.0121, 1.0098, 1.0125, 1.0114]
changes4 = [0.01, 0.001, 0.001, 0.001] 
orig4 = 1.0097
gradient4 = [(dat4-orig4)/change4 for (dat4,change4) in zip(data4, changes4)]
print gradient4