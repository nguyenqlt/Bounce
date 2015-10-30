#GRADIENT 1 - Parameters Below

#h = 3.0 # meters, initial height of f
#vx0 = 7.0 # initial horizontal velocity
#p0 = 0.02
#pd0 = 0.1

data1 = [1.0591, 1.0390, 1.0246, 1.0312]
changes1 = [0.1, 0.1, 0.001, 0.001] 
orig1 = 1.0286
gradient1 = [(dat1-orig1)/change1 for (dat1,change1) in zip(data1, changes1)]
print gradient1

data2 = [1.0227, 1.0216, 1.0189, 1.0524]
changes2 = [0.01, 0.01, 0.001, 0.01] 
orig2 = 1.0204
gradient2 = [(dat2-orig2)/change2 for (dat2,change2) in zip(data2, changes2)]
print gradient2

data3 = [0.9075, 0.9163, 0.9268, 0.9195]
changes3 = [0.01, 0.1, 0.001, 0.001] 
orig3 = 0.9130
gradient3 = [(dat3-orig3)/change3 for (dat3,change3) in zip(data3, changes3)]
print gradient3

data4 = [0.8156, 0.8227, 0.8111, 0.8196]
changes4 = [0.001, 0.01, 0.001, 0.001] 
orig4 = 0.8210
gradient4 = [(dat4-orig4)/change4 for (dat4,change4) in zip(data4, changes4)]
print gradient4