data = [1.26511, 1.25941,1.23217,1.21710]
changes = [0.01, 0.01, 0.01, 0.01]
ori = 1.2564
gradient = [(dat-ori)/change for (dat,change) in zip(data, changes)]
print zip(data, changes)

print gradient