data = [1.9616, 1.90037, 2.02842, 2.18310]
changes = [0.1, 0.1, 0.01, 0.01]
ori = 1.90537
gradient = [(dat-ori)/change for (dat,change) in zip(data, changes)]
print zip(data, changes)

print gradient