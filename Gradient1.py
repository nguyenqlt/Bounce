data = [1.0591, 1.0390, 1.0246, 1.0312]
changes = [0.1, 0.1, 0.001, 0.001] 
orig = [1.0286]
gradient = ((data-orig)/change for (data,change) in zip(data, changes))
print gradient