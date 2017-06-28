import numpy as np 

lines = [line.rstrip('\n') for line in open('dqn_log.txt')]
print(lines[0])

# w = open("demand.txt", "w")
# w.write("demand = [ \n")
# for i in range(1, 1601, 1):
# 	o1 = np.random.uniform(-10, 10)
# 	o2 = np.random.uniform(-10, 10)
# 	d1 = np.random.uniform(-10, 10)
# 	d2 = np.random.uniform(-10, 10)
# 	w.write("(%.4f, %.4f, %.4f, %.4f, %.6f, %.6f), \n" % (o1, o2, d1, d2, 1/1600.0, i/1600.0)) 
# w.write("]")
# w.close()

# w = open("demand.txt", "w")
# w.write("demand = [ \n")
# for i in range(1, 101, 1):
# 	r = np.random.rand() 
# 	if r < 0.2:
# 		o1 = np.random.uniform(-2.5, 2.5)
# 		o2 = np.random.uniform(-2.5, 2.5)
# 	elif r < 0.6:
# 		o1 = np.random.triangular(-2.5, -1, 0.5)
# 		o2 = np.random.triangular(-0.5, 1, 2.5)
# 	else:
# 		o1 = np.random.triangular(-0.5, 1, 2.5)
# 		o2 = np.random.triangular(-2.5, -1, 0.5)
# 	r = np.random.rand() 
# 	if r < 0.2:
# 		d1 = np.random.uniform(-2.5, 2.5)
# 		d2 = np.random.uniform(-2.5, 2.5)
# 	elif r < 0.6:
# 		d1 = np.random.triangular(-0.5, 1, 2.5)
# 		d2 = np.random.triangular(-0.5, 1, 2.5)
# 	else:
# 		d1 = np.random.triangular(-2.5, -1, 0.5)
# 		d2 = np.random.triangular(-2.5, -1, 0.5)
# 	w.write("(%.4f, %.4f, %.4f, %.4f, %.2f, %.2f), \n" % (o1, o2, d1, d2, 1/100.0, i/100.0)) 
# w.write("]")
# w.close()


# w = open("demand.txt", "w")
# w.write("demand = [ \n")
# for i in range(1, 101, 1):
# 	r = np.random.rand() 
# 	if r < 0.2:
# 		o1 = np.random.uniform(-2.5, 2.5)
# 		o2 = np.random.uniform(-2.5, 2.5)
# 	elif r < 0.6:
# 		o1 = np.random.triangular(-2.5, -1, 0.5)
# 		o2 = np.random.triangular(-0.5, 1, 2.5)
# 	else:
# 		o1 = np.random.triangular(-0.5, 1, 2.5)
# 		o2 = np.random.triangular(-2.5, -1, 0.5)
# 	d1 = np.sqrt( (o1-1)**2 + (o2-1)**2 )
# 	d2 = np.sqrt( (o1+1)**2 + (o2+1)**2 )
# 	l = np.exp(-d1) / (np.exp(-d1) + np.exp(-d2))
# 	r = np.random.rand() 
# 	if r < l:
# 		d1 = 1
# 		d2 = 1
# 	else:
# 		d1 = -1
# 		d2 = -1
# 	w.write("(%.4f, %.4f, %.4f, %.4f, %.2f, %.2f), \n" % (o1, o2, d1, d2, 1/100.0, i/100.0)) 
# w.write("]")
# w.close()
