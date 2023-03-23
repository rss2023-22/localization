import numpy as np
import matplotlib.pyplot as plt


alpha_hit = 0.74
alpha_short = 0.07
alpha_max = 0.07
alpha_rand = 0.12
sigma_hit = 8.0

# Your sensor table will be a `table_width` x `table_width` np array:
table_width = 201
####################################

def precompute_sensor_model():

    zmax = 200
    sigma = sigma_hit # check this is right? 
    alphaHit = alpha_hit
    alphaShort = alpha_short
    alphaMax = alpha_max
    alphaRand = alpha_rand

    plot = False # if want to plot the probability distribution. Requires matplotlib.pyplot imported as plt
    checksum = False # if want to print sum's by column (to make sure ~1.0 for proper normalization)

    def phit(zk,d):
        if zk>=0 and zk<=zmax:
            return 1.0/((2*np.pi*sigma**2)**(0.5))*np.exp(-1.0*(((zk-d)**2)/(2*sigma**2)))
        return 0

    def pshort(zk,d):
        if zk >= 0 and zk<=d and d != 0:
            return 2.0/d*(1-(zk/d))
        return 0

    def pmax(zk,d): # CHANGE THIS FUNCTION!!!
        if zk == zmax:
            return 1.0
        return 0

    def prand(zk,d):
        if zk>=0 and zk <= zmax:
            return 1/zmax
        return 0

    def getP(zk,d): # everything except hit
        return alphaShort*pshort(zk,d)+alphaMax*pmax(zk,d)+alphaRand*prand(zk,d)
            
    #compute PHIT prior to others, columns are d
    out = []
    for i in range(table_width): # z
        row = []
        for j in range(table_width): # d
            #row.append(alphaHit*phit(i,j))
            row.append(phit(i,j))
        row = np.array(row) 
        # normalize by row
        #totalVals = sum(row)
        #row = row / totalVals
        # add row to out 
        out.append(row)  

    out = np.array(out)  
    out = out/out.sum(axis=0)
    out *= alphaHit
    
    #compute other part of distribution
    for i in range(table_width):
        for j in range(table_width):
            out[i][j] += getP(i,j)

    # normalize out
    out = out/out.sum(axis=0)

    if checksum:
        for i in range(table_width):
            summed = 0
            for j in range(table_width):
                summed += out[j][i]
            print(summed)
    if plot:
        hf = plt.figure()
        ha = hf.add_subplot(111, projection='3d')
        X , Y = np.meshgrid(range(201), range(201))
        ha.plot_surface(X, Y, out)
        plt.show()
    print(out)
    pass


precompute_sensor_model()




