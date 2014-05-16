import numpy as np
import random
x=[]
y=[]

for i in range(0,100):
    tmp_x = random.random()
    tmp_y = random.random()
    
    x.append([tmp_x, tmp_y])
    if(tmp_x * tmp_x + tmp_y * tmp_y <= 1.0):
        y.append(1)
    else:
        y.append(0)
        
np.savetxt("data_x", np.array(x))
np.savetxt("data_y", np.array(y))
        
