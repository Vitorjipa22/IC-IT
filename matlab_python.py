from matplotlib import pylab
import matlab
import matlab.engine
import numpy as np


eng = matlab.engine.start_matlab() # init instance of engine

eng.eval('Temp = 2.625;', nargout=0)
eng.eval('Ti = 33.076575618016115;',nargout=0)
eng.eval('Td = 0.001;',nargout=0)
eng.eval('Kp = 32025.71745690574;',nargout=0)
eng.eval('gama = 1/8;',nargout=0)
eng.eval('num = 16;',nargout=0)
eng.eval('den = [1 4 16];',nargout=0)
eng.GetRealTimeData(nargout = 0)
out = eng.workspace['a']
out = np.array(out)
print(out)
print(type(out))
print(out.shape)
print(out[:,2])

# out = eng.eval('out;',nargout=0)
# print(out)
# print(type(out))


# #Rodando script feito no matlab



