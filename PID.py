import control as ctl
import matlab
import matlab.engine
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import time

class PID:
  # def __init__(self, num, den, gama = 1/8, temp_simu = 20, set_point = 1.0, amostragem = 0.001):
  def __init__(self, num, den):
    self.temp_simu = 20
    self.gama = 1/8
    self.amostragem = 0.001
    self.set_point = 1.0
    self.pontos_simu = np.arange(0,self.temp_simu+0.001,self.amostragem)

    self.numerador = num
    self.denominador = den

    self.G_s = ctl.tf(self.numerador, self.denominador)
    self.H_s = ctl.tf([1.],[1.])

    self.T_ma, self.yout_ma = ctl.step_response(self.G_s, self.pontos_simu)
    self.error = [self.set_point]*len(self.T_ma) - self.yout_ma

    self.temp = 20

    self.eng = matlab.engine.start_matlab()
    
  def resposta_MF(self,P,I,D):
    timeini = time.time()
    self.P = str(P)
    self.I = str(I)
    self.D = str(D)

    self.eng.eval('Ti =' + self.I + ';',nargout=0)
    self.eng.eval('Td =' + self.D + ';',nargout=0)
    self.eng.eval('Kp =' + self.P + ';',nargout=0)
    self.eng.eval('gama = 1/8;',nargout=0)
    self.eng.eval('num =' + str(self.numerador) + ';',nargout=0)
    self.eng.eval('den =' + str(self.denominador) + ';',nargout=0)
    self.eng.eval('Temp =' + str(self.temp) + ';',nargout=0)

    self.eng.GetRealTimeData(nargout = 0)

    self.time_sys = np.arange(0,20.001,0.001)

    self.out = self.eng.workspace['out']
    self.out = np.array(self.out)

    self.ITAE = self.out[:,0]
    self.ITAE = self.ITAE[-1]

    self.ITSE = self.out[:,1]
    self.ITSE = self.ITSE[-1]

    self.ISE = self.out[:,2]
    self.ISE = self.ISE[-1]

    self.IAE = self.out[:,3]
    self.IAE = self.IAE[-1]

    self.Y = self.out[:,4]
    timefim = time.time()
    # print(timefim-timeini)

    return self.Y, self.ITAE, self.ITSE, self.ISE, self.IAE, self.time_sys
  
  def resposta_MA(self):

    return self.yout_ma, self.error, self.T_ma

  def plot_MF(self, Y, erro, T, temp_plot = 20, figsize = (15,7), set_point = False, fim = False):

    
    pontos_pl = int((temp_plot * len(self.pontos_simu))/(self.temp_simu))

    Y = Y[:pontos_pl]
    T = T[:pontos_pl]

    plt.figure(figsize = figsize)
    plt.plot(T, Y, linewidth = 1.2,label = 'Saida do controlador')

    if temp_plot > erro[3]:
      plt.vlines(x = erro[3], ymin = 0, ymax = max(Y), colors = 'purple', label = 'tempo de acomodação', linewidth = 1.2, linestyle = ':')

    if temp_plot > erro[4]:
      plt.vlines(x = erro[4], ymin = 0, ymax = max(Y), colors = 'black', label = f'5% do setpoint', linewidth = 1.2, linestyle = ':') 

    if temp_plot > erro[5]:
      plt.vlines(x = erro[5], ymin = 0, ymax = max(Y), colors = 'black', label = f'95% do setpoint', linewidth = 1.2, linestyle = ':')

    plt.axhline(y = Y[-1] + 0.02, color = 'b', linestyle = ':', label = 'Margens do tempo de acomodação')
    plt.axhline(y = Y[-1] - 0.02, color = 'b', linestyle = ':')
    plt.axhline(y = erro[6], color = 'yellow', linestyle = ':', label = 'Overshoot')

    plt.grid(True,linewidth=0.4)
    plt.title('Controlador PID')
    
    plt.text((pontos_pl*1.3)/len(self.pontos_simu),-0.045,f'P:{float(self.P):,.3f}      I:{float(self.I):,.3f}      D:{float(self.D):,.3f}      ERRO:{erro[0]:,.5f}      IAE {erro[1]:,.5f}      ITSE {erro[2]:,.5f}',fontsize = 12, fontname = 'monospace', color = '#3F9C6B')

    if set_point:
      plt.plot(self.T_mf, [self.set_point]*len(self.T_mf), linewidth = 1.2, label = 'Setpoint')

    plt.legend(loc = 'center right')
    if fim:
      plt.savefig('gbest.png')
    plt.show()

  def plot_MA(self, Y, T, erro):
    
    temp_plot = 20

    pontos_pl = int((temp_plot * len(self.pontos_simu))/(self.temp_simu))

    Y = Y[:pontos_pl]
    T = T[:pontos_pl]

    plt.figure(figsize = (15,7))
    plt.plot(T, Y, linewidth = 1.2,label = 'Saida do controlador')

    if temp_plot > erro[0]:
      plt.vlines(x = erro[0], ymin = 0, ymax = max(Y), colors = 'purple', label = 'tempo de acomodação', linewidth = 1.2, linestyle = ':')

    plt.axhline(y = Y[-1] + 0.02, color = 'b', linestyle = ':', label = 'Margens do tempo de acomodação')
    plt.axhline(y = Y[-1] - 0.02, color = 'b', linestyle = ':')
    plt.axhline(y = erro[1], color = 'yellow', linestyle = ':', label = 'Overshoot')

    plt.grid(True,linewidth=0.4)
    plt.title('Controlador PID')

    plt.show()

  def set_ISE_MA(self, ISE_MA):
    self.ISE_MA = ISE_MA

  def get_ISE_MA(self):
    return self.ISE_MA

  def set_TACO_MA(self, TACO_MA):
    self.TACO_MA = TACO_MA

  def get_TACO_MA(self):
    return self.TACO_MA

if __name__ == "__main__":
    num = [16]
    den = [1,4,16]
    set_point = 1.0

    erro = list()
    pid_param = list()

    pid_param.append(num)
    pid_param.append(den)
    pid_param.append(set_point)

    pid = PID(num,den)

    # pid para exemplo de plot em malha fechada
    # Y, _, T = pid.resposta_MF(100,0.1,0.1)
    # erro = [10.365560292797628, 23.986677813973927, 0.12153776843572067, 0.152, 0.002, 0.015, 1.4021863730704907]

    # pid para exemplo de plot em maha aberta

    # Y,E, T = pid.resposta_MA()

    # pid.plot_MA(Y, T, erro=E)
    out, erro, temp = pid.resposta_MF(1.0,1.0,1.0)


