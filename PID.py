import control as ctl
import numpy as np
import matplotlib.pyplot as plt

class PID:
  def __init__(self, num, den, gama = 1/8, temp_simu = 20, set_point = 1.0, amostragem = 0.001):

    self.temp_simu = temp_simu
    self.gama = gama
    self.amostragem = amostragem
    self.set_point = set_point
    self.pontos_simu = np.arange(0,temp_simu,self.amostragem)

    self.numerador = num
    self.denominador = den

    self.G_s = ctl.tf(self.numerador, self.denominador)
    self.H_s = ctl.tf([1.],[1.])
    self.T_s=ctl.feedback(self.G_s, self.H_s, sign=-1)

  def controller(self,P,I,D):

    self.P = P
    self.I = I
    self.D = D

    num_ctl = [self.P*((self.gama*self.I*self.D) + (self.I*self.D)), self.P*(self.I+(self.gama*self.D)), self.P]
    den_ctl = [self.I*self.D*self.gama, self.I,0]

    self.C_s = ctl.tf(num_ctl, den_ctl)
    self.G_s = ctl.series(self.C_s, self.G_s)
    self.T_s = ctl.feedback(self.G_s, self.H_s, sign=-1)
    self.T_mf, self.yout_mf = ctl.step_response(self.T_s, self.pontos_simu)

    self.error = [self.set_point]*len(self.T_mf) - self.yout_mf

    return self.yout_mf, self.error, self.T_mf


  def plot(self, Y, erro, temp_plot = 20, figsize = (15,7), set_point = False):
    
    pontos_pl = int((temp_plot * len(self.pontos_simu))/(self.temp_simu))

    Y = Y[:pontos_pl]
    self.T_mf = self.T_mf[:pontos_pl]

    plt.figure(figsize = figsize)
    plt.plot(self.T_mf, Y, linewidth = 1.2,label = 'Saida do controlador')

    if temp_plot > erro[3]:
      plt.vlines(x = erro[3], ymin = 0, ymax = max(Y), colors = 'purple', label = 'tempo de acomodação', linewidth = 1.2, linestyle = ':')

    if temp_plot > erro[4]:
      plt.vlines(x = erro[4], ymin = 0, ymax = max(Y), colors = 'black', label = f'5% do setpoint', linewidth = 1.2, linestyle = ':') 

    if temp_plot > erro[5]:
      plt.vlines(x = erro[5], ymin = 0, ymax = max(Y), colors = 'black', label = f'95% do setpoint', linewidth = 1.2, linestyle = ':')


    plt.axhline(y = 1.02, color = 'b', linestyle = ':', label = 'Margens do tempo de acomodação')
    plt.axhline(y = 0.98, color = 'b', linestyle = ':')
    plt.axhline(y = erro[6], color = 'yellow', linestyle = ':', label = 'Overshoot')
    plt.grid(True,linewidth=0.4)
    plt.title('Controlador PID')
    plt.text((pontos_pl*1.3)/len(self.pontos_simu),-0.045,f'P:{self.P:,.3f}      I:{self.I:,.3f}      D:{self.D:,.3f}      ISE {erro[0]:,.5f}      IAE {erro[1]:,.5f}      ITSE {erro[2]:,.5f}',fontsize = 12, fontname = 'monospace', color = '#3F9C6B')

    if set_point:
      plt.plot(self.T_mf, [self.set_point]*len(self.T_mf), linewidth = 1.2, label = 'Setpoint')

    plt.legend(loc = 'center right')
    plt.show()

if __name__ == "__main__":
    num = [16]
    den = [1,4,16]

    pid = PID(num,den)
    Y, _, tmf = pid.controller(100,0.1,0.1)
    erro = [10.365560292797628, 23.986677813973927, 0.12153776843572067, 0.152, 0.002, 0.015, 1.4021863730704907]
    pid.plot(Y,erro, set_point = True, temp_plot = 0.25)

