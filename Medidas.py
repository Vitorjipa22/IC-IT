import numpy as np
from PID import PID
import numpy as np


def Pid(num, den):  
  '''
  Esta funçao cria um PID com determinados parametros
  e devolve varios tipos de erro
  '''
 
  #instancia o PID
  pid = PID(num = num,den = den)

  return pid


def Tempo_Acomodacao(Y, T):
  last_point = 0

  for i in range(len(Y)):
    if Y[i] >= (Y[-1] + 0.02) or Y[i] <= (Y[-1] - 0.02):
      last_point = i

  temp_acom = T[last_point]
  
  return temp_acom


def Tempo_Subida(Y, T):  
  aux1,aux2 = False,False

  for i in range(len(Y)):
    if Y[i] > 0.05*Y[-1] and aux1 == False:
      point5 = i-1
      aux1 = True

    elif Y[i] > 0.95*Y[-1] and aux2 == False:
      point95 = i-1
      aux2 = True

  return T[point5],T[point95]
  
  
def Overshoot(Y):  
  # pid = Pid(pid[0],pid[1],set_point = pid[2])

  # else:
  #   if type(particula) == list:
  #     try:
  #       Y, _, _, _, _, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
      
  #     except:
  #       particula = [round(i) for i in particula]
    
  #   else:
  #     Y, _, _, _, _, _ = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])

  overshoot = np.max(Y) if np.max(Y) > 1 else 1

  return overshoot


def Multi_erro(particula, pid):

  if type(particula) == list:
    Y, _, ITSE, ISE, _, T= pid.resposta_MF(particula[0],particula[1],particula[2])

  else:
    Y, _, ITSE, ISE, _, T= pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])

  particula.overshoot = Overshoot(Y)
  primeira_parcela = particula.overshoot - 1

  particula.tsubida[0],particula.tsubida[1] = Tempo_Subida(Y,T)

  # print('primeira_parcela (overshooting): ',primeira_parcela)
  particula.T = T
  particula.y = Y
  segunda_parcela = Tempo_Acomodacao(Y, T)
  particula.taco = segunda_parcela
  # print('segunda_parcela (acomodação)',segunda_parcela)

  quarta_parcela= abs(erro_novo(Y))
  particula.erro_novo = quarta_parcela

  terceira_parcela = ISE
  particula.ISE = ISE

  quinta_parcela = ITSE
  particula.ITSE = ITSE

  erro = (primeira_parcela + segunda_parcela + terceira_parcela + quarta_parcela + quinta_parcela)/5

  if type(particula) == list:
    return erro

  else:
    particula.erro = erro

    return particula


def erro_novo(Y):

  erro_novo = abs(Y[-1]-1)

  return erro_novo

if __name__ == "__main__":
  tf = [16,[1,4,16]]
  particula = [10, 2 ,0.05]

  # out = erros(tf,particula,20)
  pid = Pid(num=16, den = [1, 4, 16])

  ise = ISE(pid = pid, particula=particula)
  print(ise)






