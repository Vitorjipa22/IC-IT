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


def ISE(pid, particula = None, ma = False):
  # pid = Pid(pid[0],pid[1],set_point = pid[2])

  if type(particula) == list:
    if ma:
      _, erro, _ = pid.resposta_MA()
      erro = sum((erro ** 2)*0.001) 
      return erro

    else:
      _, _, _, erro, _, _= pid.resposta_MF(particula[0],particula[1],particula[2])

      return erro


  else:
    if ma:
      _, erro, _ = pid.resposta_MA()
      erro = sum((erro ** 2)*0.001) 
      return erro

    else:
      _, _, _, erro, _, _= pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])
      particula.erro = erro
   
      return particula


def IAE(particula, pid):
  # pid = Pid(pid[0],pid[1],set_point = pid[2])

  if type(particula) == list:
    _, _, _, _, erro, _= pid.resposta_MF(particula[0],particula[1],particula[2])

    return erro


  else:
    _, _, _, _, erro, _= pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])
    particula.erro = erro
   
    return particula


def ITSE(particula, pid):
  # pid = Pid(pid[0],pid[1],set_point = pid[2])

  if type(particula) == list:
    _, _, erro, _, _, _= pid.resposta_MF(particula[0],particula[1],particula[2])

    return erro

  else:
    _, _, erro, _, _, _= pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])
    particula.erro = erro
   
    return particula


def ITAE(particula, pid):
  # pid = Pid(pid[0],pid[1],set_point = pid[2])

  if type(particula) == list:
    _, erro, _, _, _, _= pid.resposta_MF(particula[0],particula[1],particula[2])

    return erro

  else:
    _, erro, _, _, _, _= pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])
    particula.erro = erro
   
    return particula
  

def Tempo_Acomodacao( pid, particula = None, ma = False):
  last_point = 0

  if ma:
    Y,_, T = pid.resposta_MA()
    
  else:
    if type(particula) == list: 
      Y, _, _, _, _, T = pid.resposta_MF(particula[0],particula[1],particula[2])
    else:
      Y, _, _, _, _, T = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])

  for i in range(len(Y)):
    if Y[i] >= (Y[-1] + 0.02) or Y[i] <= (Y[-1] - 0.02):
      last_point = i

  temp_acom = T[last_point]
  
  return temp_acom


def Tempo_Subida(particula,pid):  
  Y, _, _, _, _, T = pid.resposta_MF(particula[0],particula[1],particula[2])
  aux1, aux2 = False, False

  for i in range(len(Y)):
    if Y[i] > 0.05*Y[-1] and aux1 == False:
      point5 = i-1
      aux1 = True

    elif Y[i] > 0.95*Y[-1] and aux2 == False:
      point95 = i-1
      aux2 = True

  return T[point5],T[point95]
  
  
def Overshoot(pid, particula = None, ma = False):  
  # pid = Pid(pid[0],pid[1],set_point = pid[2])

  if ma:
    Y, _, _ = pid.resposta_MA()

  else:
    if type(particula) == list:
      try:
        Y, _, _, _, _, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
      
      except:
        particula = [round(i) for i in particula]
    
    else:
      Y, _, _, _, _, _ = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])

  overshoot = np.max(Y) if np.max(Y) > 1 else 1

  return overshoot


def Multi_erro(particula, pid):

  primeira_parcela = (Overshoot(pid, particula = particula) - 1)
  # print('primeira_parcela (overshooting): ',primeira_parcela)

  segunda_parcela = (Tempo_Acomodacao(pid, particula=particula))
  # print('segunda_parcela (acomodação)',segunda_parcela)

  quarta_parcela = abs(erro_novo(pid, particula))

  if type(particula) == list:
    ise = ISE(pid, particula=particula)
    terceira_parcela = ise / (pid.get_ISE_MA())
    quinta_parcela = ITSE(particula, pid)
    # print('Terceira parcela (ISE)',terceira_parcela)
    # print('quarta parcela (ultimo erro)',quarta_parcela)
    # print('quinta parcela (ITSE)',quinta_parcela)

    erro = (primeira_parcela + segunda_parcela + terceira_parcela + quarta_parcela + quinta_parcela)/5

    return erro
  
  else:
    ise = ISE(pid, particula=particula).erro
    quinta_parcela = ITSE(particula, pid).erro
    # print('Terceira parcela (ISE)',terceira_parcela)
    # print('quarta parcela (ultimo erro)',quarta_parcela)
    # print('quinta parcela (ITSE)',quinta_parcela)

    terceira_parcela = ise / (pid.get_ISE_MA())
    erro = (primeira_parcela + segunda_parcela + terceira_parcela)/3
    
    particula.erro = erro

    return particula


def erro_novo(pid, particula):

  if type(particula) == list:
    try:
      Y, _, _, _, _, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
    
    except:
      print('erro no erro_novo')
  
  else:
    Y, _, _, _, _, _ = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])

  erro_novo = abs(Y[-1]-1)

  return erro_novo

if __name__ == "__main__":
  tf = [16,[1,4,16]]
  particula = [10, 2 ,0.05]

  # out = erros(tf,particula,20)
  pid = Pid(num=16, den = [1, 4, 16])

  ise = ISE(pid = pid, particula=particula)
  print(ise)






