import numpy as np
import matplotlib.pyplot as plt
from PID import PID

def Pid(num, den, set_point):  
  '''
  Esta funçao cria um PID com determinados parametros
  e devolve varios tipos de erro
  '''
 
  #instancia o PID
  pid = PID(num = num,den = den, set_point = set_point)

  return pid


def ISE(particula, pid_param, alpha = 100.0):
  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])

  if type(particula) == list:
    try:
      _, erro, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
      erro = sum(erro ** 2) 
      return erro

    except:
      particula = [round(i) for i in particula]
      _, erro, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
      erro = sum(erro ** 2) 
      return erro

  else:
    _, particula.erro, _ = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])
    particula.erro = sum(particula.erro ** 2) 
   
    return particula


def IAE(particula, pid_param):
  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])

  if type(particula) == list:
    try:
      _, erro, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
      erro = sum(abs(erro))
                 
      return erro

    except:
      particula = [round(i) for i in particula]
      _, erro, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
      erro = sum(abs(erro))
      return erro

  else:
    _, particula.erro, _ = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])
    particula.erro = sum(abs(particula.erro))
    # print(f'particula.erro : {particula.erro:,.2f} posição: {particula.X} pbest {particula.pbest}' )
    return particula


def ITSE(particula, pid_param):

  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])

  if type(particula) == list:
    try:
      _, erro, temp = pid.resposta_MF(particula[0],particula[1],particula[2])
      erro = sum(temp * (erro ** 2))
                 
      return erro

    except:
      particula = [round(i) for i in particula]
      _, erro, temp = pid.resposta_MF(particula[0],particula[1],particula[2])
      erro = sum(temp * (erro ** 2))
      return erro

  else:
    _, particula.erro, temp = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])
    particula.erro = sum(temp * (erro ** 2))
    # print(f'particula.erro : {particula.erro:,.2f} posição: {particula.X} pbest {particula.pbest}' )
    return particula


def Tempo_Acomodacao( pid_param, particula = None, ma = False):
  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])

  if ma:
    Y, T = pid.resposta_MA()
    
  else:
    Y, _, T = pid.resposta_MF(particula[0],particula[1],particula[2])

  for i in range(len(Y)):
    if Y[i] > Y[-1] + 0.02 or Y[i] < Y[-1] - 0.02:
      last_point = i

  temp_acom = T[last_point]
  
  return temp_acom


def Tempo_Subida(particula,pid_param):  
  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])

  Y, _, T = pid.resposta_MF(particula[0],particula[1],particula[2])
  aux1, aux2 = False, False

  for i in range(len(Y)):
    if Y[i] > 0.05 and aux1 == False:
      point5 = i-1
      aux1 = True

    elif Y[i] > 0.95 and aux2 == False:
      point95 = i-1
      aux2 = True

  temp_subida = T[point95] - T[point5]
  # print(T[point5])
  # print(T[point95])

  #return T[point95] - T[point5]
  return T[point5],T[point95]
  
  
def Overshoot(pid_param, particula = None, ma = False):  
  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])

  if ma:
    Y, _ = pid.resposta_MA()

  else:
    if type(particula) == list:
      try:
        Y, _, _ = pid.resposta_MF(particula[0],particula[1],particula[2])
      
      except:
        particula = [round(i) for i in particula]
    
    else:
      Y, _, _ = pid.resposta_MF(particula.X[0],particula.X[1],particula.X[2])

  overshoot = np.max(Y)


  return overshoot

