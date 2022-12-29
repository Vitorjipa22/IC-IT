

import random
import math    
import copy    
import numpy as np
import matplotlib.pyplot as plt
from PID import PID
import Medidas

rnd = random.Random(100)

class Particula:
  def __init__(self, dim, mini, maxi):
    # definindo semente de aleatoriedade
    global rnd

    self.X = list()
    self.V = list()
    self.pbest = list()
    self.I = 1.0
    self.erro = 0

    # Criando valores de position, velocidade e melhor posição da particula (pbest)

    for i in range(dim):
      self.X.append((maxi[i] - mini[i]) * rnd.random() + mini[i])
      self.V.append((maxi[i] - mini[i]) * rnd.random() + mini[i])

    self.pbest = copy.copy(self.X)

def Pid(num, den, set_point):  
  '''
  Esta funçao cria um PID com determinados parametros
  e devolve varios tipos de erro
  '''
 
  #instancia o PID
  pid = PID(num = num,den = den, set_point = set_point)

  return pid


def inicializate(n_part, dim, min, max):
  '''
  incializa as particulas com valores aleatórios
  entre o minimo e o maximo determinado
  '''

  particulas = [Particula( dim, min, max) for i in range(n_part)]

  return particulas


def updating_particle(particula ,min ,max, dim):
  '''
  Função que atualiza a posição da particula
  '''
  for i in range(dim):
    particula.X[i] = particula.X[i] + particula.V[i]

    if particula.X[i] > max[i]:
      particula.X[i] = max[i]

    elif particula.X[i] < min[i]:
      particula.X[i] = min[i]
    
  return particula


def updating_velocity(particula ,w ,c1 ,c2 ,g_best,dim ):
  '''
  Função que atualiza a velocidade da particula
  '''
  rnd = random.Random(100)
  
  for i in range(dim):
    prim = w*particula.V[i]
    seg =  c1*rnd.random()*(particula.pbest[i] - particula.X[i])
    terc = c2*rnd.random()*(g_best[i]-particula.X[i])

    new_velocity = prim + seg + terc

    particula.V[i] = new_velocity

  return particula


def updating_pbest(particula, pid_param):
  '''
  Função que atualiza o pbest, ou seje, o ponto 
  com o melhor resultado entre todos os pontos ja
  percorridos por dada particula. O melhor resultado
  depende se o problema é de minimização ou maximização.
  '''

  pid = Pid(pid_param[0], pid_param[1], set_point = pid_param[2])


  if particula.erro < Medidas.ISE(particula.pbest, pid_param):
    particula.pbest = particula.X.copy()

  return particula


def updating_improvement(particula ,pid):
  
  particula.I = Medidas.ISE(particula.pbest, pid) - Medidas.ISE(particula.X, pid)

  # print(f'particula.I : {particula.I}')
  # print(f'ise posição atual: {ISE(particula.X, pid_param)} , ise pbest {ISE(particula.pbest, pid_param)}')

  return particula


def updating_gbest(particulas):
  '''
  Função que atualiza o gbest, ou seje, o ponto 
  com o melhor resultado entre as particulas. O
  melhor resultado depende se o problema é de 
  minimização ou maximização.
  '''
  
  resultados = [particula.erro for particula in particulas]

  erro_min = np.min(resultados)
  arg_min = np.argmin(resultados)

  g_best = particulas[arg_min].X.copy()

  return g_best


def update_sistem(particulas ,min ,max ,w , c1, c2, parada, dim, pid_param):
  '''
  Função responável por atualizar todo o sistemas
  do PSO
  '''
  sem_melhoras = 0
    
  n_iter = 1

  particulas = [Medidas.ISE(particula, pid_param) for particula in particulas]

  g_best = updating_gbest(particulas, pid_param).copy()

  while(True):

    if (sem_melhoras > 0):
      print(f"\nSem melhoras á {sem_melhoras} iterações\n")

    if (sem_melhoras == 5):
      print(f'< Critério de parada atingido, o sistemas não teve melhoras significativas a 5 iterações. >')
      print(f'< {n_iter} iterações antes de atingir o critério. >')
      break

    particulas = [updating_particle(particula, min, max,dim) for particula in particulas]
    particulas = [updating_velocity(particula,w,c1,c2,g_best,dim) for particula in particulas]
    particulas = [Medidas.ISE(particula, pid_param) for particula in particulas]
    particulas = [updating_improvement(particula,pid_param) for particula in particulas]
    particulas = [updating_pbest(particula,pid_param) for particula in particulas]
   
    erro_gbest = Medidas.ISE(g_best,pid_param)

    
    gbest_it = updating_gbest(particulas,pid_param)
    erro_new_gbest = Medidas.ISE(gbest_it, pid_param)

    if erro_new_gbest < erro_gbest: 
      g_best = gbest_it

    plot_pid(g_best,pid_param)

    imp = [particula.I for particula in particulas]

    sem_melhoras = sem_melhoras + 1 if np.max(imp) < parada else 0

    n_iter += 1


def plot_pid(X,pid_param):
  erro = list()

  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])
  y, _, _ = pid.controller(X[0],X[1],X[2])
  p1, p2 = Medidas.Tempo_Subida(X, pid_param)

  erro.append(Medidas.ISE(X, pid_param))
  erro.append(Medidas.IAE(X, pid_param))
  erro.append(Medidas.ITSE(X, pid_param))
  erro.append(Medidas.Tempo_Acomodacao(X, pid_param))
  erro.append(p1)
  erro.append(p2)
  erro.append(Medidas.Overshoot(X, pid_param))

  pid.plot(y,erro, temp_plot = 0.25)
