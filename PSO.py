import random
import copy    
import numpy as np
from PID import PID
import Medidas

rnd = random.Random()

class Particula:
  def __init__(self, dim, mini, maxi):
    # definindo semente de aleatoriedade
    global rnd

    self.X = list()
    self.V = list()
    self.pbest = list()
    self.gbest = list()
    self.I = 1.0
    self.erro = 0
    self.erro_gbest = 0
    
    # Criando valores de position, velocidade e melhor posição da particula (pbest)

    for i in range(dim):
      self.X.append((maxi[i] - mini[i]) * rnd.random() + mini[i])
      self.V.append((maxi[i] - mini[i]) * rnd.random() + mini[i])

    self.pbest = copy.deepcopy(self.X)
  

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


  if particula.erro < Medidas.Multi_erro(particula.pbest, pid_param):
    particula.pbest = particula.X.copy()

  return particula


def updating_improvement(particula ,pid):
  
  particula.I = Medidas.Multi_erro(particula.pbest, pid) - Medidas.Multi_erro(particula.X, pid)

  # print(f'particula.I : {particula.I}')
  # print(f'Multi_erro posição atual: {Multi_erro(particula.X, pid_param)} , Multi_erro pbest {Multi_erro(particula.pbest, pid_param)}')

  return particula


def updating_gbest(particula, gbest, erro_gbest):
  
  particula.gbest = gbest
  particula.erro_gbest = erro_gbest

  return particula


def finding_gbest(particulas):
  '''
  Função que atualiza o gbest, ou seje, o ponto 
  com o melhor resultado entre as particulas. O
  melhor resultado depende se o problema é de 
  minimização ou maximização.
  '''
  
  resultados = [particula.erro for particula in particulas]

  arg_min = np.argmin(resultados)

  g_best = particulas[arg_min].X.copy()

  return g_best


def update_sistem(sistema,particulas ,min ,max ,w , c1, c2, parada, dim, pid_param):
  '''
  Função responável por atualizar todo o sistemas
  do PSO
  '''

  sem_melhoras = 0
    
  n_iter = 1

  particulas = [Medidas.Multi_erro(particula, pid_param) for particula in particulas]

  g_best = finding_gbest(particulas).copy()
  erro_gbest = Medidas.Multi_erro(g_best,pid_param)

  particulas = [updating_gbest(particula,g_best,erro_gbest) for particula in particulas]

  sistema.append(copy.deepcopy(particulas))

  while(True):

    if (sem_melhoras > 0):
      print(f"\nSem melhoras á {sem_melhoras} iterações\n")

    if (sem_melhoras == 5):
      print(f'< Critério de parada atingido, o sistemas não teve melhoras significativas a 5 iterações. >')
      print(f'< {n_iter} iterações antes de atingir o critério. >')
      break

    particulas = [updating_particle(particula, min, max,dim) for particula in particulas]
    particulas = [updating_velocity(particula,w,c1,c2,g_best,dim) for particula in particulas]
    particulas = [Medidas.Multi_erro(particula, pid_param) for particula in particulas]
    particulas = [updating_improvement(particula,pid_param) for particula in particulas]
    particulas = [updating_pbest(particula,pid_param) for particula in particulas]
   
    erro_gbest = Medidas.Multi_erro(g_best,pid_param)

    
    gbest_it = finding_gbest(particulas)
    erro_new_gbest = Medidas.Multi_erro(gbest_it, pid_param)

    if erro_new_gbest < erro_gbest: 
      g_best = gbest_it
      particulas = [updating_gbest(particula, g_best, erro_new_gbest) for particula in particulas]

    plot_pid(g_best,pid_param)

    sistema.append(copy.deepcopy(particulas))

    imp = [particula.I for particula in particulas]

    sem_melhoras = sem_melhoras + 1 if np.max(imp) < parada else 0

    n_iter += 1


  return copy.deepcopy(sistema)


def plot_pid(X,pid_param):
  erro = list()

  pid = Pid(pid_param[0],pid_param[1],set_point = pid_param[2])
  y, _, T = pid.resposta_MF(X[0],X[1],X[2])

  temp_plot = Medidas.Tempo_Acomodacao(pid_param, ma=True) + 0.5

  p1, p2 = Medidas.Tempo_Subida(X, pid_param)

  erro.append(Medidas.Multi_erro(pid_param = pid_param, particula = X))
  erro.append(Medidas.IAE(pid_param = pid_param, particula = X))
  erro.append(Medidas.ITSE(pid_param = pid_param, particula = X))
  erro.append(Medidas.Tempo_Acomodacao(pid_param = pid_param, particula = X))
  erro.append(p1)
  erro.append(p2)
  erro.append(Medidas.Overshoot(pid_param = pid_param, particula = X))

  pid.plot_MF(y,erro,T,temp_plot = temp_plot)
