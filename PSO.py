import random
import copy    
import numpy as np
from PID import PID
import Medidas
import time
import numpy

rnd = random.Random()

class Particula:
  def __init__(self, j):
    # definindo semente de aleatoriedade

    self.X = list()
    self.V = list()
    self.pbest = list()
    self.gbest = list()
    self.I = 0
    self.erro = 0
    self.erro_gbest = 0
    
    # Criando valores de position, velocidade e melhor posição da particula (pbest)

    for i in range(3):
      if j < 2:
        self.X.append(rnd.random())
        # self.V.append((maxi[i] - mini[i]) * rnd.random() + mini[i])
        self.V.append(0)

      if j >= 2 and j< 5:
        self.X.append((rnd.random()*9)+1)
        # self.V.append((maxi[i] - mini[i]) * rnd.random() + mini[i])
        self.V.append(0)

      if j >= 5 and j< 7:
        self.X.append((rnd.random()*40)+10)
        # self.V.append((maxi[i] - mini[i]) * rnd.random() + mini[i])
        self.V.append(0)
      
      if j >= 7 and j< 10:
        self.X.append((rnd.random()*50)+50)
        # self.V.append((maxi[i] - mini[i]) * rnd.random() + mini[i])
        self.V.append(0)

    print('particula aleatoria',self.X)
    self.pbest = copy.deepcopy(self.X)
    self.erro_pbest = 0
  

def Pid(num, den):  
  # Esta funçao instacia um PID com determinados parametros

  pid = PID(num = num,den = den)

  return pid


def inicializate(n_part):
  '''
  incializa as particulas com valores aleatórios
  entre o minimo e o maximo determinado
  '''

  print('<- particulas geradas aleatóriamente ->')

  particulas = [Particula(i) for i in range(n_part)]

  return particulas


def updating_particle(particula):
  '''
  Função que atualiza a posição da particula
  '''
  for i in range(3):
    particula.X[i] = particula.X[i] + particula.V[i]

    # if particula.X[i] > max[i]:
    #   particula.X[i] = max[i]

    if particula.X[i] < 0.001:
      particula.X[i] = 0.001
    
  return particula


def updating_velocity(particula, w, c1, c2, g_best):
  '''
  Função que atualiza a velocidade da particula
  '''
  rnd = random.Random(100)
  
  for i in range(3):
    prim = w*particula.V[i]
    seg =  c1*rnd.random()*(particula.pbest[i] - particula.X[i])
    terc = c2*rnd.random()*(g_best[i]-particula.X[i])

    new_velocity = prim + seg + terc

    particula.V[i] = new_velocity

  return particula


def updating_pbest(particula, ini = False):
  '''
  Função que atualiza o pbest, ou seje, o ponto 
  com o melhor resultado entre todos os pontos ja
  percorridos por dada particula. O melhor resultado
  depende se o problema é de minimização ou maximização.
  '''

  if ini:
    particula.erro_pbest = particula.erro

  else:
    if particula.erro < particula.erro_pbest:
      particula.pbest = particula.X.copy()
      particula.erro_pbest = particula.erro

  return particula


def updating_improvement(particula):
  
  particula.I = particula.erro_pbest - particula.erro

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


def update_sistem(sistema, particulas, w, c1, c2, parada, pid_param):
  '''
  Função responável por atualizar todo o sistemas
  do PSO
  '''

  pid = Pid(pid_param[0], pid_param[1])
  tempo = list()

  ISE_MA = Medidas.ISE(pid, ma=True)
  TACO_MA = Medidas.Tempo_Acomodacao(pid, ma=True)

  pid.set_ISE_MA(ISE_MA)
  pid.set_TACO_MA(TACO_MA)

  sem_melhoras = 0
  n_iter = 1

  particulas = [Medidas.Multi_erro(particula, pid) for particula in particulas]
  particulas = [updating_pbest(particula, ini=True) for particula in particulas]

  g_best = finding_gbest(particulas).copy()
  erro_gbest = Medidas.Multi_erro(g_best,pid)

  particulas = [updating_gbest(particula,g_best,erro_gbest) for particula in particulas]
  sistema.append(copy.deepcopy(particulas))

  plot_pid(g_best, pid, n_iter)

  while(True):
    ini = time.time()

    if (sem_melhoras == 5):
      plot_pid(g_best, pid, save = True)
      print('Kp: ',g_best[0])
      print('Ti: ',g_best[1])
      print('Td: ',g_best[2])
      print(f'< Critério de parada atinigido, nenhuma particula teve uma melhora maior que {parada} por 5 iterações consecutivas. >')
      print(f'< {n_iter} iterações antes de atingir o critério. >')
      print(f'< O tempo médio das iterações foi {round(numpy.mean(tempo), 3)}s. >')
      break
      
    # print('iterações sem melhorar: ',sem_melhoras)
    particulas = [updating_velocity(particula,w,c1,c2,g_best) for particula in particulas]
    particulas = [updating_particle(particula) for particula in particulas]
    # particulas = [updating_velocity(particula,w,c1,c2,g_best,dim) for particula in particulas]
    particulas = [Medidas.Multi_erro(particula, pid) for particula in particulas]
    particulas = [updating_improvement(particula) for particula in particulas]
    particulas = [updating_pbest(particula) for particula in particulas]

    erro_gbest = Medidas.Multi_erro(g_best,pid)
    gbest_it = finding_gbest(particulas)
    erro_new_gbest = Medidas.Multi_erro(gbest_it, pid)

    if erro_new_gbest < erro_gbest:
      g_best = gbest_it
      particulas = [updating_gbest(particula, g_best, erro_new_gbest) for particula in particulas]

    n_iter += 1

    plot_pid(g_best, pid, n_iter)

    sistema.append(copy.deepcopy(particulas))

    imp = [particula.I for particula in particulas]

    sem_melhoras = sem_melhoras + 1 if np.max(imp) < parada else 0

    fim = time.time()
    tempo.append(fim-ini)
    

  return copy.deepcopy(sistema)


def plot_pid(X, pid, i, save = True):
  erro = list()

  y, _, _, _, _, T = pid.resposta_MF(X[0],X[1],X[2])

  temp_plot = Medidas.Tempo_Acomodacao(pid, ma=True) + 0.5

  p1, p2 = Medidas.Tempo_Subida(X, pid)

  erro.append(Medidas.Multi_erro(pid = pid, particula = X))
  erro.append(Medidas.IAE(pid = pid, particula = X))
  erro.append(Medidas.ITSE(pid = pid, particula = X))
  erro.append(Medidas.Tempo_Acomodacao(pid = pid, particula = X))
  erro.append(p1)
  erro.append(p2)
  erro.append(Medidas.Overshoot(pid = pid, particula = X))

  pid.plot_MF(y, erro, T, i, temp_plot = temp_plot, save=save)
