"""
https://stackoverflow.com/questions/66263323/fit-3d-coordinates-into-a-parabola
"""

import math
import numpy as np
import sympy as sy
import pandas as pd
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from numpy.polynomial import Polynomial


# 
# curve fit
# 
def func2(x, xp, yp, a, b): # funciona bien en casos reales, pero ajusta la catenaria "por zonas"
    return a * (x[0] - xp) **2 + b * (x[1] - yp) **2 

def func(x, a, b, c, d, e):   # no funciona !?
    return a * x[0] **2 + b * x[1] **2 + c * x[0] + d * x[1] + e

# modelos estudiados en "3D CATENARY CURVE FITTING FOR GEOMETRIC CALIBRATION"
def catenary2(x, a, c, th, rho):
    tmp = math.cos(th) * x[0] - math.sin(th) * x[1] - rho*math.sin(th)
    return a + c * (np.cosh(tmp / c)  - 1)

def catenary1(x, a, b, c, th):
    tmp = math.cos(th) * x[0] - math.sin(th) * x[1] - b
    return a + c * (np.cosh(tmp / c)  - 1)

def plane(x, a, b, c):
    return a * x[0] + b * x[1] + c 


# ecuación de un plano 3d a partir de 3 puntos
#
# From: https://kitchingroup.cheme.cmu.edu/blog/2015/01/18/Equation-of-a-plane-through-three-points/
#
def plane_fit(ptos) :
  if ptos.shape[0] != 3:
     print (" Número de puntos", ptos.shape[0], "distinto de 3")
  p1 = ptos[0]
  p2 = ptos[1]
  p3 = ptos[2]

  # These two vectors are in the plane
  v1 = p3 - p1
  v2 = p2 - p1

  # the cross product is a vector normal to the plane
  cp = np.cross(v1, v2)
  cp /= np.linalg.norm(cp)
  a, b, c = cp

  # This evaluates a * x3 + b * y3 + c * z3 which equals d
  d = - np.dot(cp, p3)

  ### print('The equation is {0}x + {1}y + {2}z - {3} = 0'.format(a, b, c, d))
  return a, b, c, d



# aplicar ransac
# 
def ransac_polyfit(data, order=2, k=100, t=0.1, d=10, f=0.1, debug=0):
  # Thanks https://en.wikipedia.org/wiki/Random_sample_consensus

  # order – minimum number of data points required to fit the model
  # k – maximum number of iterations allowed in the algorithm
  # t – threshold value to determine when a data point fits a model
  # d – number of close data points required to assert that a model fits well to data
  # f – fraction of close data points required  (TODO: no se usa ahora)
  # debug - ver si se imprimen mensaje


  # inicializar todos los parámetros
  x = data[:,0]
  y = data[:,1]
  z = data[:,2]

  besterr = np.inf
  bestfit = None
  bestnuminliers = order
  bestinliers = None
  desired_prob = 0.95
  num_itera_needed = k

  # bucle principal
  for kk in range(k):

    popt_ = None
    c_ = 1.0
    # descartar planos "horizontales"
    while abs(c_) > 0.1 :
      random_indices = np.random.choice(data.shape[0], size=order, replace=False)  # select 4 points
      data_ = data[random_indices,:]
      #print ("index: ", random_indices, data_.shape)

      popt_ = plane_fit(data_) # ajustar a un plano
      a_, b_, c_, d_ = popt_
          
    if popt_ is not None :
      ## calcular la distancia al plano a probar de todos los puntos
      dist = a_ * x + b_ * y + c_ * z + d_ 

      alsoinliers = np.abs(dist) < t  # determinar que otros puntos están en la curva
      data_ = data[alsoinliers,:]  # TODO: falta incluir "random_indices" !?
      sum_inliers = sum(alsoinliers)

      #noinliers = np.abs(z - z_) < 2 * t # puntos "molestos" cerca de la curva
      #sum_inliers = 2*sum(alsoinliers) - sum(noinliers)
      
      if sum_inliers > d :   # si sobrepasan el número mínimo de puntos
        # try to readjust inliers !? !? !? 
        thiserr = np.sum(np.abs(dist[alsoinliers]))  # calcular el error total
        thiserr = thiserr / sum_inliers  # average error

        if sum_inliers > bestnuminliers :  
        # if sum_inliers > bestnuminliers  and  thiserr < besterr:  # también exigente
        # if thiserr < besterr :  # esta condicion sola FUNCIONA PEOR, es más "exigente"
          bestfit = popt_
          besterr = thiserr
          bestnuminliers = sum_inliers
          bestinliers = np.argwhere(alsoinliers != False)

          # for debug
          if debug > 0 :
            print ("     k %4d  inliers %5d error %.3f"%(kk, bestnuminliers, thiserr), "params", popt_)  

      """
          # calcular probabilidad de outliers
          prob_outlier = 1 - bestnuminliers  / (data.shape[0])
          num_itera_needed = math.log(1 - desired_prob)/math.log(1 - (1 - prob_outlier)**order)
          # for debug
          if debug > 0 :
            print('   prob_outlier:', prob_outlier, 'iterations needed:', num_itera_needed, " done:", k)
      if kk >  num_itera_needed :   # comprobar iteraciones según teoría y salir 
         if debug > 0 :
           print (" Last iteration", kk)
         break
      """
  return bestfit, bestnuminliers, besterr, bestinliers



# 
# leer datos de fichero
#
def leer_datos (filename ) :
  df = pd.read_csv(filename)
  
  points = df.to_numpy()

  print (" leídos ", points.shape, "datos del fichero ", filename, "\n")

  return points, df['X'], df['Y'], df['Z']


#
# leer datos formato laz  (https://laspy.readthedocs.io/en/latest/)
#
import laspy
def leer_las (filename) :
  las = laspy.read(filename)

  print (" leídos ", len(las.points), "datos del fichero ", filename, "\n")

  return np.column_stack([las.x, las.y, las.z]), las.x, las.y, las.z



##
## main
##

# inicializar dibujo
fig = plt.figure()
ax = plt.axes(projection = '3d')
ax.grid(True)


# leer datos de fichero y preprocesar

#  data, x, y, z = leer_datos('./data/output.csv')
#  data, x, y, z = leer_datos('./data/output1.csv')
#  data, x, y, z = leer_las('./data/pl_no_ground_LIDAR2019_NdP_70500_68500_EPSG2169.laz')
#  data, x, y, z = leer_las('./data/pl_no_ground_LIDAR2019_NdP_69500_69500_EPSG2169.laz')
data, x, y, z = leer_las('./data/pl_no_ground_LIDAR2019_NdP_72500_64500_EPSG2169.laz')
#ax.scatter(x, y, z, color='yellow', marker='+')  # debug: pintar todos los puntos
#plt.show() # debug: mostrar todos los puntos
  

# puntos mayores que altura máxima del fichero -11 metros en output.csv
max_z = np.max (data[:,2]) 
delta_z = 12 * 2.5
pts_z = z > (max_z - delta_z)  # (z > 355) # (z <= 338)  #(z < 348.6)  &  (z > 338)
data_z = data [pts_z,:]

print ("  puntos > ", max_z, "(max_z) - ", delta_z, " = ", data_z.shape, "\n")
#ax.scatter(data_z[:,0],data_z[:,1],data_z[:,2])  # debug: print selected points
#plt.show()  # debug: show selected points

data = data_z  # replace the data


# DEBUG: translate to the origin (smaller number) !?  # only usefull for catenary fitting
#data[:,0] = data[:,0] - np.min (data[:,0])
#data[:,1] = data[:,1] - np.min (data[:,1])



#
# aplicar RANSAC
#

# definir parámetros RANSAC
mydata = data
curves = 12
min_points = 200
th_error = 0.6


# bucle RANSAC
for j in range(curves) :
  best_model, numinliers, besterr, inliers = ransac_polyfit (mydata, order=3, k=900, t=th_error, d=min_points, f=0.1, debug=2)

  # Condición de finalización del bucle
  if best_model is None :
     print (" No more curves detected than ", j, "RANSAC planes")
     break

  # imprimir datos del modelo y dibujar los inliers detectados
  print("RANSAC %2d "%(j), " model: ", best_model)
  print('RANSAC %2d  num points: %5d  error %.3f '%(j, numinliers, besterr))

  mylabel = '%d (%d points)'%(j, numinliers)
  a, b, c, d = best_model
  mylabel += '  params  [%.5f %.5f %.5f %.5f]'%(a, b, c, d)
  ax.scatter(mydata[inliers,0], mydata[inliers,1], mydata[inliers,2],  marker='o', label=mylabel)  # RANSAC inliers

  # eliminar los puntos del modelo
  myindex = inliers
  mydata = np.delete(mydata, myindex, axis=0)
  print ('  left data:' , mydata.shape)
  if mydata.shape[0] < min_points :
    break

# mostrar resto de puntos (no ajustado a ninguna curva)
mylabel = 'noise (%d points)'%(mydata.shape[0])
ax.scatter(mydata[:,0], mydata[:,1], mydata[:,2], color='black', marker='+', label=mylabel) # noise or no RANSAC


# mostrar figura
plt.title("Planes   (threshold dist: %4.1f)"%th_error)
ax.legend()
plt.show()
