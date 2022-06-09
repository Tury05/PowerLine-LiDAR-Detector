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

# crear los datos sintéticos
# 
def crear_datos (n=400, num_lines=9, space_lines=100, error=10, space=1000) :

  np.random.seed(123)

  a_gt, b_gt, c_gt, d_gt, e_gt = datos_semilla()
  print('gt: z = %.5f * x ^ 2 + %.5f * y ^ 2 + %.5f * x + %.5f * y + %.5f' %(a_gt, b_gt, c_gt, d_gt, e_gt))


  # construcción de los datos (varias líneas)
  x = np.array([])
  y = np.array([])
  z = np.array([])


  for i in range(num_lines):
    xi = np.linspace(-2*space, space, n) + i * space_lines
    yi = np.linspace(-space, space, n)
    erri_ = np.row_stack ( np.random. rand(n, 1) ) 
    erri = (erri_.reshape(n,) - np.full(n, 0.5) ) * error 
    #print ("err size", erri)
    z0i = a_gt * xi ** 2 + b_gt * yi ** 2 + c_gt * xi + d_gt * yi + e_gt  
    #print (" z no error: ", z0)
    zi = z0i  + erri
    #print (" z", zi)

    x = np.append(x, xi)
    y = np.append(y, yi)
    z = np.append(z, zi)

  data = np.column_stack([x, y, z])
  print("  Sizes", x.shape, y.shape, z.shape, data.shape )

  return data, x, y, z


# curve fit
# 
def func2(x, xp, yp, a, b): # funciona bien en casos reales, pero ajusta la catenaria "por zonas"
    return a * (x[0] - xp) **2 + b * (x[1] - yp) **2 

def plane(x, a, b, c, d):   # no funciona
    return a * x[0] + b*x[1] + c

# From "3D CATENARY CURVE FITTING FOR GEOMETRIC CALIBRATION" DOI: 10.5194/isprsarchives-XXXVIII-5-W12-259-2011
def catenary2(x, a, c, th, rho):
    tmp = math.cos(th) * x[0] - math.sin(th) * x[1] - rho*math.sin(th)
    return a + c * (np.cosh(tmp / c)  - 1)

def catenary1(x, a, b, c, th):
    tmp = math.cos(th) * x[0] - math.sin(th) * x[1] - b
    return a + c * (np.cosh(tmp / c)  - 1)


# datos semilla
#
def datos_semilla () :
  a_gt = 0.000004
  b_gt = 0.00005
  c_gt = 0.00008
  d_gt = -0.0005
  e_gt = 50

  return a_gt, b_gt, c_gt, d_gt, e_gt


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
    random_indices = np.random.choice(data.shape[0], size=order, replace=False)  # select 4 points
    data_ = data[random_indices,:]
    #print ("index: ", random_indices, data_.shape)

    popt_ = None
    try:
      popt_, _ = curve_fit(catenary2, data_[:,:2].T, ydata=data_[:,2])   # ajustar a una parábola 3D
    except RuntimeError:
      #print("    Error in initial fit, try again")
      pass
          
    if popt_ is not None :
      ## quadratic function
      ##xp_, yp_, a_, b_ = popt_
      ##z_ = a_ * (x - xp_) ** 2 + b_ * (y - yp_) ** 2    # calcular la z "teórica"
      ## catenary function
      a_, c_, th_, rho_ = popt_
      tmp_ = math.cos(th_) * x - math.sin(th_) * y - rho_ * math.sin(th_)
      z_ = a_ + (np.cosh(tmp_ / c_) - 1)

      alsoinliers = np.abs(z - z_) < t  # determinar que otros puntos están en la curva
      data_ = data[alsoinliers,:]  # TODO: falta incluir "random_indices" !?
      sum_inliers = sum(alsoinliers)

      #noinliers = np.abs(z - z_) < 2 * t # puntos "molestos" cerca de la curva
      #sum_inliers = 2*sum(alsoinliers) - sum(noinliers)
      
      if sum_inliers > d :   # si sobrepasan el número mínimo de puntos
        # try to readjust inliers !? !? !? 
        thiserr = np.sum(np.abs(data_[:,2] - z_[alsoinliers]))  # calcular el error total
        thiserr = thiserr / sum_inliers  # average error
        # comprobar si el nuevo modelo (parábola 3D) es mejor y guardar

        if sum_inliers > bestnuminliers :  
        # if sum_inliers > bestnuminliers  and  thiserr < besterr:  # también exigente
        # if thiserr < besterr :  # esta condicion sola FUNCIONA PEOR, es más "exigente"
          bestfit = popt_
          besterr = thiserr
          bestnuminliers = sum_inliers
          bestinliers = np.argwhere(alsoinliers != False)

          # for debug
          if debug > 0 :
            print ("     k", kk, " inliers ", bestnuminliers, "error", thiserr, "params", popt_)  
          if debug > 2 :
            print ("    index: ", random_indices, data_.shape[0],  np.transpose(bestinliers) )

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



# https://stackoverflow.com/questions/66263323/fit-3d-coordinates-into-a-parabola
# test data generation with some noise 
# here read in your data

# draw scatter coordiante

fig = plt.figure()
ax = plt.axes(projection = '3d')


synthetic = False
error = 1

if synthetic == True:
  # parámetros de las líneas, los puntos de cada línea y el error al generar los datos
  n = 400
  num_lines = 6
  space_lines = 100
  space = 1000
  data, x, y, z = crear_datos(n, num_lines, space_lines, error, space)

  ax.scatter(x, y, z, color='yellow', marker='+')
  #plt.show() # todo ok

else : 
  #  data, x, y, z = leer_datos('./data/output.csv')
  #  data, x, y, z = leer_datos('./data/output1.csv')
  #  data, x, y, z = leer_las('./data/pl_no_ground_LIDAR2019_NdP_70500_68500_EPSG2169.laz')
  data, x, y, z = leer_las('./data/pl_no_ground_LIDAR2019_NdP_69500_69500_EPSG2169.laz')
  #  data, x, y, z = leer_las('./data/pl_no_ground_LIDAR2019_NdP_72500_64500_EPSG2169.laz')
  #ax.scatter(x, y, z, color='yellow', marker='+')  # debug: pintar todos los puntos
  #plt.show() # debug: mostrar todos los puntos
  
  # puntos mayores que altura máxima del fichero -11 metros en output.csv
  max_z = np.max (data[:,2]) 
  delta_z = 12 * 2.5
  pts_z = z > (max_z - delta_z)  # (z > 355) # (z <= 338)  #(z < 348.6)  &  (z > 338)
  data_z = data [pts_z,:]

  print ("  puntos > ", max_z, "(max_z) - ", delta_z, " = ", data_z.shape)
  #ax.scatter(data_z[:,0],data_z[:,1],data_z[:,2])  # debug: print selected points
  #plt.show()  # debug: show selected points

  data = data_z  # replace the data


# DEBUG: initial fit to a catenay
dif_x = np.min (data[:,0])
dif_y = np.min (data[:,1])
data[:,0] = data[:,0] - dif_x
data[:,1] = data[:,1] - dif_y
popt_ = None
try:
   popt_, _ = curve_fit(catenary2, data[:,:2].T, ydata=data[:,2], p0=[348,0.8,2.6,-130])   # ajustar a la función
except RuntimeError:
   print("    Error in initial fit")
   pass
          
if popt_ is not None :
   ## quadratic function
   ##xp_, yp_, a_, b_ = popt_
   ##z_ = a_ * (x - xp_) ** 2 + b_ * (y - yp_) ** 2    # calcular la z "teórica"
   ## catenary function
   a_, c_, th_, rho_ = popt_
   tmp_ = math.cos(th_) * data[:,0] - math.sin(th_) * data[:,1] - rho_ * math.sin(th_) 
   z_ = a_ + np.cosh((tmp_ / c_) - 1)

   #ax.scatter(data[:,0], data[:,1], z_, color='red')
   #ax.scatter(data[:,0], data[:,1], data[:,2], color='blue')
   print(a_, c_, th_, rho_, tmp_, z_, "error", np.sum(np.abs(data[:,2] - z_)) / z_.shape[0])
   #plt.show()

# representación 2D: salen líneas rectas !!!
#plt.plot(x, y, "o", markersize=5 )
#plt.show()



# fit curve (no funciona con varias curvas !!!)
if synthetic == True :
  myrange = np.arange(n, 2*n)
  popt, _ = curve_fit(catenary2, data[myrange,:2].T, ydata=data[myrange,2])
  xp, yp, a, b = popt
  print('FIT eq: z = %.5f * (x - %.5f) ^ 2 + %.5f * (y - %5.f) ^ 2 ' %(a, xp, b, yp))
  z1b = a * (x - xp) ** 2 + b * (y - yp) ** 2

  '''
  popt, _ = curve_fit(plane, data[myrange,:2].T, ydata=data[myrange,2])
  a, b, c, d = popt
  print('FIT eq: z = %.5f * x + %.5f * y + %5.f + %5.f) ^  ' %(a, b, c, d))
  z1b = a * x  + b * y + c
  '''

  fit_error = np.sum( np.abs (z1b[myrange] - data[myrange, 2]) )
  print ('FIT num points: ', myrange.shape)
  print ('FIT points: ', myrange)
  print ('FIT error: ', fit_error)
  ax.scatter(x[myrange], y[myrange], z1b[myrange], color='red')

  plt.show()


"""
### DEBUG (sobra)
###
# plot fitted curve (TODO: sobra)
#npoints = 100
#x1 = np.linspace(-space, space, npoints )
#y1 = np.linspace(-space, space, npoints )
#z1 = 1 * x1 ** 2 + b * y1 ** 2 + c * x1 + d * y1 + e
#ax.plot(x1, y1, z1, color='green')  # GLOBAL fitted curve
# comparar con "ground truth": datos originales
#z1a = a_gt * x1 ** 2 + b_gt * y1 ** 2 + c_gt * x1 + d_gt * y1 + e_gt
#ax.scatter(x1, y1, z1a, color='pink')
#z1gt = a_gt * x ** 2 + b_gt * y ** 2 + c_gt * x + d_gt * y + e_gt  # son los mismos datos semilla !?
#ax.scatter(x, y, z1gt, color='blue')
#print (z1gt, "\n",  z1b) #  , "\n",  err)
"""


#aplicar RANSAC
mydata = data
curves = 12
min_points = 200

colors = cm.rainbow(np.linspace(0, 1, curves))

for j,c in zip(range(curves), colors) :
  best_model, numinliers, besterr, inliers = ransac_polyfit (mydata, order=4, k=900, t=1.2, d=min_points, f=0.1, debug=2)

  if best_model is None :
     print (" No more curves detected than ", j, "RANSAC curves")
     break

  print("RANSAC", j, " model: ", best_model, "    color", c)
  print('RANSAC', j, ' num points: ', numinliers)
  print("RANSAC", j, " error: ", besterr)
  #print('RANSAC', j, ' points: ', np.transpose(inliers) )

  my_x = mydata[:,0]
  my_y = mydata[:,1]
  my_z = mydata[:,2]
  #xp, yp, a, b = best_model ##  ==> parabole !!!
  #z1 = 1 * x1 ** 2 + b * y1 ** 2 + c * x1 + d * y1 + e
  ###ax.plot(x1, y1, z1, color='black')  # RANSAC curve  ==> bad x !! (x change with each line!!)
  #z1n =  a * (my_x - xp) ** 2 + b * (my_y - yp) ** 2  ##  ==> parabole !!!
  #ax.scatter(my_x[inliers], my_y[inliers], z1n[inliers], color='black')  # RANSAC curve

  a, c, th, rho = best_model
  #tmp_ = math.cos(th) * my_x[inliers] - math.sin(th) * my_y[inliers] - rho * math.sin(th) 
  #z_ = a + np.cosh((tmp_ / c) - 1)
  mylabel = '%d (%d points)'%(j, numinliers)
  mylabel += '  params  [%5.1f %5.1f %5.3f %5.1f]'%(a, c, th, rho)
  ax.scatter(my_x[inliers], my_y[inliers], my_z[inliers],  marker='o', label=mylabel)  # RANSAC inliers 
  #ax.scatter(my_x[inliers], my_y[inliers], z_[inliers], color=c,  marker='+')  # RANSAC projected

  myindex = inliers
  mydata = np.delete(mydata, myindex, axis=0)
  print ('  left data:' , mydata.shape)
  if mydata.shape[0] < min_points :
    break

mylabel = 'noise (%d points)'%(mydata.shape[0])
ax.scatter(mydata[:,0], mydata[:,1], mydata[:,2], color='black', marker='+', label=mylabel) # noise or no RANSAC

ax.legend()
ax.grid(True)
plt.title("Catenary2")
plt.show()
