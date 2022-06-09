
Implementación propia para detectar las líneas de alta tensión mediante método de RANSAC

0) activar entorno
conda activate myenv

1) Primer paso (opcional): convertir fichero .laz a .csv:
(ya no es necesario porque ahora se pueden leer directamente los ficheros de formato .las y .laz)
$ pdal pipeline las2csv.json --readers.las.filename=./pl_no_ground_LIDAR2019_NdP_69500_69500_EPSG2169.laz


2a) para probar curva cuadrática:   ==> TODO: fallo de representación, es una curva de revolución y no una línea !!!!!!!
python 


2b) para probar curva caternaria: 
siguiendo el paper "3D CATENARY CURVE FITTING FOR GEOMETRIC CALIBRATION" DOI: 10.5194/isprsarchives-XXXVIII-5-W12-259-2011
python  ransac_3dcatenary2.py  


3) opciones:
  a) datos sintéticos: cambiar línea 207 synthetic = False a True
  b) cambiar fichero: descomentar una línea entre 222 y 226 (se pueden poner otros ficheros)
  c) cambiar la "altura de corte": editar línea 232:   delta_z = 12 * 2.5 
  d) cambiar parámetros del ajuste RANSAC: línea 325: 
       ransac_polyfit (mydata, order=4, k=900, t=1.2, d=min_points, f=0.1, debug=2)
       Ver explicación de cada parámetro en la definición de la función (el parámetro f no se utiliza)
  e) activar la sección a evitar (puntos que pesan negativamente): descomentar líneas 132-133 (noinliers y sum_inliers)
       OJO: porque se eliminan los datos de noinliers y se restan de los inliers !!! (puede ser lioso de interpretar)


4) Obviamente, hay muchísimas cosas comentadas. La mayoría sobra, otras son para depuración