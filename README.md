# TFG-LiDAR-Processing

# Avances
- Modificacion erode y dilate (Añadido opcion de vecindad 4/8)

# Cosas por hacer
- Probar RANSAC
- Concurrencia y paralelismo

# Ejecucion

El programa se ejecuta con el siguiente comando:
```
cargo run <carpeta_inputs> <carpeta_outputs>
```

Para ejecutar el archivo filtering se debe introducir el siguiente comando:

```
python filtering.py [flag] <carpeta_inputs>
```
Los flags disponibles son -g para obtener la nube con los grounds clasificados
y -gt para obtener los ground truths.

# Ejecucion Metricas

Entrar en la carpeta confusion-matrix y ejecutar el comando:
```
cargo run <carpeta de nubes filtradas> <carpeta de ground truths> <carpeta con tiles originales>
```