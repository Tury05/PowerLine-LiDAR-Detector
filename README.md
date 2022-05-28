# TFG-LiDAR-Processing

# Avances
- Función de limpieza de ruido
- Filtrado por distribución de alturas

# Cosas por hacer
- Detectar lineas rectas
- Posibilidad de lineas en barrancos (No se ha probado este caso)
- Concurrencia y paralelismo

# Ejecucion

Se ha implementado el flag "-n" en el caso de que se quiera que el archivo de salida no incluya los puntos pertenecientes al suelo.


Ejemplo de ejecución incluyendo los puntos "ground" en el archivo de salida:
```
cargo run <input.las> <output.las>
```

Ejemplo de ejecución eliminando los puntos "ground" en el archivo de salida:
```
cargo run <input.las> <output.las> -n
```

