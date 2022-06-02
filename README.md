# TFG-LiDAR-Processing

# Avances
- Función de limpieza de ruido
- Filtrado por distribución de alturas

# Cosas por hacer
- Detectar lineas rectas
- Posibilidad de lineas en barrancos (No se ha probado este caso)
- Concurrencia y paralelismo

# Detalles
El fichero python ejecuta la segmentación de suelo a un fichero establecido en el codigo.
Para cambairlo hay que hacerlo desde el codigo (lineas 5 y 12).

# Ejecucion

El programa se ejecuta con el siguiente comando:
```
python filtering.py
cargo run <input.las> <output.las>
```

