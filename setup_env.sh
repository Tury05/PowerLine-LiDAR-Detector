#!/bin/bash
set -e

# -----------------------------
# 1️⃣ Activar entorno conda
# -----------------------------
ENV_NAME="pl_detector"

echo "Activando entorno conda: $ENV_NAME..."
source $(conda info --base)/etc/profile.d/conda.sh
conda activate $ENV_NAME || {
    echo "No se pudo activar el entorno '$ENV_NAME'. Asegúrate de que existe."
    exit 1
}

# -----------------------------
# 2️⃣ Instalar dependencias Python
# -----------------------------
if [ -f "environment.yml" ]; then
    echo "Instalando dependencias Python desde environment.yml..."
    conda env update --name $ENV_NAME --file environment.yml
else
    echo "No se encontró environment.yml. Saltando instalación de dependencias."
fi

# -----------------------------
# 3️⃣ Compilar módulo Rust
# -----------------------------
RUST_DIR="lidar-processing"

if [ -d "$RUST_DIR" ]; then
    echo "Compilando módulo Rust..."
    cd $RUST_DIR
    cargo build --release
    cd ..
    echo "Compilado en $RUST_DIR/target/release/"
else
    echo "No se encontró la carpeta ($RUST_DIR). Saltando compilación."
fi

# -----------------------------
# 3️⃣ Compilar módulo validación Rust
# -----------------------------
RUST_DIR="confusion-matrix"

if [ -d "$RUST_DIR" ]; then
    echo "Compilando módulo Rust de validacion..."
    cd $RUST_DIR
    cargo build --release
    cd ..
    echo "Compilado en $RUST_DIR/target/release/"
else
    echo "No se encontró la carpeta ($RUST_DIR). Saltando compilación."
fi

echo "✅ Entorno listo."