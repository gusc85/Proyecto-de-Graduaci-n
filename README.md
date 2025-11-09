# Proyecto-de-Graduaci-n
# Controlador_Mamdani — GUI Mamdani para control de ventilador (Temp/Humedad)

## Requisitos
- MATLAB R2021b o superior (recomendado R2022a+)
- Fuzzy Logic Toolbox (para mamfis, addInput, addMF, addRule, evalfis, plotmf, etc.)
- Soporte para App Designer UI Components (uifigure, uiknob, uigauge, uitable, etc.)

---

## Ejecución
Desde el Command Window o un script:
```matlab
Controlador_Mamdani
```
No recibe parámetros. Al ejecutar, abre la GUI con valores y visualización por defecto.  
Puedes operar manualmente con los knobs o activar el barrido automático con el botón “▶ Simulación”.

---

## Interfaz gráfica (GUI)
- Knobs:
  - Temperatura (0–100 °C)
  - Humedad (0–100 %)
- Botón “▶ Simulación / ⏸ Pausar”:  
  Activa o pausa un barrido automático (temporizador 0.3 s) que recorre temperatura y humedad con inversión de sentido en los límites.
- Panel “Visualización Difusa”:  
  Muestra las funciones de membresía de la variable activa, con línea vertical y puntos de pertenencia.
- LED e iconos:  
  Cambian color y símbolo según la MF dominante (🧊, 🌡️, 🔥 o 💧).
- Medidor semicircular:  
  Representa la velocidad del ventilador (0–10).
- Etiqueta “Velocidad: x.xx”:  
  Muestra el valor numérico de la salida difusa.
- Tabla de reglas:  
  Evalúa combinaciones representativas de temperatura y humedad.
- Botón “Ver superficie 3D”:  
  Muestra la superficie Temperatura–Humedad–Velocidad y exporta como archivo resultados_superficie_puntos.eps compatible con LaTeX.

---

## Estructura del archivo
- Controlador_Mamdani.m
  - Define el FIS (entradas, salida, MFs y reglas)
  - Crea la interfaz gráfica (uifigure, uiknob, uigauge, uitable, LED)
  - Incluye callbacks internos:
    - onTempChanged
    - onHumChanged
    - simStep
    - toggleSimulacion
    - actualizarVisual
  - Contiene la función mostrarSuperficie3D(fisLocal) para graficar la superficie 3D y exportarla en formato EPS

---

## Solución de problemas
- Error “Undefined function ‘mamfis’ / ‘evalfis’ / ‘plotmf’”:  
  Instala y activa el Fuzzy Logic Toolbox.

- La GUI no aparece o se cierra:  
  Cierra figuras previas (close all) y verifica que no haya errores en la consola de MATLAB.

- Error al exportar EPS:  
  Revisa permisos de escritura en la carpeta o cambia el nombre del archivo exportado.

---
