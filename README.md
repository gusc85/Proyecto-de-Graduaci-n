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


# Brazo 3DOF con DLS + PID/LQR/Fuzzy

## Requisitos
- MATLAB R2021b o superior
- Control System Toolbox (para dlqr)
- Fuzzy Logic Toolbox (para mamfis, readfis, addInput, addMF, addRule, evalfis)  
  Nota: si no existe el archivo controlador_fuzzy_mamdani.fis, el código crea un FIS de respaldo automáticamente.
- No requiere App Designer; la GUI usa figure y uicontrol clásicos

---

## Ejecución
Desde el Command Window:
```matlab
brazo_fuzzy_gui_3d_mejorado
```
Al ejecutar se abre la ventana principal con:
- pestañas de control (modo, objetivos, parámetros)
- visualización 3D del brazo
- gráfica en vivo de posición cartesiana vs tiempo
- botones para simular, detener, reiniciar y correr batch con guardado a CSV

---

## Interfaz gráfica
- Pestaña Control
  - Selector de modo: DLS + PID, DLS + LQR, DLS + Fuzzy Centroid, DLS + Fuzzy Bisector
  - Objetivo cartesiano X, Y, Z
  - Longitudes L1, L2
  - Seguir mouse en plano XY
  - Activar ruido y sigma
  - Tolerancia de convergencia
  - Botones Simular, Detener, Reiniciar, Guardar Batch
  - Etiquetas de estado e indicadores numéricos (t, norma de error, lambda, mu, kappa)
- Pestaña PID/LQR
  - PID: Kp, Ki, Kd
  - LQR: Qx, Qy, Qz, R
  - Tiempos máximos por modo PID y LQR para la escala del eje X en la gráfica en vivo
- Pestaña Fuzzy
  - K mínima y K máxima en grados para escalar el paso articular
  - Método de defuzzificación: Centroid o Bisector
  - Tiempos máximos para Fuzzy Centroid y Fuzzy Bisector
  - Lambda adaptativo con parámetros a_mu y b_kappa
- Visualización 3D
  - Espacio de trabajo con esfera de alcance, disco en z=0, anillo de alcance mínimo y ejes cartesianos
  - Brazo con eslabones L1 y L2, efector final, objetivo y proyecciones a planos XY, XZ, YZ
  - Interacción con mouse:
    - clic derecho y arrastre vertical para ajustar Z del objetivo
    - rueda para cambiar Z
    - modo Seguir mouse para mover el objetivo en XY en tiempo real
- Gráfica en vivo
  - x(t), y(t), z(t) y líneas de referencia x_ref, y_ref, z_ref
  - límite del eje X sincronizado con el tiempo máximo del modo activo
- Post simulación
  - gráfica Norma del error cartesiano vs tiempo
  - si el modo es Fuzzy, gráfica de Kx, Ky, Kz vs tiempo
- Batch
  - genera objetivos en un círculo sobre el plano Z actual
  - ejecuta todos los modos y guarda resultados como CSV
  - crea una figura comparativa de tiempos promedio con barras de error

---

## Estructura del archivo
- Función principal
  - parámetros del sistema, límites articulares, objetivos y tiempos por modo
  - creación de la ventana, pestañas, controles y callbacks
  - configuración de visualización 3D y gráfica en vivo
  - bucle de simulación con DLS y control por modo (PID, LQR, Fuzzy)
  - funciones internas:
    - setupLivePlot, onTimeEdit, getModeTime, onModeChanged
    - applyDefuzzToFIS, onRun, finalizePlots, updateInfo
    - onStop, onReset, onSaveBatch
    - onMouseDown, onMouseUp, onMouseMove, onScroll
- Funciones auxiliares
  - compute_dlqr_safe: calcula K de LQR con fallback estable
  - drawWorkspace3D: renderiza el espacio de trabajo
  - drawArm3D: dibuja eslabones, efector, objetivo y proyecciones
  - updateProjections: actualiza proyecciones a planos
  - getMouseOnZ: intersección rayo-cámara con plano z
  - clampXYGivenZ, clampTarget3DAll: limitan objetivos a zona alcanzable
  - jacobian3d: jacobiano geométrico 3x3
  - fk3d: cinemática directa del brazo
  - clampJoint, clamp, wrapToPi_local, tern, valOrZero, posOrDefault
  - getKdegAndComponents: evalúa FIS y reparte K por ejes
  - evalfisFlexible: evalúa FIS con 1, 2 o 3 entradas según disponibilidad
  - simulateOnce: corre una simulación individual para batch
  - ensureFIS, crearFISporDefecto: carga o construye un FIS Mamdani de respaldo

---

## Solución de problemas
- Error Undefined function dlqr
  - instala y habilita Control System Toolbox
- Error Undefined function mamfis, readfis, addMF, evalfis
  - instala y habilita Fuzzy Logic Toolbox
- No existe controlador_fuzzy_mamdani.fis
  - el script crea un FIS por defecto; si deseas el tuyo, colócalo en la misma carpeta
- La ventana se congela o va lenta
  - reduce el tiempo máximo del modo activo o sube dt_step_s
  - cierra figuras previas con close all
- Al guardar batch no se crea el CSV o la imagen
  - verifica permisos de escritura en la carpeta de destino
- La escala de la gráfica en vivo no coincide con la simulación
  - ajusta T máx del modo activo en su pestaña; la gráfica usa ese valor como límite de eje X
- Convergencia no alcanzada
  - ajusta tolerancia, Kp/Ki/Kd, Q/R, o K mínima y máxima en Fuzzy
  - revisa objetivos fuera de alcanzabilidad; el código los clampa, pero puede requerir más tiempo
- Colores o estilos de líneas
  - se definen en drawWorkspace3D, drawArm3D y setupLivePlot; edítalos ahí si necesitas otro look

---

# Péndulo Invertido con PID, LQR y Fuzzy

## Requisitos
- MATLAB R2021b o superior  
- Control System Toolbox (para `lqr`) — opcional, el código usa valores por defecto si no está disponible  
- No requiere archivos externos: el FIS se construye dentro del script  

---

## Ejecución
Desde la ventana de comandos de MATLAB:
```matlab
pendulo_pid_lqr_fuzzy
```
Al ejecutar, se abrirá una ventana interactiva con:
- Animación del péndulo en 2D  
- Gráficas en tiempo real de ángulo y torque  
- Panel de estado con valores numéricos  
- Botones de simulación, reinicio y batch  

---

## Interfaz gráfica
- Panel izquierdo: animación del péndulo con barra y masa móvil  
- Panel superior derecho: gráfica θ (grados) y torque u (N·m) vs tiempo  
- Pestañas de control:
  - **⚙️ Control:** selección de modo, ángulo inicial, opciones de arrastre con mouse, auto-detención, y botones (Simular, Detener, Reiniciar, Guardar batch)
  - **🔵 PID:** parámetros Kp, Ki, Kd  
  - **🟢 LQR:** pesos Q11, Q22 y R, botón para recalcular K  
  - **🟠 Fuzzy:** parámetros Ko, Kmin, Kmax, tolerancia angular y método de defuzzificación (Centroid o Bisector)
- Panel “📊 Estado Actual”: muestra el ángulo, velocidad angular, torque y tiempo actual con colores dinámicos según magnitud

---

## Estructura del archivo
- Función principal: define los parámetros del sistema, crea la GUI y gestiona la simulación
- Secciones internas:
  - Configuración física y de controladores (PID, LQR, Fuzzy)
  - Callbacks para botones, sliders y ediciones
  - Temporizador (`TMR`) para actualizar la simulación en tiempo real
  - Rutinas de graficado y visualización
  - Lógica de auto-stop con verificación de convergencia
  - Función automática para generar gráfica de error vs tiempo
  - Función de batch para comparar los 4 métodos y exportar resultados

---

## Solución de problemas
- Error `Undefined function lqr`  
  Instala y habilita Control System Toolbox o usa las ganancias por defecto [10 2]
- Simulación lenta o congelada  
  Reduce la duración máxima (`tmax_pid`, `tmax_lqr`, etc.) o cierra figuras previas con `close all`
- La animación no responde  
  Verifica que el temporizador (`TMR`) esté en ejecución y que no haya errores en consola
- La gráfica de error no aparece  
  Asegúrate de detener la simulación con el botón **Detener** o que el auto-stop esté activo
- Los resultados del batch no se guardan  
  Revisa permisos de escritura y que la carpeta seleccionada exista
- Colores y estilos  
  Se pueden editar dentro de las funciones `refreshSim`, `refreshPlotAll` y `simulate_compare_and_save`

---
