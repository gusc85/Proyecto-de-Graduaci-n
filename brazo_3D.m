function brazo_fuzzy_gui_3d_mejorado()

%% ---------- Parámetros del sistema ----------
% Definición de geometría base del brazo
L1 = 10; L2 = 7;
theta1 = deg2rad(0);
theta2 = deg2rad(15);
theta3 = deg2rad(0);

% Definición de límites articulares realistas (en grados)
JOINT_LIMITS = struct( ...
    'theta1', [-180, 180], ...
    'theta2', [-120, 120], ...
    'theta3', [-150, 150]);

% Definición de objetivo cartesiano inicial
x_ref = 10; y_ref = -14; z_ref = -4;

% Cálculo de alcance (mínimo y máximo) y rango de dibujo
rmin = abs(L1 - L2);
rmax = L1 + L2;
RANGE = rmax + 4;

% Definición del paso de simulación (frecuencia del lazo visual)
dt_step_s = 0.015;   % ~67 Hz

% Configuración de tiempos máximos por modo de control
Tmax_PID     = 15;   % s
Tmax_LQR     = 12;   % s
Tmax_FuzzyC  = 15;   % s
Tmax_FuzzyB  = 15;   % s

% Selección inicial del límite del eje X de la gráfica en vivo (modo PID)
current_live_xlim = Tmax_PID;

% Configuración de parámetros de control y criterios de estabilidad
tol             = 0.02;
stable_needed   = 30;   % muestras consecutivas dentro de la tolerancia
beta            = 0.88; % suavizado exponencial de incremento articular
K_min_deg       = 0.5;
K_max_deg       = 7.0;
K_safety_ang_deg = 10;

% Configuración base de DLS (lambda y umbrales)
lambda_base     = 0.12;
lambda_smallE   = 0.30;
err_small_th    = 0.20;
cond_boost      = 0.30;
singularity_threshold = 1e-3;

% Parámetros por defecto PID
Kp_def = 1.2;
Ki_def = 0.008;
Kd_def = 1.8;

% Parámetros por defecto LQR (pesos y costo)
Qx_def = 0.08;
Qy_def = 0.08;
Qz_def = 0.08;
R_def  = 8;

% Configuración de ruido (opcional)
noise_on    = false;
sigma_noise = 0.00;

% Activación de lambda adaptativo y coeficientes asociados
lambda_soft_on = true;
a_mu   = 0.00;
b_kappa = 0.00;

% Preparación del FIS (lectura desde archivo o creación por defecto)
fis1 = ensureFIS('controlador_fuzzy_mamdani.fis');

%% ---------- Crear ventana principal ----------
% Construcción de la ventana principal del simulador
fig = figure('Name', 'Simulador Brazo Robótico 3DOF - Control Avanzado', ...
    'NumberTitle', 'off', ...
    'Position', [40 30 1440 820], ...
    'Color', [0.94 0.94 0.96], ...
    'MenuBar', 'none', ...
    'ToolBar', 'figure');

%% ---------- Panel de control con TABS ----------
% Construcción del contenedor de pestañas
tabGroup = uitabgroup(fig, 'Position', [0.01 0.02 0.24 0.96]);

% --- TAB 1: Control Principal ---
% Preparación de controles principales: modo, objetivo, longitudes, ruido y tolerancia
tab1 = uitab(tabGroup, 'Title', '⚙️ Control', 'BackgroundColor', [0.95 0.95 0.97]);

yPos = 680;
uicontrol(tab1, 'Style', 'text', 'String', '🎯 MODO DE CONTROL', ...
    'Position', [10 yPos 300 22], 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [0.2 0.4 0.7], 'ForegroundColor', 'w');
yPos = yPos - 30;

ddMode = uicontrol(tab1, 'Style', 'popupmenu', ...
    'String', {'🔵 DLS + PID (cartesiano)', ...
               '🟢 DLS + LQR (cartesiano)', ...
               '🟠 DLS + Fuzzy - Centroid', ...
               '🟡 DLS + Fuzzy - Bisector'}, ...
    'Value', 1, 'Position', [10 yPos 300 26], ...
    'FontSize', 9, 'Callback', @onModeChanged);
yPos = yPos - 45;

uicontrol(tab1, 'Style', 'text', 'String', '📍 OBJETIVO (X, Y, Z)', ...
    'Position', [10 yPos 300 22], 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [0.7 0.2 0.4], 'ForegroundColor', 'w');
yPos = yPos - 35;

uicontrol(tab1, 'Style', 'text', 'String', 'X:', ...
    'Position', [10 yPos 30 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edX = uicontrol(tab1, 'Style', 'edit', 'String', num2str(x_ref), ...
    'Position', [45 yPos 80 24], 'FontSize', 9);
uicontrol(tab1, 'Style', 'text', 'String', 'Y:', ...
    'Position', [135 yPos 30 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edY = uicontrol(tab1, 'Style', 'edit', 'String', num2str(y_ref), ...
    'Position', [170 yPos 80 24], 'FontSize', 9);
yPos = yPos - 30;

uicontrol(tab1, 'Style', 'text', 'String', 'Z:', ...
    'Position', [10 yPos 30 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edZ = uicontrol(tab1, 'Style', 'edit', 'String', num2str(z_ref), ...
    'Position', [45 yPos 80 24], 'FontSize', 9);
yPos = yPos - 40;

uicontrol(tab1, 'Style', 'text', 'String', '📏 ESLABONES', ...
    'Position', [10 yPos 300 22], 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [0.3 0.6 0.3], 'ForegroundColor', 'w');
yPos = yPos - 35;

uicontrol(tab1, 'Style', 'text', 'String', 'L1:', ...
    'Position', [10 yPos 35 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edL1 = uicontrol(tab1, 'Style', 'edit', 'String', num2str(L1), ...
    'Position', [50 yPos 90 24], 'FontSize', 9);
uicontrol(tab1, 'Style', 'text', 'String', 'L2:', ...
    'Position', [155 yPos 35 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edL2 = uicontrol(tab1, 'Style', 'edit', 'String', num2str(L2), ...
    'Position', [195 yPos 90 24], 'FontSize', 9);
yPos = yPos - 40;

chkFollow = uicontrol(tab1, 'Style', 'checkbox', ...
    'String', '🖱️ Seguir mouse (XY)', 'Value', false, ...  % inicial desactivado
    'Position', [10 yPos 280 24], 'FontSize', 9, ...
    'BackgroundColor', [0.95 0.95 0.97]);
yPos = yPos - 30;

uicontrol(tab1, 'Style', 'text', 'String', '🎲 Ruido σ:', ...
    'Position', [10 yPos 80 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edSigma = uicontrol(tab1, 'Style', 'edit', 'String', num2str(sigma_noise), ...
    'Position', [95 yPos 70 24], 'FontSize', 9);
chkNoise = uicontrol(tab1, 'Style', 'checkbox', 'String', 'ON', ...
    'Value', noise_on, 'Position', [175 yPos 80 24], 'FontSize', 9, ...
    'BackgroundColor', [0.95 0.95 0.97]);
yPos = yPos - 35;

uicontrol(tab1, 'Style', 'text', 'String', '🎯 Tolerancia:', ...
    'Position', [10 yPos 100 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edTol = uicontrol(tab1, 'Style', 'edit', 'String', num2str(tol, '%.4f'), ...
    'Position', [115 yPos 90 24], 'FontSize', 9);
yPos = yPos - 50;

% Construcción de botones de control de simulación: ejecutar, detener, reiniciar, batch
btnRun = uicontrol(tab1, 'Style', 'pushbutton', ...
    'String', '▶️  SIMULAR', 'Position', [10 yPos 300 40], ...
    'FontWeight', 'bold', 'FontSize', 11, ...
    'BackgroundColor', [0.2 0.7 0.3], 'ForegroundColor', 'w', ...
    'Callback', @onRun);
yPos = yPos - 45;

btnStop = uicontrol(tab1, 'Style', 'pushbutton', ...
    'String', '⏹️  DETENER', 'Position', [10 yPos 300 35], ...
    'Enable', 'off', 'FontSize', 10, 'Callback', @onStop);
yPos = yPos - 40;

btnReset = uicontrol(tab1, 'Style', 'pushbutton', ...
    'String', '🔄  REINICIAR', 'Position', [10 yPos 300 35], ...
    'FontSize', 10, 'Callback', @onReset);
yPos = yPos - 40;

btnSave = uicontrol(tab1, 'Style', 'pushbutton', ...
    'String', '💾  Guardar Batch', 'Position', [10 yPos 300 35], ...
    'FontSize', 10, 'Callback', @onSaveBatch);
yPos = yPos - 50;

uicontrol(tab1, 'Style', 'text', 'String', 'ℹ️ ESTADO', ...
    'Position', [10 yPos 300 20], 'FontWeight', 'bold', 'FontSize', 9, ...
    'BackgroundColor', [0.5 0.5 0.5], 'ForegroundColor', 'w');
yPos = yPos - 25;

lblInfo = uicontrol(tab1, 'Style', 'text', 'String', '✅ Sistema listo', ...
    'Position', [10 yPos 300 22], 'FontSize', 9, ...
    'BackgroundColor', [0.9 1.0 0.9], 'HorizontalAlignment', 'left');
yPos = yPos - 25;

lblStats = uicontrol(tab1, 'Style', 'text', ...
    'String', 't = 0.00 s | ||e|| = 0.000', ...
    'Position', [10 yPos 300 20], 'FontSize', 8, ...
    'HorizontalAlignment', 'left', 'BackgroundColor', [0.95 0.95 0.97]);
yPos = yPos - 25;

lblIndic = uicontrol(tab1, 'Style', 'text', ...
    'String', 'λ=0.000 | μ=0.000 | κ=0.000', ...
    'Position', [10 yPos 300 20], 'FontSize', 8, ...
    'HorizontalAlignment', 'left', 'BackgroundColor', [0.95 0.95 0.97]);

% --- TAB 2: Parámetros PID/LQR ---
% Preparación de campos para parametrización de PID y LQR, incluyendo tiempos por modo
tab2 = uitab(tabGroup, 'Title', '🎛️ PID/LQR', 'BackgroundColor', [0.95 0.95 0.97]);

yPos = 680;
uicontrol(tab2, 'Style', 'text', 'String', '🔵 CONTROLADOR PID', ...
    'Position', [10 yPos 300 22], 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [0.2 0.4 0.7], 'ForegroundColor', 'w');
yPos = yPos - 35;

uicontrol(tab2, 'Style', 'text', 'String', 'Kp (Proporcional):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edKp = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Kp_def), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 30;

uicontrol(tab2, 'Style', 'text', 'String', 'Ki (Integral):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edKi = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Ki_def), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 30;

uicontrol(tab2, 'Style', 'text', 'String', 'Kd (Derivativo):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edKd = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Kd_def), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 40;

% Configuración de tiempo máximo del modo PID
uicontrol(tab2, 'Style', 'text', 'String', 'T máx PID (s):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edTpid = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Tmax_PID), ...
    'Position', [145 yPos 90 24], 'FontSize', 9, ...
    'Callback', @(~,~) onTimeEdit(1));
yPos = yPos - 50;

uicontrol(tab2, 'Style', 'text', 'String', '🟢 CONTROLADOR LQR', ...
    'Position', [10 yPos 300 22], 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [0.3 0.7 0.3], 'ForegroundColor', 'w');
yPos = yPos - 35;

uicontrol(tab2, 'Style', 'text', 'String', 'Qx (Peso X):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edQx = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Qx_def), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 30;

uicontrol(tab2, 'Style', 'text', 'String', 'Qy (Peso Y):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edQy = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Qy_def), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 30;

uicontrol(tab2, 'Style', 'text', 'String', 'Qz (Peso Z):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edQz = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Qz_def), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 30;

uicontrol(tab2, 'Style', 'text', 'String', 'R (Costo control):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edR = uicontrol(tab2, 'Style', 'edit', 'String', num2str(R_def), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 40;

% Configuración de tiempo máximo del modo LQR
uicontrol(tab2, 'Style', 'text', 'String', 'T máx LQR (s):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edTlqr = uicontrol(tab2, 'Style', 'edit', 'String', num2str(Tmax_LQR), ...
    'Position', [145 yPos 90 24], 'FontSize', 9, ...
    'Callback', @(~,~) onTimeEdit(2));

% --- TAB 3: Parámetros Fuzzy ---
% Configuración de parámetros del modo Fuzzy y métodos de defuzzificación
tab3 = uitab(tabGroup, 'Title', '🟠 Fuzzy', 'BackgroundColor', [0.95 0.95 0.97]);

yPos = 680;
uicontrol(tab3, 'Style', 'text', 'String', '🟠 CONTROL FUZZY', ...
    'Position', [10 yPos 300 22], 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [0.9 0.5 0.1], 'ForegroundColor', 'w');
yPos = yPos - 35;

uicontrol(tab3, 'Style', 'text', 'String', 'K mínima (°):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edKmin = uicontrol(tab3, 'Style', 'edit', 'String', num2str(K_min_deg), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 30;

uicontrol(tab3, 'Style', 'text', 'String', 'K máxima (°):', ...
    'Position', [10 yPos 130 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edKmax = uicontrol(tab3, 'Style', 'edit', 'String', num2str(K_max_deg), ...
    'Position', [145 yPos 90 24], 'FontSize', 9);
yPos = yPos - 40;

uicontrol(tab3, 'Style', 'text', 'String', 'Método defuzzificación:', ...
    'Position', [10 yPos 300 20], 'HorizontalAlignment', 'left', ...
    'FontSize', 9, 'FontWeight', 'bold');
yPos = yPos - 30;

ddDefuzz = uicontrol(tab3, 'Style', 'popupmenu', ...
    'String', {'Centroid', 'Bisector'}, 'Value', 1, ...
    'Position', [10 yPos 200 26], 'FontSize', 9, ...
    'Callback', @(~,~) applyDefuzzToFIS());
yPos = yPos - 40;

% Configuración de tiempos máximos para fuzzy (ambos métodos)
uicontrol(tab3, 'Style', 'text', 'String', 'T máx Fuzzy Centroid (s):', ...
    'Position', [10 yPos 200 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edTfzC = uicontrol(tab3, 'Style', 'edit', 'String', num2str(Tmax_FuzzyC), ...
    'Position', [210 yPos 80 24], 'FontSize', 9, ...
    'Callback', @(~,~) onTimeEdit(3));
yPos = yPos - 30;

uicontrol(tab3, 'Style', 'text', 'String', 'T máx Fuzzy Bisector (s):', ...
    'Position', [10 yPos 200 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edTfzB = uicontrol(tab3, 'Style', 'edit', 'String', num2str(Tmax_FuzzyB), ...
    'Position', [210 yPos 80 24], 'FontSize', 9, ...
    'Callback', @(~,~) onTimeEdit(4));
yPos = yPos - 40;

chkLamSoft = uicontrol(tab3, 'Style', 'checkbox', ...
    'String', '🔧 Lambda adaptativo (μ/κ)', 'Value', lambda_soft_on, ...
    'Position', [10 yPos 280 24], 'FontSize', 9, ...
    'BackgroundColor', [0.95 0.95 0.97]);
yPos = yPos - 35;

uicontrol(tab3, 'Style', 'text', 'String', 'a_μ:', ...
    'Position', [10 yPos 60 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edAMu = uicontrol(tab3, 'Style', 'edit', 'String', num2str(a_mu), ...
    'Position', [75 yPos 70 24], 'FontSize', 9);

uicontrol(tab3, 'Style', 'text', 'String', 'b_κ:', ...
    'Position', [155 yPos 60 20], 'HorizontalAlignment', 'left', 'FontSize', 9);
edBK = uicontrol(tab3, 'Style', 'edit', 'String', num2str(b_kappa), ...
    'Position', [220 yPos 70 24], 'FontSize', 9);

%% ---------- Visualización 3D y gráfica en vivo ----------
% Preparación de ejes: escena 3D y trazado en tiempo real
ax3D  = axes('Parent', fig, 'Position', [0.3 0.20 0.35 0.60]);
axLive = axes('Parent', fig, 'Position', [0.71 0.14 0.27 0.78]);

% Almacenamiento de tiempos por modo en appdata
timeStruct = struct('pid', Tmax_PID, ...
                    'lqr', Tmax_LQR, ...
                    'fzC', Tmax_FuzzyC, ...
                    'fzB', Tmax_FuzzyB);
setappdata(fig, 'modeTimes', timeStruct);

% Inicialización del gráfico en vivo con el eje X según el modo actual
setupLivePlot(current_live_xlim);

% Inicialización de banderas y primer render de workspace y brazo
setappdata(fig, 'stopFlag', false);
drawWorkspace3D(ax3D, RANGE, rmin, rmax);
[x1, y1, z1, x2, y2, z2, xe, ye, ze] = fk3d(theta1, theta2, theta3, L1, L2);
[hL1, hL2, hEE, hGoal, ~, hProj] = drawArm3D(ax3D, ...
    x1, y1, z1, x2, y2, z2, ...
    xe, ye, ze, x_ref, y_ref, z_ref, RANGE);

% Activación del modo seleccionado (habilitación de controles y eje X)
onModeChanged();

% Vinculación de callbacks de interacción con el mouse (seguir, arrastre Z y rueda)
set(fig, 'WindowButtonMotionFcn', @(~,~) onMouseMove());
set(fig, 'WindowButtonDownFcn', @onMouseDown);
set(fig, 'WindowButtonUpFcn',   @onMouseUp);
set(fig, 'WindowScrollWheelFcn', @(~, evt) onScroll(evt));

%% ================= FUNCIONES INTERNAS (NESTED) =================
    function setupLivePlot(xmax)
        % Preparación de ejes y series para la gráfica en vivo (x(t), y(t), z(t) y referencias)
        cla(axLive);
        grid(axLive, 'on');
        title(axLive, 'Posición Cartesiana (Tiempo Real)', 'FontWeight', 'bold');
        xlabel(axLive, 'Tiempo (s)');
        ylabel(axLive, 'Posición (unidades)');
        hold(axLive, 'on');

        xlim(axLive, [0 xmax]);
        ylim(axLive, [-RANGE RANGE]);

        % Definición de paleta de trazado
        colX   = [0 0.447 0.741];
        colY   = [0.2 0.7 0.2];
        colZ   = [0.85 0.33 0.10];
        colRef = [0.5 0.5 0.5];

        % Construcción de series
        hX = plot(axLive, NaN, NaN, 'LineWidth', 2.0, 'Color', colX, 'DisplayName', 'x(t)');
        hY = plot(axLive, NaN, NaN, 'LineWidth', 2.0, 'Color', colY, 'DisplayName', 'y(t)');
        hZ = plot(axLive, NaN, NaN, 'LineWidth', 2.0, 'Color', colZ, 'DisplayName', 'z(t)');

        % Construcción de referencias como líneas horizontales
        hXref = yline(axLive, x_ref, '--', 'Color', colRef, 'LineWidth', 1.5, ...
            'DisplayName', 'x_{ref}', 'Alpha', 0.6);
        hYref = yline(axLive, y_ref, '--', 'Color', colRef, 'LineWidth', 1.5, ...
            'DisplayName', 'y_{ref}', 'Alpha', 0.6);
        hZref = yline(axLive, z_ref, '--', 'Color', colRef, 'LineWidth', 1.5, ...
            'DisplayName', 'z_{ref}', 'Alpha', 0.6);

        legend(axLive, 'Location', 'best', 'FontSize', 8);

        % Almacenamiento de handles para actualización rápida
        setappdata(fig, 'plotHandles', struct( ...
            'hX', hX, 'hY', hY, 'hZ', hZ, ...
            'hXref', hXref, 'hYref', hYref, 'hZref', hZref));
        setappdata(fig, 'live_xlim', xmax);
    end

    function onTimeEdit(whichMode)
        % Actualización del tiempo máximo según el modo editado (1=PID, 2=LQR, 3=FuzzyC, 4=FuzzyB)
        timeStruct = getappdata(fig, 'modeTimes');
        switch whichMode
            case 1
                val = str2double(get(edTpid, 'String'));
                if isnan(val) || val <= 0, val = 15; end
                timeStruct.pid = val;
            case 2
                val = str2double(get(edTlqr, 'String'));
                if isnan(val) || val <= 0, val = 12; end
                timeStruct.lqr = val;
            case 3
                val = str2double(get(edTfzC, 'String'));
                if isnan(val) || val <= 0, val = 15; end
                timeStruct.fzC = val;
            case 4
                val = str2double(get(edTfzB, 'String'));
                if isnan(val) || val <= 0, val = 15; end
                timeStruct.fzB = val;
        end
        setappdata(fig, 'modeTimes', timeStruct);

        % Sincronización inmediata del eje X si el modo visible coincide
        curMode = get(ddMode, 'Value');
        xmax = getModeTime(curMode, timeStruct);
        xlim(axLive, [0 xmax]);
        setappdata(fig, 'live_xlim', xmax);
    end

    function xmax = getModeTime(modeVal, tstruct)
        % Devolución del tiempo máximo vinculado al modo activo
        switch modeVal
            case 1, xmax = tstruct.pid;
            case 2, xmax = tstruct.lqr;
            case 3, xmax = tstruct.fzC;
            case 4, xmax = tstruct.fzB;
            otherwise, xmax = 15;
        end
    end

    function onModeChanged(~, ~)
        % Configuración del estado de los paneles según el modo seleccionado
        modeVal = get(ddMode, 'Value');

        % Habilitación de grupos de parámetros por controlador
        set([edKp edKi edKd], 'Enable', tern(modeVal==1, 'on', 'inactive'));
        set([edQx edQy edQz edR], 'Enable', tern(modeVal==2, 'on', 'inactive'));
        set([edKmin edKmax ddDefuzz edTfzC edTfzB], 'Enable', tern(modeVal>=3, 'on', 'inactive'));

        % Indicador de modo mediante color de fondo
        colors = { [0.8 0.9 1.0], ...
                   [0.8 1.0 0.8], ...
                   [1.0 0.95 0.8], ...
                   [1.0 0.98 0.85]};
        set(lblInfo, 'BackgroundColor', colors{modeVal});

        % Sincronización del eje X de la gráfica en vivo con el tiempo por modo
        tstruct = getappdata(fig, 'modeTimes');
        xmax = getModeTime(modeVal, tstruct);
        xlim(axLive, [0 xmax]);
        setappdata(fig, 'live_xlim', xmax);
    end

    function applyDefuzzToFIS()
        % Aplicación del método de defuzzificación seleccionado al FIS si está disponible
        try
            opts = get(ddDefuzz, 'String');
            sel  = lower(opts{get(ddDefuzz, 'Value')});
            if isstruct(fis1) && ~(isfield(fis1, 'Stub') && fis1.Stub)
                fis1.DefuzzificationMethod = sel;
            end
        catch
            % Silencioso: evita interrupciones si el FIS no está disponible
        end
    end

    function onRun(~, ~)
        % Lectura de parámetros de escena, tolerancia, tiempos por modo y opciones del controlador
        L1v = str2double(get(edL1, 'String'));
        L2v = str2double(get(edL2, 'String'));
        Xv  = str2double(get(edX, 'String'));
        Yv  = str2double(get(edY, 'String'));
        Zv  = str2double(get(edZ, 'String'));

        if any(isnan([L1v L2v Xv Yv Zv])) || L1v <= 0 || L2v <= 0
            updateInfo('⚠️ Entradas inválidas', [1 0.8 0.8]);
            return;
        end

        % Actualización de eslabones y rangos de alcanzabilidad
        L1 = L1v; L2 = L2v;
        rmin = abs(L1 - L2);
        rmax =  L1 + L2;
        RANGE = rmax + 4;

        % Lectura/validación de tolerancia
        tol_in = str2double(get(edTol, 'String'));
        if ~isfinite(tol_in) || tol_in <= 0, tol_in = 0.02; end
        tol = tol_in;
        set(edTol, 'String', num2str(tol, '%.4f'));

        % Resolución del tiempo máximo para el modo seleccionado
        tstruct = getappdata(fig, 'modeTimes');
        modeIdx = get(ddMode, 'Value');
        Tmax_this = getModeTime(modeIdx, tstruct);
        max_iter = ceil(Tmax_this / dt_step_s);
        if max_iter < 50, max_iter = 50; end  % mínimo razonable

        % Limitación del objetivo a la zona alcanzable
        [x_ref, y_ref, z_ref] = clampTarget3DAll(Xv, Yv, Zv, rmin, rmax);
        set(edX, 'String', num2str(x_ref, '%.2f'));
        set(edY, 'String', num2str(y_ref, '%.2f'));
        set(edZ, 'String', num2str(z_ref, '%.2f'));

        % Lectura de parámetros Fuzzy (Kmin/Kmax y defuzzificación)
        Kmin_try = str2double(get(edKmin, 'String'));
        Kmax_try = str2double(get(edKmax, 'String'));
        if ~isnan(Kmin_try), K_min_deg = max(0, Kmin_try); end
        if ~isnan(Kmax_try), K_max_deg = max(K_min_deg + 0.1, Kmax_try); end
        applyDefuzzToFIS();

        % Reinicio de escena 3D y redibujado del brazo
        cla(ax3D);
        drawWorkspace3D(ax3D, RANGE, rmin, rmax);
        [x1, y1, z1, x2, y2, z2, xe, ye, ze] = fk3d(theta1, theta2, theta3, L1, L2);
        [hL1, hL2, hEE, hGoal, ~, hProj] = drawArm3D(ax3D, ...
            x1, y1, z1, x2, y2, z2, ...
            xe, ye, ze, x_ref, y_ref, z_ref, RANGE);

        % Reinicio de gráfica en vivo y referencias
        handles   = getappdata(fig, 'plotHandles');
        live_xlim = getappdata(fig, 'live_xlim');
        xlim(axLive, [0 live_xlim]);
        ylim(axLive, [-RANGE RANGE]);
        set(handles.hX, 'XData', NaN, 'YData', NaN);
        set(handles.hY, 'XData', NaN, 'YData', NaN);
        set(handles.hZ, 'XData', NaN, 'YData', NaN);
        set(handles.hXref, 'Value', x_ref);
        set(handles.hYref, 'Value', y_ref);
        set(handles.hZref, 'Value', z_ref);

        % Preparación de buffers para registro de historiales
        T_hist   = nan(max_iter, 1);
        err_hist = nan(max_iter, 1);
        XE_hist  = nan(max_iter, 1);
        YE_hist  = nan(max_iter, 1);
        ZE_hist  = nan(max_iter, 1);
        Kx_hist  = nan(max_iter, 1);
        Ky_hist  = nan(max_iter, 1);
        Kz_hist  = nan(max_iter, 1);

        % Inicialización de variables del lazo
        dtheta_prev  = [0; 0; 0];
        stable_count = 0;

        % Lectura de PID
        Kp = str2double(get(edKp, 'String')); if isnan(Kp), Kp = Kp_def; end
        Ki = str2double(get(edKi, 'String')); if isnan(Ki), Ki = Ki_def; end
        Kd = str2double(get(edKd, 'String')); if isnan(Kd), Kd = Kd_def; end
        e_prev = [0; 0; 0];
        ei     = [0; 0; 0];
        ei_max = 5;

        % Lectura de opciones y ruido
        noise_on       = logical(get(chkNoise, 'Value'));
        sigma_noise    = valOrZero(get(edSigma, 'String'));
        lambda_soft_on = logical(get(chkLamSoft, 'Value'));
        a_mu           = valOrZero(get(edAMu, 'String'));
        b_kappa        = valOrZero(get(edBK, 'String'));

        % Lectura de LQR y construcción de ganancia K
        Qx = posOrDefault(get(edQx, 'String'), Qx_def);
        Qy = posOrDefault(get(edQy, 'String'), Qy_def);
        Qz = posOrDefault(get(edQz, 'String'), Qz_def);
        Rv = posOrDefault(get(edR,  'String'), R_def);
        Ad = eye(3); Bd = eye(3);
        Q  = diag([Qx Qy Qz]);
        R  = Rv * eye(3);
        K_lqr = compute_dlqr_safe(Ad, Bd, Q, R);

        % Actualización de la UI de estado
        set(btnRun,  'Enable', 'off');
        set(btnStop, 'Enable', 'on');
        updateInfo('⏳ Simulando...', [1 1 0.8]);
        setappdata(fig, 'stopFlag', false);

        % Frecuencia de actualización visual reducida
        update_counter  = 0;
        update_interval = 3;

        % Bucle de simulación principal
        for k = 1:max_iter
            % Seguimiento del mouse opcional (XY en plano z_ref)
            if get(chkFollow, 'Value')
                [xr, yr, insideAxes] = getMouseOnZ(ax3D, z_ref);
                if insideAxes
                    [xr, yr] = clampXYGivenZ(xr, yr, z_ref, rmin, rmax);
                    x_ref = xr; y_ref = yr;
                    set(hGoal, 'XData', x_ref, 'YData', y_ref, 'ZData', z_ref);
                    set(edX, 'String', num2str(x_ref, '%.2f'));
                    set(edY, 'String', num2str(y_ref, '%.2f'));
                    set(handles.hXref, 'Value', x_ref);
                    set(handles.hYref, 'Value', y_ref);
                end
            end

            % Cinemática directa y error cartesiano
            [x1, y1, z1, x2, y2, z2, xe, ye, ze] = fk3d(theta1, theta2, theta3, L1, L2);
            e = [x_ref - xe; y_ref - ye; z_ref - ze];

            % Inyección de ruido (si está activo)
            if noise_on && sigma_noise > 0
                e = e + sigma_noise * randn(3, 1);
            end

            % Registro de métricas de error y trayectoria
            err          = norm(e);
            err_hist(k)  = err;
            XE_hist(k)   = xe;
            YE_hist(k)   = ye;
            ZE_hist(k)   = ze;
            T_hist(k)    = (k - 1) * dt_step_s;

            % Detección de estabilidad persistente
            if err < tol
                stable_count = stable_count + 1;
                if stable_count >= stable_needed
                    updateInfo(sprintf('✅ Convergencia (||e|| = %.4f)', err), [0.8 1 0.8]);
                    finalizePlots(k, modeIdx, ...
                        Kx_hist, Ky_hist, Kz_hist, err_hist, T_hist);
                    break;
                end
            else
                stable_count = 0;
            end

            % Cálculo de Jacobiano y regularización DLS
            J   = jacobian3d(theta1, theta2, theta3, L1, L2);
            JJt = J * J.';

            % Ajuste de lambda según error, singularidad y condición
            lam = lambda_base;
            if err < err_small_th
                lam = lam + lambda_smallE;
            end

            detJJt = det(JJt);
            if abs(detJJt) < singularity_threshold
                lam = lam + 0.5;
            end

            cJJt = cond(JJt);
            if ~isfinite(cJJt) || cJJt > 1e3
                lam = lam + cond_boost;
            end

            % Cálculo de indicadores μ y κ para lambda adaptativo
            mu    = sqrt(max(eps, abs(detJJt)));
            kappa = cJJt;

            if lambda_soft_on
                muN  = max(0, min(1, mu / max(eps, L1 * L2)));
                kapN = max(0, min(1, (log10(max(1, kappa))) / 3));
                lam  = lam + a_mu * (1 - muN) + b_kappa * kapN;
            end

            % Ley de control según modo
            switch modeIdx
                case 1  % PID
                    e_dot = (e - e_prev);
                    ei    = max(-ei_max, min(ei_max, ei + e));
                    u_cart = Kp * e + Ki * ei + Kd * e_dot;
                case 2  % LQR
                    u_cart = +K_lqr * e;
                case {3, 4}  % Fuzzy
                    u_cart = e;
            end
            e_prev = e;

            % Conversión cartesiano→articular mediante DLS
            A = JJt + (lam^2) * eye(3);
            cart2joint = (J.' * (A \ u_cart));

            % Escalado adaptativo de incremento articular con Fuzzy
            if modeIdx >= 3
                [Kdeg, Kxdeg, Kydeg, Kzdeg] = ...
                    getKdegAndComponents(fis1, e, K_min_deg, K_max_deg);
                Ktheta = deg2rad(Kdeg);
                D = diag([Ktheta, Ktheta, Ktheta]);
                Kx_hist(k) = Kxdeg;
                Ky_hist(k) = Kydeg;
                Kz_hist(k) = Kzdeg;
            else
                D = eye(3);
            end

            % Suavizado, saturación de paso y actualización articular
            dtheta  = D * cart2joint;
            dtheta  = beta * dtheta_prev + (1 - beta) * dtheta;
            dtheta_prev = dtheta;

            Ks_ang = deg2rad(K_safety_ang_deg);
            dtheta = max(-Ks_ang, min(Ks_ang, dtheta));

            theta1 = wrapToPi_local(theta1 + dtheta(1));
            theta2 = clampJoint(theta2 + dtheta(2), JOINT_LIMITS.theta2);
            theta3 = clampJoint(theta3 + dtheta(3), JOINT_LIMITS.theta3);

            % Actualización visual a tasa reducida
            update_counter = update_counter + 1;
            if update_counter >= update_interval
                update_counter = 0;

                [x1, y1, z1, x2, y2, z2, xe, ye, ze] = ...
                    fk3d(theta1, theta2, theta3, L1, L2);
                set(hL1, 'XData', [0 x1], 'YData', [0 y1], 'ZData', [0 z1]);
                set(hL2, 'XData', [x1 x2], 'YData', [y1 y2], 'ZData', [z1 z2]);
                set(hEE, 'XData', xe, 'YData', ye, 'ZData', ze);
                updateProjections(hProj, xe, ye, ze);

                set(handles.hX, 'XData', T_hist(1:k), 'YData', XE_hist(1:k));
                set(handles.hY, 'XData', T_hist(1:k), 'YData', YE_hist(1:k));
                set(handles.hZ, 'XData', T_hist(1:k), 'YData', ZE_hist(1:k));
                set(handles.hZref, 'Value', z_ref);

                set(lblIndic, 'String', ...
                    sprintf('λ=%.3f | μ=%.3f | κ=%.1f', lam, mu, kappa));
                set(lblStats, 'String', ...
                    sprintf('t=%.2fs | ||e||=%.3f | estable=%d/%d', ...
                        T_hist(k), err, stable_count, stable_needed));

                drawnow limitrate;
            end

            % Ritmo de simulación y condición de parada por usuario
            pause(dt_step_s);
            if getappdata(fig, 'stopFlag')
                updateInfo('⏹️ Detenido por usuario', [1 0.9 0.8]);
                finalizePlots(k, modeIdx, ...
                    Kx_hist, Ky_hist, Kz_hist, err_hist, T_hist);
                break;
            end
        end

        % Mensaje de cierre si se alcanzó el máximo de iteraciones
        if k >= max_iter
            updateInfo(sprintf('⏱️ Máx. iteraciones (||e|| = %.4f)', err), [1 1 0.8]);
        end

        % Restauración de botones
        set(btnRun,  'Enable', 'on');
        set(btnStop, 'Enable', 'off');
    end

    function finalizePlots(kend, modeIdx, Kx_hist, Ky_hist, Kz_hist, err_hist, T_hist)
        % Construcción de gráfica post-simulación: norma de error cartesiano vs tiempo
        if kend >= 1 && any(~isnan(err_hist(1:kend))) && any(~isnan(T_hist(1:kend)))
            tvec = T_hist(1:kend);

            figErr = figure('Name', 'Análisis de Error - Post Simulación', ...
                'Color', 'w', 'Position', [100 100 900 450]);

            plot(tvec, err_hist(1:kend), ...
            'LineWidth', 2, ...
            'Color', [0.8 0.2 0.2], ...
            'DisplayName', 'Error');
        
            grid on; hold on;
            
            yline(tol, '--k', 'LineWidth', 1.5, 'DisplayName', 'Tolerancia');
            
            xlabel('Tiempo (s)', 'FontSize', 14);
            ylabel('‖e‖ (unidades)', 'FontSize', 14);
            title('Norma del Error Cartesiano vs Tiempo', 'FontWeight', 'bold', 'FontSize', 14);
            
            legend('Location', 'best', 'FontSize', 14);

        end

        % Construcción de gráfica de ganancias adaptativas (solo Fuzzy)
        if (modeIdx == 3 || modeIdx == 4) && kend >= 1
            if any(~isnan(Kx_hist(1:kend))) || any(~isnan(Ky_hist(1:kend))) || any(~isnan(Kz_hist(1:kend)))
                tvec = T_hist(1:kend);
                figK = figure('Name', 'Ganancias Fuzzy Adaptativas', ...
                    'Color', 'w', 'Position', [150 150 900 450]);

                plot(tvec, Kx_hist(1:kend), 'LineWidth', 2, 'Color', [0.8 0.2 0.2], 'DisplayName', 'Kx');
                hold on; grid on;
                plot(tvec, Ky_hist(1:kend), 'LineWidth', 2, 'Color', [0.2 0.7 0.2], 'DisplayName', 'Ky');
                plot(tvec, Kz_hist(1:kend), 'LineWidth', 2, 'Color', [0.2 0.4 0.8], 'DisplayName', 'Kz');

                xlabel('Tiempo (s)', 'FontSize', 14);
                ylabel('Ganancia K (grados)', 'FontSize', 14);
                title('Evolución de Ganancias Adaptativas (Fuzzy)', 'FontWeight', 'bold','FontSize', 14);
                legend('Location', 'best', 'FontSize', 14);
            end
        end
    end

    function updateInfo(msg, color)
        % Actualización del panel de estado con mensaje y color de fondo
        set(lblInfo, 'String', msg, 'BackgroundColor', color);
    end

    function onStop(~, ~)
        % Señalización de parada por usuario
        setappdata(fig, 'stopFlag', true);
    end

    function onReset(~, ~)
        % Restauración de valores por defecto del sistema y de la interfaz
        L1 = 10; L2 = 7;
        theta1 = deg2rad(0);
        theta2 = deg2rad(15);
        theta3 = deg2rad(0);
        x_ref  = 10;
        y_ref  = -14;
        z_ref  = -4;
        rmin   = abs(L1 - L2);
        rmax   = L1 + L2;
        RANGE  = rmax + 4;

        % Restablecimiento de tiempos por modo y sincronización de campos
        Tmax_PID     = 3;
        Tmax_LQR     = 3;
        Tmax_FuzzyC  = 15;
        Tmax_FuzzyB  = 15;
        tstruct = struct('pid', Tmax_PID, 'lqr', Tmax_LQR, ...
                         'fzC', Tmax_FuzzyC, 'fzB', Tmax_FuzzyB);
        setappdata(fig, 'modeTimes', tstruct);

        set(edTpid, 'String', num2str(Tmax_PID));
        set(edTlqr, 'String', num2str(Tmax_LQR));
        set(edTfzC, 'String', num2str(Tmax_FuzzyC));
        set(edTfzB, 'String', num2str(Tmax_FuzzyB));

        % Restablecimiento de controles del panel principal
        set(edL1, 'String', num2str(L1));
        set(edL2, 'String', num2str(L2));
        set(edX, 'String', num2str(x_ref));
        set(edY, 'String', num2str(y_ref));
        set(edZ, 'String', num2str(z_ref));
        set(chkFollow, 'Value', false);
        set(chkNoise,  'Value', false);
        set(edSigma,   'String', '0');
        set(chkLamSoft, 'Value', true);
        set(edAMu, 'String', '0');
        set(edBK, 'String', '0');
        set(ddMode, 'Value', 1);
        onModeChanged();

        % Restablecimiento de parámetros PID/LQR/Fuzzy
        set(edKp, 'String', num2str(Kp_def));
        set(edKi, 'String', num2str(Ki_def));
        set(edKd, 'String', num2str(Kd_def));
        set(edQx, 'String', num2str(Qx_def));
        set(edQy, 'String', num2str(Qy_def));
        set(edQz, 'String', num2str(Qz_def));
        set(edR,  'String', num2str(R_def));
        set(edKmin, 'String', '0.5');
        set(edKmax, 'String', '7.0');
        set(ddDefuzz, 'Value', 1);
        applyDefuzzToFIS();
        set(edTol, 'String', '0.02');

        % Restauración de botones y etiquetas
        set(btnRun,  'Enable', 'on');
        set(btnStop, 'Enable', 'off');
        updateInfo('✅ Sistema reiniciado', [0.9 1.0 0.9]);
        set(lblStats, 'String', 't = 0.00 s | ||e|| = 0.000');
        set(lblIndic, 'String', 'λ=0.000 | μ=0.000 | κ=0.000');

        % Reinicio de visualización 3D y gráfico en vivo
        cla(ax3D);
        drawWorkspace3D(ax3D, RANGE, rmin, rmax);
        [x1, y1, z1, x2, y2, z2, xe, ye, ze] = ...
            fk3d(theta1, theta2, theta3, L1, L2);
        [hL1, hL2, hEE, hGoal, ~, hProj] = drawArm3D(ax3D, ...
            x1, y1, z1, x2, y2, z2, ...
            xe, ye, ze, x_ref, y_ref, z_ref, RANGE);

        setupLivePlot(Tmax_PID); % retorno a la escala temporal del modo PID
    end

    function onSaveBatch(~, ~)
    % Preparación y ejecución de corridas automáticas (batch) con guardado de resultados
    outdir = uigetdir('', 'Selecciona carpeta para guardar resultados batch');
    if isequal(outdir, 0)
        updateInfo('❌ Guardado cancelado', [1 0.9 0.8]);
        return;
    end

    % Lectura y validación de eslabones
    L1v = str2double(get(edL1, 'String'));
    L2v = str2double(get(edL2, 'String'));
    if any(isnan([L1v L2v]))
        updateInfo('⚠️ L1/L2 inválidos', [1 0.8 0.8]);
        return;
    end
    L1 = L1v; L2 = L2v;
    rmin = abs(L1 - L2); rmax = L1 + L2;

    % Definición del plano Z para los objetivos del batch
    z_plane = str2double(get(edZ, 'String'));
    if isnan(z_plane), z_plane = 0; end

    % Cálculo de radios alcanzables en el plano z
    rho_max = sqrt(max(rmax^2 - z_plane^2, 0));
    rho_min = sqrt(max(rmin^2 - z_plane^2, 0));

    if rho_max < 1e-6
        updateInfo('⚠️ No hay alcanzabilidad en ese z', [1 0.8 0.8]);
        return;
    end

    % Construcción de objetivos (N puntos en círculo)
    rho = max(rho_min, 0.75 * rho_max);
    NPTS = 12;
    ths  = linspace(0, 2*pi, NPTS + 1); ths(end) = [];
    targets = zeros(NPTS, 3);

    for i = 1:NPTS
        [xx, yy] = clampXYGivenZ(rho * cos(ths(i)), rho * sin(ths(i)), ...
                                 z_plane, rmin, rmax);
        targets(i, :) = [xx, yy, z_plane];
    end

    % Captura de opciones para batch (ruido, lambda adaptativo y PID/LQR/Fuzzy)
    noise_on_b       = logical(get(chkNoise, 'Value'));
    sigma_noise_b    = valOrZero(get(edSigma, 'String'));
    lambda_soft_on_b = logical(get(chkLamSoft, 'Value'));
    a_mu_b           = valOrZero(get(edAMu, 'String'));
    b_kappa_b        = valOrZero(get(edBK, 'String'));

    Kp_b = str2double(get(edKp, 'String')); if isnan(Kp_b), Kp_b = Kp_def; end
    Ki_b = str2double(get(edKi, 'String')); if isnan(Ki_b), Ki_b = Ki_def; end
    Kd_b = str2double(get(edKd, 'String')); if isnan(Kd_b), Kd_b = Kd_def; end

    Qx_b = posOrDefault(get(edQx, 'String'), Qx_def);
    Qy_b = posOrDefault(get(edQy, 'String'), Qy_def);
    Qz_b = posOrDefault(get(edQz, 'String'), Qz_def);
    Rv_b = posOrDefault(get(edR,  'String'), R_def);

    Kmin_b = str2double(get(edKmin, 'String'));
    if isnan(Kmin_b), Kmin_b = K_min_deg; end
    Kmax_b = str2double(get(edKmax, 'String'));
    if isnan(Kmax_b), Kmax_b = K_max_deg; end

    tol_b = str2double(get(edTol, 'String'));
    if ~isfinite(tol_b) || tol_b <= 0, tol_b = 0.02; end

    % Definición de modos incluidos en el batch
    modeIds   = [1 2 3 4];
    modeNames = {'PID', 'LQR', 'Fuzzy-Centroid', 'Fuzzy-Bisector'};

    % Inicialización de estructura de resultados
    init_theta = [theta1; theta2; theta3];
    rows = [];

    updateInfo('⏳ Ejecutando batch...', [1 1 0.8]);

    % Construcción de barra de progreso
    wb = waitbar(0, 'Ejecutando simulaciones batch...');
    totalSims = numel(modeIds) * NPTS;
    simCount  = 0;
    % Bucle por modos y objetivos
    for m = 1:numel(modeIds)
        modeIdx_batch = modeIds(m);
        tcur = init_theta;

        % Ajuste del método de defuzzificación para el modo Fuzzy
        fisBatch = fis1;
        if modeIdx_batch == 3
            try, fisBatch.DefuzzificationMethod = 'centroid'; end
        elseif modeIdx_batch == 4
            try, fisBatch.DefuzzificationMethod = 'bisector'; end
        end

        for i = 1:NPTS
            tgt = targets(i, :);

            % Preparación de configuración de simulación individual
            cfg = struct( ...
                'L1', L1, 'L2', L2, ...
                'max_iter', 1000, 'tol', tol_b, ...
                'stable_needed', stable_needed, 'beta', beta, ...
                'K_min_deg', Kmin_b, 'K_max_deg', Kmax_b, ...
                'K_safety_ang_deg', K_safety_ang_deg, ...
                'lambda_base', lambda_base, 'lambda_smallE', lambda_smallE, ...
                'err_small_th', err_small_th, 'cond_boost', cond_boost, ...
                'noise_on', noise_on_b, 'sigma_noise', sigma_noise_b, ...
                'lambda_soft_on', lambda_soft_on_b, ...
                'a_mu', a_mu_b, 'b_kappa', b_kappa_b, ...
                'modeIdx', modeIdx_batch, 'fis1', fisBatch, ...
                'Kp', Kp_b, 'Ki', Ki_b, 'Kd', Kd_b, ...
                'Qx', Qx_b, 'Qy', Qy_b, 'Qz', Qz_b, 'R', Rv_b, ...
                'dt_step_s', dt_step_s, ...
                'JOINT_LIMITS', JOINT_LIMITS);

            % Ejecución
            sim = simulateOnce(tcur, tgt, cfg);
            tcur = sim.theta_end(:);

            % Recolección de resultados
            r = struct();
            r.pt         = i;
            r.mode       = modeNames{m};
            r.success    = sim.success;
            r.steps      = sim.steps;
            r.time_sim_s = sim.time_sim_s;
            r.time_wall_s = sim.time_wall_s;
            r.err_final  = sim.err_final;
            r.x_ref      = tgt(1);
            r.y_ref      = tgt(2);
            r.z_ref      = tgt(3);
            r.xe_end     = sim.ee_end(1);
            r.ye_end     = sim.ee_end(2);
            r.ze_end     = sim.ee_end(3);
            r.t1_ini_deg = rad2deg(sim.theta_ini(1));
            r.t2_ini_deg = rad2deg(sim.theta_ini(2));
            r.t3_ini_deg = rad2deg(sim.theta_ini(3));
            r.t1_end_deg = rad2deg(sim.theta_end(1));
            r.t2_end_deg = rad2deg(sim.theta_end(2));
            r.t3_end_deg = rad2deg(sim.theta_end(3));
            r.Kp         = Kp_b;
            r.Ki         = Ki_b;
            r.Kd         = Kd_b;
            r.Qx         = Qx_b;
            r.Qy         = Qy_b;
            r.Qz         = Qz_b;
            r.R          = Rv_b;

            rows = [rows; r]; %#ok<AGROW>

            simCount = simCount + 1;
            waitbar(simCount / totalSims, wb, ...
                sprintf('Progreso: %d/%d (%s - Punto %d)', ...
                    simCount, totalSims, modeNames{m}, i));
        end
    end
    close(wb);

    % Conversión a tabla
    T_all = struct2table(rows);

    % Guardado
    ts = datestr(now, 'yyyymmdd_HHMMSS');
    csvfile = fullfile(outdir, sprintf('batch_results_%s.csv', ts));

    try
        writetable(T_all, csvfile);
        assignin('base', 'T_batch', T_all);
    catch
        warning('No se pudo escribir CSV');
    end

    % Construcción de gráfico comparativo
    modesList = unique(T_all.mode, 'stable');
    palette = [0   158 115; ...
               213 94  0;  ...
               0   114 178; ...
               230 159 0] / 255;

    avg_time = zeros(numel(modesList), 1);
    std_time = zeros(numel(modesList), 1);

    for m = 1:numel(modesList)
        idx = strcmp(T_all.mode, modesList{m});
        avg_time(m) = mean(T_all.time_sim_s(idx), 'omitnan');
        std_time(m) = std(T_all.time_sim_s(idx), 'omitnan');
    end

    figBatch = figure('Name', 'Comparación de Controladores (Batch)', ...
        'Color', 'w', 'Position', [200 200 900 600]);

    hold on; grid on;
    for m = 1:numel(modesList)
        errorbar(m, avg_time(m), std_time(m), 'o', ...
            'Color', palette(m, :), ...
            'MarkerEdgeColor', palette(m, :), ...
            'MarkerFaceColor', palette(m, :), ...
            'LineStyle', 'none', ...
            'LineWidth', 2, ...
            'CapSize', 12, ...
            'MarkerSize', 10);
    end


    saveas(figBatch, fullfile(outdir, sprintf('comparison_plot_%s.png', ts)));
    %% ============================================================
    %  FIGURAS DEL BATCH – PID, LQR, FIS-C, FIS-B
    %% ============================================================
    % --- columnas de diferencias para cada eje ---
    T_all.dX = T_all.x_ref - T_all.xe_end;
    T_all.dY = T_all.y_ref - T_all.ye_end;
    T_all.dZ = T_all.z_ref - T_all.ze_end;
    

    modesList = {'PID','LQR','Fuzzy-Centroid','Fuzzy-Bisector'};
    modeShort = {'PID','LQR','FIS-C','FIS-B'};
    palette = lines(4);

    % --- PUNTOS (CORREGIDO: USAR 'pt', NO 'point') ---
    pts = unique(T_all.pt);


    %% ============================================================
    % FIGURA 1 — Pasos (iteraciones) por punto y por modo
    %% ============================================================

    fig1 = figure('Name','Pasos por punto','Color','w','Position',[200 200 900 600]);
    hold on; grid on;

    for m = 1:numel(modesList)
        idx = strcmp(T_all.mode, modesList{m});
        plot(T_all.pt(idx), T_all.steps(idx), '-o', ...
            'Color', palette(m,:), ...
            'MarkerFaceColor', palette(m,:), ...
            'LineWidth', 2, 'MarkerSize', 6, ...
            'DisplayName', modeShort{m});
    end

xlabel('Punto','FontSize',16);
ylabel('Pasos','FontSize',16);
title('Pasos (iteraciones) por punto','FontWeight','bold','FontSize',16);
legend('Location','best','FontSize',14);


    saveas(fig1, fullfile(outdir, 'batch_steps_per_point.png'));
    saveas(fig1, fullfile(outdir, 'batch_steps_per_point.eps'), 'epsc');


    %% ============================================================
    % FIGURA 2 — Error final ||e||_final por punto y por modo
    %% ============================================================

    fig2 = figure('Name','Error final por punto','Color','w','Position',[200 200 900 600]);
    hold on; grid on;

    bw = 0.18;  % ancho barras

    for m = 1:numel(modesList)
        idx = strcmp(T_all.mode, modesList{m});
        bar(pts + (m-2.5)*bw, T_all.err_final(idx), ...
            'FaceColor', palette(m,:), ...
            'BarWidth', bw, ...
            'DisplayName', modeShort{m});
    end

xlabel('Punto','FontSize',16);
ylabel('||e||_{final}','FontSize',16);
title('Error final por punto','FontWeight','bold','FontSize',16);
legend('Location','best','FontSize',14);


    saveas(fig2, fullfile(outdir, 'batch_err_final_per_point.png'));
    saveas(fig2, fullfile(outdir, 'batch_err_final_per_point.eps'), 'epsc');


    %% ============================================================
% FIGURA 3 — |ΔX|, |ΔY|, |ΔZ| por punto y por modo
%% ============================================================

%% ============================================================
% FIGURA 3 — |ΔX|, |ΔY|, |ΔZ| por punto y por modo  (CORREGIDA)
%% ============================================================

% --- columnas de diferencias (si no existen) ---
T_all.dX = T_all.x_ref - T_all.xe_end;
T_all.dY = T_all.y_ref - T_all.ye_end;
T_all.dZ = T_all.z_ref - T_all.ze_end;

% --- puntos (usando la columna correcta) ---
pts = unique(T_all.pt);

fig3 = figure('Name','Diferencias eje por punto', ...
    'Color','w','Position',[200 200 900 900]);

axesNames = {'|ΔX|','|ΔY|','|ΔZ|'};
fields    = {'dX','dY','dZ'};     % columnas reales

for ax = 1:3
    subplot(3,1,ax);
    hold on; grid on;

    for m = 1:numel(modesList)
        idx = strcmp(T_all.mode, modesList{m});
        bar(pts + (m-2.5)*bw, abs(T_all.(fields{ax})(idx)), ...
            'FaceColor', palette(m,:), ...
            'BarWidth', bw, ...
            'DisplayName', modeShort{m});
    end

    ylabel(axesNames{ax}, 'FontSize', 16);      % eje Y grande
    title([axesNames{ax} ' por punto'], ...
        'FontWeight','bold','FontSize',16);     % título grande
end

xlabel('Punto', 'FontSize', 16);                % eje X grande
legend('Location','best','FontSize',14);        % leyenda tamaño 14

saveas(fig3, fullfile(outdir, 'batch_axis_errors.png'));
saveas(fig3, fullfile(outdir, 'batch_axis_errors.eps'), 'epsc');


%% ============================================================
% FIGURA 4 — Tiempo de simulación por punto y por modo
%% ============================================================

fig4 = figure('Name','Tiempo por punto','Color','w','Position',[200 200 900 600]);
hold on; grid on;

for m = 1:numel(modesList)
    idx = strcmp(T_all.mode, modesList{m});
    plot(pts, T_all.time_sim_s(idx), '-o', ...
        'LineWidth', 2, 'Color', palette(m,:), ...
        'MarkerFaceColor', palette(m,:), ...
        'DisplayName', modeShort{m});   % ← AQUI SE CORRIGE
end

xlabel('Punto','FontSize',16);
ylabel('Tiempo de simulación (s)','FontSize',16);
title('Tiempo por punto y controlador','FontWeight','bold','FontSize',16);

% Leyenda con nombres correctos
legend('Location','best','FontSize',14);

saveas(fig4, fullfile(outdir, 'batch_time_per_point.png'));
saveas(fig4, fullfile(outdir, 'batch_time_per_point.eps'), 'epsc');





    %% Mensaje final
    updateInfo(sprintf('✅ Batch completado: %s', outdir), [0.8 1 0.8]);
end


    %% ======= FUNCIONES DE MOUSE =======
    function onMouseDown(~, ~)
        % Activación de arrastre para variación de z_ref con clic derecho
        if strcmp(get(fig, 'SelectionType'), 'alt')
            setappdata(fig, 'dragZ', true);
            setappdata(fig, 'lastPix', get(0, 'PointerLocation'));
        end
    end

    function onMouseUp(~, ~)
        % Liberación de banderas de arrastre
        if isappdata(fig, 'dragZ'),  rmappdata(fig, 'dragZ');  end
        if isappdata(fig, 'lastPix'), rmappdata(fig, 'lastPix'); end
    end

    function onMouseMove()
        % Gestión de arrastre vertical para modificar z_ref y proyecciones
        if isappdata(fig, 'dragZ') && getappdata(fig, 'dragZ')
            pcur  = get(0, 'PointerLocation');
            plast = getappdata(fig, 'lastPix');

            if ~isempty(plast)
                dy   = pcur(2) - plast(2);
                axPx = getpixelposition(ax3D, true);
                scale = (2 * rmax) / max(1, axPx(4));
                z_ref = z_ref + dy * scale;

                % Limitación de z_ref según alcanzabilidad radial
                rho = hypot(x_ref, y_ref);
                z_allowed_max = sqrt(max(rmax^2 - rho^2, 0));
                z_required_min = sqrt(max(rmin^2 - rho^2, 0));
                z_ref = max(-z_allowed_max, min(z_allowed_max, z_ref));

                if rho < rmin
                    if z_ref >= 0
                        z_ref = max(z_ref, z_required_min);
                    else
                        z_ref = min(z_ref, -z_required_min);
                    end
                end

                % Sincronización de UI y proyecciones
                set(edZ, 'String', num2str(z_ref, '%.2f'));
                set(hGoal, 'XData', x_ref, 'YData', y_ref, 'ZData', z_ref);

                [~, ~, ~, ~, ~, ~, xe, ye, ze] = ...
                    fk3d(theta1, theta2, theta3, L1, L2);
                updateProjections(hProj, xe, ye, ze);

                handles = getappdata(fig, 'plotHandles');
                set(handles.hZref, 'Value', z_ref);
                drawnow limitrate;
            end

            setappdata(fig, 'lastPix', pcur);
            return;
        end

        % Actualización de objetivo si el modo “seguir mouse” está activo
        if get(chkFollow, 'Value') && strcmp(get(btnStop, 'Enable'), 'off')
            [xr, yr, inside] = getMouseOnZ(ax3D, z_ref);
            if inside
                [xr, yr] = clampXYGivenZ(xr, yr, z_ref, rmin, rmax);
                x_ref = xr; y_ref = yr;
                set(hGoal, 'XData', x_ref, 'YData', y_ref, 'ZData', z_ref);
                set(edX, 'String', num2str(x_ref, '%.2f'));
                set(edY, 'String', num2str(y_ref, '%.2f'));

                handles = getappdata(fig, 'plotHandles');
                set(handles.hXref, 'Value', x_ref);
                set(handles.hYref, 'Value', y_ref);
            end
        end
    end

    function onScroll(evt)
        % Modificación incremental de z_ref con la rueda del mouse y actualización de proyecciones
        step = 0.25 * evt.VerticalScrollCount;
        rho  = hypot(x_ref, y_ref);
        z_allowed_max  = sqrt(max(rmax^2 - rho^2, 0));
        z_required_min = sqrt(max(rmin^2 - rho^2, 0));

        z_ref = z_ref - step;
        z_ref = max(-z_allowed_max, min(z_allowed_max, z_ref));

        if rho < rmin
            if z_ref >= 0
                z_ref = max(z_ref, z_required_min);
            else
                z_ref = min(z_ref, -z_required_min);
            end
        end

        set(edZ, 'String', num2str(z_ref, '%.2f'));
        set(hGoal, 'XData', x_ref, 'YData', y_ref, 'ZData', z_ref);

        [~, ~, ~, ~, ~, ~, xe, ye, ze] = ...
            fk3d(theta1, theta2, theta3, L1, L2);
        updateProjections(hProj, xe, ye, ze);

        handles = getappdata(fig, 'plotHandles');
        set(handles.hZref, 'Value', z_ref);
        drawnow;
    end

end  % ====== FIN DE LA FUNCIÓN PRINCIPAL ======


%% =================== FUNCIONES AUXILIARES ===================

function K = compute_dlqr_safe(Ad, Bd, Q, R)
    % Preparación de K vía dlqr con retroceso a una aproximación diagonal estable
    try
        K = dlqr(Ad, Bd, Q, R);
    catch
        q = diag(Q).';
        r = diag(R).';
        P = 0.5 * (q + sqrt(q.^2 + 4 .* q .* r + eps));
        kdiag = P ./ (r + P + eps);
        K = diag(kdiag);
    end
end

function drawWorkspace3D(ax, RANGE, rmin, rmax)
    % Construcción del espacio de trabajo:
    % 1) Esfera semitransparente (alcance máximo)
    % 2) Disco en z=0 con anillo de alcance mínimo
    % 3) Ejes cartesianos con positivos y negativos visibles

    cla(ax);
    hold(ax, 'on');

    % Configuración de límites, etiquetas y vista
    xlim(ax, [-RANGE RANGE]);
    ylim(ax, [-RANGE RANGE]);
    zlim(ax, [-RANGE RANGE]);
    xlabel(ax, 'X', 'FontWeight', 'bold');
    ylabel(ax, 'Y', 'FontWeight', 'bold');
    zlabel(ax, 'Z', 'FontWeight', 'bold');
    view(ax, 45, 25);
    axis(ax, 'equal'); axis(ax, 'vis3d');
    set(ax, 'Projection', 'perspective');
    camtarget(ax, [0 0 0]); camup(ax, [0 0 1]); camva(ax, 7);

    % Activación de grilla principal y menor
    grid(ax, 'on');
    set(ax, 'XMinorGrid', 'on', 'YMinorGrid', 'on', 'ZMinorGrid', 'on');
    ax.GridAlpha      = 0.20;
    ax.MinorGridAlpha = 0.08;
    ax.XColor = [0.3 0.3 0.3];
    ax.YColor = [0.3 0.3 0.3];
    ax.ZColor = [0.3 0.3 0.3];

    % Superficie de alcance máximo (esfera)
    [XS, YS, ZS] = sphere(60);
    surf(ax, XS * rmax, YS * rmax, ZS * rmax, ...
        'FaceColor', [0.78 0.78 0.78], 'FaceAlpha', 0.15, 'EdgeColor', 'none');

    % Disco de piso y anillo interno (alcance mínimo)
    ang = linspace(0, 2*pi, 200);
    x_circ = rmax * cos(ang);
    y_circ = rmax * sin(ang);
    fill3(ax, x_circ, y_circ, zeros(size(x_circ)), [0.8 0.8 0.8], ...
        'FaceAlpha', 0.20, 'EdgeColor', [0.2 0.2 0.2], 'LineStyle', '-');

    x_inner = rmin * cos(ang);
    y_inner = rmin * sin(ang);
    plot3(ax, x_inner, y_inner, zeros(size(x_inner)), 'k--', 'LineWidth', 1.2);

    % Ejes cartesianos (positivos sólidos, negativos tenues punteados)
    colX = [1 0 0]; colY = [0 1 0]; colZ = [0 0 1];
    lineW = 2.5; soft = 0.5;

    plot3(ax, [0 RANGE], [0 0], [0 0], '-',  'Color', colX, 'LineWidth', lineW);
    plot3(ax, [0 0], [0 RANGE], [0 0], '-',  'Color', colY, 'LineWidth', lineW);
    plot3(ax, [0 0], [0 0], [0 RANGE], '-',  'Color', colZ, 'LineWidth', lineW);

    plot3(ax, [0 -RANGE], [0 0], [0 0], '--', 'Color', colX*soft, 'LineWidth', 1.8);
    plot3(ax, [0 0], [0 -RANGE], [0 0], '--', 'Color', colY*soft, 'LineWidth', 1.8);
    plot3(ax, [0 0], [0 0], [0 -RANGE], '--', 'Color', colZ*soft, 'LineWidth', 1.8);

    % Marcación de la base del brazo
    plot3(ax, 0, 0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 4);

    % Reordenamiento para mantener ejes por encima de la esfera
    ch = get(ax, 'Children');
    set(ax, 'Children', flipud(ch));

    drawnow;
end

function [hL1, hL2, hEE, hGoal, hPlanes, hProj] = drawArm3D(ax, ...
    x1, y1, z1, x2, y2, z2, xe, ye, ze, xr, yr, zr, ~)
    % Construcción del brazo y del objetivo con proyecciones auxiliares

    hL1 = plot3(ax, [0 x1], [0 y1], [0 z1], '-', ...
        'LineWidth', 4.5, 'Color', [0.2 0.4 0.9]);

    hL2 = plot3(ax, [x1 x2], [y1 y2], [z1 z2], '-', ...
        'LineWidth', 4.5, 'Color', [0.9 0.3 0.2]);

    hEE = plot3(ax, xe, ye, ze, 'o', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', [0.2 0.9 0.3], ...
        'MarkerEdgeColor', [0 0.5 0], ...
        'LineWidth', 2.5);

    hGoal = plot3(ax, xr, yr, zr, 'x', ...
        'MarkerSize', 14, ...
        'MarkerFaceColor', [1 0.8 0.2], ...
        'MarkerEdgeColor', [0.8 0.4 0], ...
        'LineWidth', 2);

    % Líneas de proyección a planos principales
    hLineXY = plot3(ax, [xe xe], [ye ye], [ze 0], ':', 'LineWidth', 1.5, 'Color', [0.4 0.4 0.4]);
    hLineXZ = plot3(ax, [xe xe], [0 0], [ze ze], ':', 'LineWidth', 1.5, 'Color', [0.4 0.4 0.4]);
    hLineYZ = plot3(ax, [0 0], [ye ye], [ze ze], ':', 'LineWidth', 1.5, 'Color', [0.4 0.4 0.4]);

    % Puntos de referencia sobre ejes
    hAxX = plot3(ax, xe, 0, 0, 'o', 'Color', [0.9 0.2 0.2], 'MarkerFaceColor', [0.9 0.2 0.2], 'MarkerSize', 6);
    hAxY = plot3(ax, 0, ye, 0, 'o', 'Color', [0.2 0.8 0.2], 'MarkerFaceColor', [0.2 0.8 0.2], 'MarkerSize', 6);
    hAxZ = plot3(ax, 0, 0, ze, 'o', 'Color', [0.2 0.4 0.9], 'MarkerFaceColor', [0.2 0.4 0.9], 'MarkerSize', 6);

    hPlanes = {};
    hProj = struct('lxy', hLineXY, 'lxz', hLineXZ, 'lyz', hLineYZ, ...
                   'axX', hAxX, 'axY', hAxY, 'axZ', hAxZ);
end

function updateProjections(hProj, xe, ye, ze)
    % Actualización de líneas y puntos de proyección para el efector final
    set(hProj.lxy, 'XData', [xe xe], 'YData', [ye ye], 'ZData', [ze 0]);
    set(hProj.lxz, 'XData', [xe xe], 'YData', [0 0], 'ZData', [ze ze]);
    set(hProj.lyz, 'XData', [0 0], 'YData', [ye ye], 'ZData', [ze ze]);
    set(hProj.axX, 'XData', xe, 'YData', 0, 'ZData', 0);
    set(hProj.axY, 'XData', 0,  'YData', ye, 'ZData', 0);
    set(hProj.axZ, 'XData', 0,  'YData', 0,  'ZData', ze);
end

function [xr, yr, inside] = getMouseOnZ(ax, z_plane)
    % Cálculo de intersección del rayo de cámara con plano z=z_plane
    cp = get(ax, 'CurrentPoint');
    p1 = cp(1, :); p2 = cp(2, :);
    dir = p2 - p1;

    if abs(dir(3)) < 1e-12
        xr = NaN; yr = NaN; inside = false; return;
    end

    t = (z_plane - p1(3)) / dir(3);
    x = p1(1) + t * dir(1);
    y = p1(2) + t * dir(2);

    xl = xlim(ax); yl = ylim(ax);
    inside = (x >= xl(1) && x <= xl(2) && y >= yl(1) && y <= yl(2));
    xr = x; yr = y;
end

function [xc, yc] = clampXYGivenZ(x, y, z, rmin, rmax)
    % Limitación del objetivo en XY para un Z dado según alcanzabilidad
    rho = hypot(x, y);
    rho_max = sqrt(max(rmax^2 - z^2, 0));
    rho_min = sqrt(max(rmin^2 - z^2, 0));

    if rho < 1e-12
        if rho_min == 0, xc = 0; yc = 0;
        else, xc = rho_min; yc = 0; end
        return;
    end

    rr = min(max(rho, rho_min), max(rho_min, rho_max));
    s  = rr / rho;
    xc = x * s; yc = y * s;
end

function [x2, y2, z2] = clampTarget3DAll(x, y, z, rmin, rmax)
    % Limitación de un objetivo 3D al casquete esférico alcanzable [rmin, rmax]
    r = sqrt(x^2 + y^2 + z^2);
    if r < 1e-12, x2 = rmin; y2 = 0; z2 = 0; return; end
    rr = min(max(r, rmin), rmax);
    s  = rr / r;
    x2 = x * s; y2 = y * s; z2 = z * s;
end

function J = jacobian3d(t1, t2, t3, L1, L2)
    % Construcción del Jacobiano geométrico 3x3 del efector final
    c1  = cos(t1);  s1  = sin(t1);
    c2  = cos(t2);  s2  = sin(t2);
    c23 = cos(t2 + t3); s23 = sin(t2 + t3);

    rho  = L1 * c2 + L2 * c23;
    dr2  = -L1 * s2 - L2 * s23;
    dr3  = -L2 * s23;
    dz2  = L1 * c2 + L2 * c23;
    dz3  = L2 * c23;

    J = [-rho*s1, dr2*c1, dr3*c1;
          rho*c1, dr2*s1, dr3*s1;
               0,    dz2,    dz3];
end

function [x1, y1, z1, x2, y2, z2, xe, ye, ze] = fk3d(t1, t2, t3, L1, L2)
    % Cálculo de posiciones (cinemática directa) para eslabón 1, eslabón 2 y efector
    c1  = cos(t1);  s1  = sin(t1);
    c2  = cos(t2);  s2  = sin(t2);
    c23 = cos(t2 + t3); s23 = sin(t2 + t3);

    rho1 = L1 * c2;  z1   = L1 * s2;
    x1   = rho1 * c1; y1   = rho1 * s1;

    rho  = L1 * c2 + L2 * c23;
    ze   = L1 * s2 + L2 * s23;
    xe   = rho * c1; ye   = rho * s1;

    x2 = x1 + L2 * c23 * c1;
    y2 = y1 + L2 * c23 * s1;
    z2 = z1 + L2 * s23;
end

function theta_clamped = clampJoint(theta, limits)
    % Saturación de ángulo articular en límites angulares (en grados)
    theta_deg = rad2deg(theta);
    theta_deg = max(limits(1), min(limits(2), theta_deg));
    theta_clamped = deg2rad(theta_deg);
end

function val = clamp(x, a, b)
    % Saturación escalar entre a y b
    val = max(a, min(b, x));
end

function a = wrapToPi_local(a)
    % Normalización de ángulo a (-π, π]
    a = atan2(sin(a), cos(a));
end

function s = tern(cond, a, b)
    % Selección ternaria simple para strings/estados
    if cond, s = a; else, s = b; end
end

function v = valOrZero(strv)
    % Conversión numérica robusta con predeterminado en 0
    v = str2double(strv);
    if isnan(v), v = 0; end
end

function v = posOrDefault(strv, def)
    % Conversión numérica con mínimo positivo y valor por defecto
    v = str2double(strv);
    if isnan(v) || v <= 0, v = def; end
end

function [Kdeg, Kxdeg, Kydeg, Kzdeg] = getKdegAndComponents(fis, e, Kmin, Kmax)
    % Evaluación del FIS para obtener K total y su reparto relativo por ejes
    Kraw = evalfisFlexible(fis, e);
    Kdeg = clamp(abs(Kraw), Kmin, Kmax);

    w = abs(e(:));
    s = sum(w) + eps;
    Kxdeg = Kdeg * (w(1) / s);
    Kydeg = Kdeg * (w(2) / s);
    Kzdeg = Kdeg * (w(3) / s);
end

function out = evalfisFlexible(fis, e)
    % Evaluación flexible del FIS según el número de entradas disponibles
    try
        ni = numel(fis.Inputs);
    catch
        ni = 0;
    end

    ex = clamp(e(1), -20, 20);
    ey = clamp(e(2), -20, 20);
    ez = clamp(e(3), -20, 20);

    try
        if isfield(fis, 'Stub') && fis.Stub
            out = 2.0;
        else
            if ni >= 3
                out = evalfis(fis, [ex, ey, ez]);
            elseif ni == 2
                out = evalfis(fis, [ex, ey]);
            elseif ni == 1
                out = evalfis(fis, [norm([ex, ey, ez])]);
            else
                out = 2.0;
            end
        end
        if ~isfinite(out), out = 2.0; end
    catch
        out = 2.0;
    end
end

function sim = simulateOnce(theta_ini, target_xyz, cfg)
    % Preparación y ejecución de una simulación individual para un objetivo dado
    L1 = cfg.L1; L2 = cfg.L2;
    max_iter = cfg.max_iter; tol = cfg.tol;
    stable_needed = cfg.stable_needed; beta = cfg.beta;
    K_min_deg = cfg.K_min_deg; K_max_deg = cfg.K_max_deg;
    K_safety_ang_deg = cfg.K_safety_ang_deg;
    lambda_base = cfg.lambda_base; lambda_smallE = cfg.lambda_smallE;
    err_small_th = cfg.err_small_th; cond_boost = cfg.cond_boost;
    noise_on = cfg.noise_on; sigma_noise = cfg.sigma_noise;
    lambda_soft_on = cfg.lambda_soft_on; a_mu = cfg.a_mu; b_kappa = cfg.b_kappa;
    modeIdx = cfg.modeIdx; fis1 = cfg.fis1;
    Kp = cfg.Kp; Ki = cfg.Ki; Kd = cfg.Kd;

    % Lectura de pesos LQR con valores seguros por defecto
    Qx = 1.0; if isfield(cfg, 'Qx'), Qx = cfg.Qx; end
    Qy = 1.0; if isfield(cfg, 'Qy'), Qy = cfg.Qy; end
    Qz = 1.0; if isfield(cfg, 'Qz'), Qz = cfg.Qz; end
    Rv = 0.1; if isfield(cfg, 'R'),  Rv = cfg.R;  end

    % Periodo de integración de la simulación
    dt_step_s = 0.02; if isfield(cfg, 'dt_step_s'), dt_step_s = cfg.dt_step_s; end

    % Límites articulares para la simulación (se permiten sobreescrituras)
    JOINT_LIMITS = struct('theta1', [-180 180], 'theta2', [-120 120], 'theta3', [-150 150]);
    if isfield(cfg, 'JOINT_LIMITS'), JOINT_LIMITS = cfg.JOINT_LIMITS; end

    % Inicialización de estados y objetivo
    t1 = theta_ini(1); t2 = theta_ini(2); t3 = theta_ini(3);
    x_ref = target_xyz(1); y_ref = target_xyz(2); z_ref = target_xyz(3);

    % Buffers de historial
    err_hist = nan(max_iter, 1);
    TH = nan(max_iter, 3);
    EE = nan(max_iter, 3);

    % Inicialización del controlador
    dtheta_prev  = [0; 0; 0];
    stable_count = 0;
    e_prev       = [0; 0; 0];
    ei           = [0; 0; 0];
    ei_max       = 5;

    % Construcción de K LQR si corresponde
    if modeIdx == 2
        Ad = eye(3); Bd = eye(3);
        Q = diag([Qx Qy Qz]); R = Rv * eye(3);
        K_lqr = compute_dlqr_safe(Ad, Bd, Q, R);
    else
        K_lqr = zeros(3);
    end

    % Temporización (tiempo de pared y acumulado de simulación)
    tWall = tic; t_sim = 0.0;

    % Bucle de integración
    for k = 1:max_iter
        [~, ~, ~, ~, ~, ~, xe, ye, ze] = fk3d(t1, t2, t3, L1, L2);
        e = [x_ref - xe; y_ref - ye; z_ref - ze];

        if noise_on && sigma_noise > 0
            e = e + sigma_noise * randn(3, 1);
        end

        err = norm(e);
        err_hist(k) = err;
        TH(k, :)    = [t1 t2 t3];
        EE(k, :)    = [xe ye ze];

        if err < tol
            stable_count = stable_count + 1;
            if stable_count >= stable_needed
                t_sim = t_sim + dt_step_s;
                break;
            end
        else
            stable_count = 0;
        end

        J   = jacobian3d(t1, t2, t3, L1, L2);
        JJt = J * J.'; lam = lambda_base;

        if err < err_small_th, lam = lam + lambda_smallE; end

        cJJt = cond(JJt);
        if ~isfinite(cJJt) || cJJt > 1e3, lam = lam + cond_boost; end

        if lambda_soft_on
            mu = sqrt(max(eps, abs(det(JJt))));
            kappa = cJJt;
            muN  = max(0, min(1, mu / max(eps, L1 * L2)));
            kapN = max(0, min(1, (log10(max(1, kappa))) / 3));
            lam  = lam + a_mu * (1 - muN) + b_kappa * kapN;
        end

        switch modeIdx
            case 1
                e_dot = (e - e_prev);
                ei = max(-ei_max, min(ei_max, ei + e));
                u_cart = Kp * e + Ki * ei + Kd * e_dot;
            case 2
                u_cart = +K_lqr * e;
            case {3, 4}
                u_cart = e;
        end
        e_prev = e;

        A = JJt + (lam^2) * eye(3);
        cart2joint = (J.' * (A \ u_cart));

        if modeIdx >= 3
            Kdeg = clamp(abs(evalfisFlexible(fis1, e)), K_min_deg, K_max_deg);
            Ktheta = deg2rad(Kdeg);
            D = diag([Ktheta, Ktheta, Ktheta]);
        else
            D = eye(3);
        end

        dtheta = D * cart2joint;
        dtheta = beta * dtheta_prev + (1 - beta) * dtheta;
        dtheta_prev = dtheta;

        Ks_ang = deg2rad(K_safety_ang_deg);
        dtheta = max(-Ks_ang, min(Ks_ang, dtheta));

        t1 = wrapToPi_local(t1 + dtheta(1));
        t2 = clampJoint(t2 + dtheta(2), JOINT_LIMITS.theta2);
        t3 = clampJoint(t3 + dtheta(3), JOINT_LIMITS.theta3);

        t_sim = t_sim + dt_step_s;
    end

    % Cierre y empaquetado de resultados
    time_wall_s = toc(tWall);

    steps = find(isnan(err_hist), 1) - 1;
    if isempty(steps), steps = max_iter; end

    err_final = err_hist(steps);
    ee_end    = EE(steps, :).';
    theta_end = [t1; t2; t3];

    sim = struct();
    sim.theta_ini  = theta_ini(:);
    sim.theta_end  = theta_end(:);
    sim.ee_end     = ee_end(:);
    sim.steps      = steps;
    sim.err_final  = err_final;
    sim.success    = (err_final < tol && stable_count >= stable_needed);
    sim.err_hist   = err_hist(1:steps);
    sim.theta_hist = TH(1:steps, :);
    sim.ee_hist    = EE(1:steps, :);
    sim.time_sim_s = t_sim;
    sim.time_wall_s = time_wall_s;
end

function fis = ensureFIS(fisFile)
    % Preparación del FIS desde archivo .fis o construcción de uno por defecto
    if exist('readfis', 'file') && exist(fisFile, 'file')
        try
            fis = readfis(fisFile);
            return;
        catch
            % Silencioso: se intentará crear uno por defecto
        end
    end

    try
        fis = crearFISporDefecto();
    catch
        fis = struct('Inputs', [], 'Stub', true);
    end
end

function fis = crearFISporDefecto()
    % Construcción de un FIS Mamdani de respaldo (3 entradas y 1 salida)
    fis = mamfis('Name', 'Controlador_Fuzzy_Mamdani');

    fis = addInput(fis, [-20 20], 'Name', 'Error_X');
    fis = addInput(fis, [-20 20], 'Name', 'Error_Y');
    fis = addInput(fis, [-20 20], 'Name', 'Error_Z');

    fis = addOutput(fis, [-12 12], 'Name', 'Delta_Theta');

    % Definición de MFs triangulares para entradas
    ng = [-20 -20 -10];
    nm = [-20 -10 0];
    z  = [-5 0 5];
    pm = [0 10 20];
    pg = [10 20 20];

    for var = ["Error_X", "Error_Y", "Error_Z"]
        fis = addMF(fis, var, 'trimf', ng, 'Name', 'NG');
        fis = addMF(fis, var, 'trimf', nm, 'Name', 'NM');
        fis = addMF(fis, var, 'trimf', z,  'Name', 'Z');
        fis = addMF(fis, var, 'trimf', pm, 'Name', 'PM');
        fis = addMF(fis, var, 'trimf', pg, 'Name', 'PG');
    end

    % Definición de MFs triangulares para salida
    ng_o = [-12 -12 -6];
    nm_o = [-12 -6 0];
    z_o  = [-2 0 2];
    pm_o = [0 6 12];
    pg_o = [6 12 12];

    fis = addMF(fis, 'Delta_Theta', 'trimf', ng_o, 'Name', 'NG');
    fis = addMF(fis, 'Delta_Theta', 'trimf', nm_o, 'Name', 'NM');
    fis = addMF(fis, 'Delta_Theta', 'trimf', z_o,  'Name', 'Z');
    fis = addMF(fis, 'Delta_Theta', 'trimf', pm_o, 'Name', 'PM');
    fis = addMF(fis, 'Delta_Theta', 'trimf', pg_o, 'Name', 'PB');

    % Construcción de una base de reglas densa (promedio de índices)
    ruleList = [];
    for i = 1:5
        for j = 1:5
            for k = 1:5
                out = round((i + j + k) / 3);
                out = max(1, min(5, out));
                ruleList(end + 1, :) = [i, j, k, out, 1, 1]; %#ok<AGROW>
            end
        end
    end
    fis = addRule(fis, ruleList);

    % Configuración de operadores e integración
    fis.AndMethod             = 'min';
    fis.OrMethod              = 'max';
    fis.ImplicationMethod     = 'min';
    fis.AggregationMethod     = 'max';
    fis.DefuzzificationMethod = 'centroid';
end
