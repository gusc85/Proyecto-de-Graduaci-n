function pendulo_pid_lqr_fuzzy()

%% ---------- Parámetros físicos ----------
m = 1.0;        % masa (kg)
L = 0.5;        % longitud (m)
g = 9.81;       % gravedad (m/s²)
b = 0.08;       % fricción (N·m·s/rad)
I = m * L^2;    % momento de inercia

% Actuador
u_max    = 6.0;     % torque máximo (N·m)
tau_u    = 0.08;    % constante de tiempo actuador (s)
max_udot = 60;      % límite de cambio de torque (N·m/s)

% Simulación
dt      = 1/300;    % paso de integración (s)
vis_hz  = 60;       % frecuencia de visualización (Hz)
substeps = max(1, round((1/vis_hz)/dt));

%% ---------- Estado inicial ----------
theta0_deg = 180;   % ángulo inicial (grados)
x   = [deg2rad(wrapTo180_local(theta0_deg)); 0];  % [theta; dtheta]
t   = 0;
th  = []; 
uh  = []; 
tt  = [];
u_act = 0;

%% ---------- Criterios de convergencia ----------
thr_th_deg_pid_lqr = 2.0;   % umbral angular PID/LQR (grados)
thr_dth_deg        = 5.0;   % umbral velocidad angular (grados/s)
dwell_s   = 0.50;           % tiempo de estabilización (s)
dwell_N   = ceil(dwell_s/dt);
ok_cnt    = 0;
autoStop  = true;

% Fuzzy: tolerancia exclusiva
tol_deg_fuz = 2.0;

%% ---------- Controladores (valores por defecto) ----------
% PID optimizado
Kp = 3.2; Ki = 0; Kd = 1.9; Ierr = 0;

% LQR
[A, B] = linearize_upright(m, L, g, b, I);
Q11 = 45; Q22 = 8; R = 0.9;
try
    K_lqr = lqr(A, B, diag([Q11, Q22]), R);
catch
    warning('lqr() no disponible; usando K=[10 2]');
    K_lqr = [10 2];
end

% Fuzzy
Ko   = 7.8; 
Kmin = 3.0; 
Kmax = 12.5;
defuzzMethod = 'centroid';
theta_rng    = [-pi, pi];
dtheta_rng   = [-12, 12];
u_rng        = [-u_max, u_max];

% Crear FIS
fis = construirFIS_struct(theta_rng, dtheta_rng, u_rng);

%% ---------- TIEMPOS DE EJE X POR MODO (SIMULACIÓN INDIVIDUAL / GUI) ----------
tmax_pid  = 5;
tmax_lqr  = 5;
tmax_fuzC = 20;
tmax_fuzB = 20;

%% ---------- TIEMPOS DE BATCH (CORRIDAS AUTOMÁTICAS) ----------
batch_tmax_pid  = 1.7;
batch_tmax_lqr  = 1.7;
batch_tmax_fuzC = 20;
batch_tmax_fuzB = 20;

%% ---------- Crear ventana principal ----------
fig = figure('Name', 'Simulador Péndulo Invertido - Control Avanzado', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'ToolBar', 'figure', ...
    'Position', [80 50 1380 780], ...
    'Color', [0.94 0.94 0.96]);

%% ---------- Panel de animación ----------
axSim = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.04 0.03 0.4 1]);
axis(axSim, 'equal');
box(axSim, 'on');
grid(axSim, 'on');
hold(axSim, 'on');

Rviz = 1.3 * L;
xlim(axSim, [-1.05*Rviz 1.28*Rviz]);
ylim(axSim, [-1.05*Rviz 1.05*Rviz]);

plot(axSim, 0, 0, 'ks', 'MarkerSize', 18, 'MarkerFaceColor', [0.3 0.3 0.3], 'LineWidth', 2);

[bx, by] = xy_from_theta(L, x(1));
hRod = plot(axSim, [0 bx], [0 by], 'LineWidth', 5, 'Color', [0.85 0.35 0.10]);
hBob = plot(axSim, bx, by, 'o', 'MarkerSize', 18, ...
    'MarkerFaceColor', [0.20 0.35 0.85], 'MarkerEdgeColor', [0 0 0], 'LineWidth', 2);

plot(axSim, [0 0], [0 Rviz], '--', 'Color', [0.2 0.8 0.2], 'LineWidth', 2.5, 'LineStyle', '--');

title(axSim, {'Péndulo Invertido'; 'θ = 0° → Vertical Arriba'}, 'FontWeight', 'bold', 'FontSize', 11);
xlabel(axSim, 'x (m)', 'FontWeight', 'bold');
ylabel(axSim, 'y (m)', 'FontWeight', 'bold');

hTxt = text(axSim, -1.02*Rviz, 1.02*Rviz, '', ...
    'FontName', 'Consolas', 'FontSize', 9.5, ...
    'VerticalAlignment', 'top', 'BackgroundColor', [1 1 1 0.8]);

hConv = text(axSim, 0.13*Rviz, -1.03*Rviz, '', ...
    'FontName', 'Consolas', 'FontSize', 10, ...
    'HorizontalAlignment', 'center', ...
    'VerticalAlignment', 'bottom', ...
    'Color', [0.00 0.60 0.00], ...
    'FontWeight', 'bold', ...
    'Visible', 'off', ...
    'BackgroundColor', [0.9 1 0.9 0.9]);

%% ---------- Gráfica en tiempo real ----------
axAll = axes('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.48 0.55 0.48 0.40]);
box(axAll, 'on');
grid(axAll, 'on');
hold(axAll, 'on');
title(axAll, 'Historial Completo: θ (deg) y Torque u (N·m)', 'FontWeight', 'bold', 'FontSize', 11);

yyaxis(axAll, 'left');
lineTh = plot(axAll, nan, nan, 'LineWidth', 2.2, 'DisplayName', 'θ (deg)', 'Color', [0.8 0.2 0.2]);
ylabel(axAll, 'θ (deg)', 'FontWeight', 'bold', 'Color', [0.8 0.2 0.2]);
yline(axAll, 0, '--k', 'LineWidth', 1.5, 'Alpha', 0.5, 'HandleVisibility', 'off');

yyaxis(axAll, 'right');
lineU = plot(axAll, nan, nan, 'LineWidth', 1.8, 'DisplayName', 'u (N·m)', 'Color', [0.2 0.4 0.8]);
ylabel(axAll, 'u (N·m)', 'FontWeight', 'bold', 'Color', [0.2 0.4 0.8]);
yline(axAll, 0, '--k', 'LineWidth', 1.5, 'Alpha', 0.5, 'HandleVisibility', 'off');

xlabel(axAll, 'Tiempo (s)', 'FontWeight', 'bold');
legend(axAll, 'Location', 'northwest', 'FontSize', 9);

%% ---------- Tabs de control ----------
tabGroup = uitabgroup(fig, 'Position', [0.50 0.03 0.45 0.45]);

% TAB 1
tab1 = uitab(tabGroup, 'Title', '⚙️ Control', 'BackgroundColor', [0.96 0.96 0.98]);
yPos = 0.73;

uicontrol(tab1, 'Style', 'text', 'String', '🎯 MODO DE CONTROL', ...
    'Position', [20 yPos*380 350 28], 'FontWeight', 'bold', 'FontSize', 11, ...
    'BackgroundColor', [0.2 0.4 0.7], 'ForegroundColor', 'w');
yPos = yPos - 0.08;

popCtrl = uicontrol(tab1, 'Style', 'popupmenu', ...
    'String', {'🔵 PID', '🟢 LQR', '🟠 Fuzzy - Centroid', '🟡 Fuzzy - Bisector'}, ...
    'Value', 1, ...
    'Position', [20 yPos*380 350 32], ...
    'FontSize', 10, ...
    'BackgroundColor', [0.98 0.98 1], ...
    'Callback', @onModeChanged);
yPos = yPos - 0.12;

uicontrol(tab1, 'Style', 'text', 'String', '📐 CONFIGURACIÓN INICIAL', ...
    'Position', [20 yPos*380 350 28], 'FontWeight', 'bold', 'FontSize', 11, ...
    'BackgroundColor', [0.7 0.2 0.4], 'ForegroundColor', 'w');
yPos = yPos - 0.08;

uicontrol(tab1, 'Style', 'text', 'String', 'Ángulo inicial (grados):', ...
    'Position', [20 yPos*380 180 24], 'HorizontalAlignment', 'left', ...
    'FontSize', 9, 'BackgroundColor', [0.96 0.96 0.98]);

editTheta = uicontrol(tab1, 'Style', 'edit', ...
    'String', num2str(theta0_deg), ...
    'Position', [210 yPos*380 80 28], ...
    'FontSize', 10, ...
    'BackgroundColor', 'w', ...
    'Callback', @onThetaEdit);
yPos = yPos - 0.10;

chkDrag = uicontrol(tab1, 'Style', 'checkbox', ...
    'String', '🖱️ Arrastrar con mouse', ...
    'Position', [20 yPos*380 200 28], ...
    'BackgroundColor', [0.96 0.96 0.98], ...
    'Value', 0, ...
    'FontSize', 9, ...
    'Callback', @onToggleDrag);

chkAuto = uicontrol(tab1, 'Style', 'checkbox', ...
    'String', '⏹️ Auto-detener al converger', ...
    'Position', [20 (yPos-0.08)*380 210 28], ...
    'BackgroundColor', [0.96 0.96 0.98], ...
    'Value', 1, ...
    'FontSize', 9, ...
    'Callback', @(src,~) setAutoStop(src.Value));
yPos = yPos - 0.20;

uicontrol(tab1, 'Style', 'pushbutton', 'String', '▶️  SIMULAR', ...
    'Position', [20 yPos*380 160 45], ...
    'FontWeight', 'bold', 'FontSize', 11, ...
    'BackgroundColor', [0.2 0.7 0.3], 'ForegroundColor', 'w', ...
    'Callback', @onStart);

uicontrol(tab1, 'Style', 'pushbutton', 'String', '⏹️  DETENER', ...
    'Position', [200 yPos*380 160 45], ...
    'FontSize', 11, ...
    'Callback', @onStop);
yPos = yPos - 0.14;

uicontrol(tab1, 'Style', 'pushbutton', 'String', '🔄  REINICIAR', ...
    'Position', [20 yPos*380 160 40], ...
    'FontSize', 10, ...
    'Callback', @onReset);

uicontrol(tab1, 'Style', 'pushbutton', 'String', '💾  GUARDAR BATCH', ...
    'Position', [200 yPos*380 160 40], ...
    'FontSize', 10, ...
    'Callback', @onSaveCSV);

% TAB 2
tab2 = uitab(tabGroup, 'Title', '🔵 PID', 'BackgroundColor', [0.96 0.98 1]);
yPos = 0.85;
uicontrol(tab2, 'Style', 'text', 'String', 'PARÁMETROS PID', ...
    'Position', [20 yPos*380 760 28], 'FontWeight', 'bold', 'FontSize', 11, ...
    'BackgroundColor', [0.2 0.4 0.7], 'ForegroundColor', 'w');
yPos = yPos - 0.10;
mkParamRow(tab2, 'Kp (Proporcional):', Kp, @onKp, yPos);
yPos = yPos - 0.12;
mkParamRow(tab2, 'Ki (Integral):', Ki, @onKi, yPos);
yPos = yPos - 0.12;
mkParamRow(tab2, 'Kd (Derivativo):', Kd, @onKd, yPos);

% TAB 3
tab3 = uitab(tabGroup, 'Title', '🟢 LQR', 'BackgroundColor', [0.96 1 0.96]);
yPos = 0.85;
uicontrol(tab3, 'Style', 'text', 'String', 'PARÁMETROS LQR', ...
    'Position', [20 yPos*380 760 28], 'FontWeight', 'bold', 'FontSize', 11, ...
    'BackgroundColor', [0.3 0.7 0.3], 'ForegroundColor', 'w');
yPos = yPos - 0.10;
mkParamRow(tab3, 'Q₁₁ (Peso θ):', Q11, @onQ11, yPos);
yPos = yPos - 0.12;
mkParamRow(tab3, 'Q₂₂ (Peso dθ):', Q22, @onQ22, yPos);
yPos = yPos - 0.12;
mkParamRow(tab3, 'R (Costo control):', R, @onR, yPos);
yPos = yPos - 0.14;
uicontrol(tab3, 'Style', 'pushbutton', 'String', '🔄  Recalcular Ganancias K', ...
    'Position', [20 yPos*380 300 40], ...
    'FontSize', 10, ...
    'BackgroundColor', [0.92 0.98 0.92], ...
    'Callback', @recalcLQR);

% TAB 4
tab4 = uitab(tabGroup, 'Title', '🟠 Fuzzy', 'BackgroundColor', [1 0.98 0.96]);
yPos = 0.85;
uicontrol(tab4, 'Style', 'text', 'String', 'PARÁMETROS FUZZY', ...
    'Position', [20 yPos*380 760 28], 'FontWeight', 'bold', 'FontSize', 11, ...
    'BackgroundColor', [0.9 0.5 0.1], 'ForegroundColor', 'w');
yPos = yPos - 0.10;
mkParamRow(tab4, 'Ko (Escala):', Ko, @onKo, yPos);
yPos = yPos - 0.12;
mkParamRow(tab4, 'Kmin (Mínima):', Kmin, @onKmin, yPos);
yPos = yPos - 0.12;
mkParamRow(tab4, 'Kmax (Máxima):', Kmax, @onKmax, yPos);
yPos = yPos - 0.12;
mkParamRow(tab4, 'Tolerancia |θ| (grados):', tol_deg_fuz, @onTolFuzzy, yPos);
yPos = yPos - 0.14;
uicontrol(tab4, 'Style', 'text', 'String', 'Método de defuzzificación:', ...
    'Position', [20 yPos*380 250 24], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 9.5, ...
    'BackgroundColor', [1 0.98 0.96]);
popDefz = uicontrol(tab4, 'Style', 'popupmenu', ...
    'String', {'Centroid', 'Bisector'}, ...
    'Value', 1, ...
    'Position', [280 yPos*380 150 28], ...
    'FontSize', 10, ...
    'BackgroundColor', 'w', ...
    'Callback', @onDefuzzChanged);
yPos = yPos - 0.12;
uicontrol(tab4, 'Style', 'pushbutton', 'String', '🔧  Recrear FIS', ...
    'Position', [20 yPos*380 250 40], ...
    'FontSize', 10, ...
    'BackgroundColor', [1 0.96 0.95], ...
    'Callback', @onRefis);

%% ---------- Panel de ESTADO ----------
pStatus = uipanel('Parent', fig, 'Title', '📊 Estado Actual', ...
    'Units', 'normalized', ...
    'Position', [0.78 0.03 0.16 0.41], ...
    'BackgroundColor', [1 1 1], 'FontWeight', 'bold', 'FontSize', 10);

yP = 0.82;
sp = 0.18;

uicontrol(pStatus, 'Style', 'text', 'String', 'Ángulo θ:', ...
    'Units', 'normalized', 'Position', [0.05 yP 0.45 0.12], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [1 1 1]);
txtTheta = uicontrol(pStatus, 'Style', 'text', 'String', '180.0°', ...
    'Units', 'normalized', 'Position', [0.50 yP 0.45 0.12], ...
    'HorizontalAlignment', 'right', 'FontSize', 11, ...
    'BackgroundColor', [0.95 0.95 1], 'ForegroundColor', [0.8 0.2 0.2]);
yP = yP - sp;

uicontrol(pStatus, 'Style', 'text', 'String', 'Velocidad dθ:', ...
    'Units', 'normalized', 'Position', [0.05 yP 0.45 0.12], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [1 1 1]);
txtDTheta = uicontrol(pStatus, 'Style', 'text', 'String', '0.0°/s', ...
    'Units', 'normalized', 'Position', [0.50 yP 0.45 0.12], ...
    'HorizontalAlignment', 'right', 'FontSize', 11, ...
    'BackgroundColor', [0.95 1 0.95], 'ForegroundColor', [0.2 0.6 0.2]);
yP = yP - sp;

uicontrol(pStatus, 'Style', 'text', 'String', 'Torque u:', ...
    'Units', 'normalized', 'Position', [0.05 yP 0.45 0.12], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [1 1 1]);
txtTorque = uicontrol(pStatus, 'Style', 'text', 'String', '0.00 N·m', ...
    'Units', 'normalized', 'Position', [0.50 yP 0.45 0.12], ...
    'HorizontalAlignment', 'right', 'FontSize', 11, ...
    'BackgroundColor', [1 0.95 0.95], 'ForegroundColor', [0.6 0.2 0.6]);
yP = yP - sp;

uicontrol(pStatus, 'Style', 'text', 'String', 'Tiempo:', ...
    'Units', 'normalized', 'Position', [0.05 yP 0.45 0.12], ...
    'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 10, ...
    'BackgroundColor', [1 1 1]);
txtTime = uicontrol(pStatus, 'Style', 'text', 'String', '0.00 s', ...
    'Units', 'normalized', 'Position', [0.50 yP 0.45 0.12], ...
    'HorizontalAlignment', 'right', 'FontSize', 11, ...
    'BackgroundColor', [1 1 0.95], 'ForegroundColor', [0.3 0.3 0.3]);

%% ---------- Timer ----------
TMR = timer('ExecutionMode', 'fixedRate', ...
    'Period', 1/vis_hz, ...
    'TimerFcn', @onTick);

set(fig, 'CloseRequestFcn', @onClose);
isDragging     = false;
update_counter = 0;

%% ========== CALLBACKS ==========

    function setAutoStop(v)
        autoStop = logical(v);
    end

    function onModeChanged(~, ~)
        v = get(popCtrl, 'Value');
        if v == 3
            defuzzMethod = 'centroid';
            set(popDefz, 'Value', 1);
        elseif v == 4
            defuzzMethod = 'bisector';
            set(popDefz, 'Value', 2);
        end
        refreshPlotAll();
    end

    function onStart(~, ~)
        set(hConv, 'String', '', 'Visible', 'off');
        if strcmp(TMR.Running, 'off')
            start(TMR);
        end
    end

    function onStop(~, ~)
        if strcmp(TMR.Running, 'on')
            stop(TMR);
            % ➜ Al detener manualmente, mostrar la gráfica de Error vs Tiempo
            plotErrorAfterStop();
        end
    end

    function onReset(~, ~)
        onStop();
        x = [deg2rad(wrapTo180_local(theta0_deg)); 0];
        t = 0;
        th = []; uh = []; tt = [];
        u_act = 0;
        Ierr  = 0;
        ok_cnt = 0;
        update_counter = 0;
        set(hConv, 'String', '', 'Visible', 'off');
        refreshSim(u_act);
        refreshPlotAll();
    end

    function onSaveCSV(~, ~)
        wasRunning = strcmp(TMR.Running, 'on');
        if wasRunning, stop(TMR); end

        outdir = uigetdir('', 'Selecciona carpeta para guardar resultados batch');
        if isequal(outdir, 0)
            if wasRunning, start(TMR); end
            return;
        end

        P = struct();
        P.m = m; P.L = L; P.g = g; P.b = b; P.I = I;
        P.u_max = u_max; P.u_rng = [-u_max, u_max];
        P.tau_u = tau_u; P.max_udot = max_udot;
        P.dt = dt; P.substeps = substeps;
        P.Kp = Kp; P.Ki = Ki; P.Kd = Kd;
        try
            P.K_lqr = lqr(A, B, diag([Q11, Q22]), R);
        catch
            P.K_lqr = [10 2];
        end
        P.Ko = Ko; P.Kmin = Kmin; P.Kmax = Kmax;
        P.defuzzMethod = defuzzMethod;
        P.fis = fis;
        P.tol_deg_fuz = tol_deg_fuz;
        P.thr_th_pid_lqr = thr_th_deg_pid_lqr;
        P.thr_dth_deg    = thr_dth_deg;
        P.dwell_s        = dwell_s;

        % tiempos SOLO del batch
        P.tmax_pid_batch  = batch_tmax_pid;
        P.tmax_lqr_batch  = batch_tmax_lqr;
        P.tmax_fuzC_batch = batch_tmax_fuzC;
        P.tmax_fuzB_batch = batch_tmax_fuzB;

        try
            % 👇 ahora SÍ las abre
            simulate_compare_and_save(outdir, P, true);
            msgbox('Batch completado exitosamente', 'Éxito', 'help');
        catch ME
            errordlg(sprintf('Error en batch: %s', ME.message), 'Error');
        end

        if wasRunning, start(TMR); end
    end

    function onThetaEdit(src, ~)
        v = str2double(src.String);
        if isnan(v)
            src.String = num2str(theta0_deg, '%.1f');
            return;
        end
        v = max(-180, min(180, v));
        theta0_deg = v;
        src.String = num2str(v, '%.1f');
    end

    function onToggleDrag(src, ~)
        if src.Value == 1
            if strcmp(TMR.Running, 'on')
                src.Value = 0;
                warndlg('Detén la simulación para arrastrar.', 'Advertencia');
                return;
            end
            set(fig, 'WindowButtonDownFcn', @startDrag);
            set(fig, 'WindowButtonUpFcn',   @stopDrag);
            set(fig, 'WindowButtonMotionFcn', @doDrag);
        else
            set(fig, 'WindowButtonDownFcn', []);
            set(fig, 'WindowButtonUpFcn',   []);
            set(fig, 'WindowButtonMotionFcn', []);
            isDragging = false;
        end
    end

    function startDrag(~, ~)
        if strcmp(TMR.Running, 'on'), return; end
        isDragging = true;
        doDrag();
    end

    function stopDrag(~, ~)
        isDragging = false;
    end

    function doDrag(~, ~)
        if ~isDragging, return; end
        cp = get(axSim, 'CurrentPoint');
        X = cp(1, 1); Y = cp(1, 2);
        ang = atan2(X, Y);
        x(1) = ang;
        x(2) = 0;
        theta0_deg = wrapTo180_local(rad2deg(ang));
        set(editTheta, 'String', num2str(theta0_deg, '%.1f'));
        refreshSim(u_act);
        drawnow;
    end

    % PID
    function onKp(src, ~)
        v = str2double(src.String);
        if ~isnan(v), Kp = v; end
        src.String = num2str(Kp, '%.3g');
    end

    function onKi(src, ~)
        v = str2double(src.String);
        if ~isnan(v), Ki = v; end
        src.String = num2str(Ki, '%.3g');
    end

    function onKd(src, ~)
        v = str2double(src.String);
        if ~isnan(v), Kd = v; end
        src.String = num2str(Kd, '%.3g');
    end

    % LQR
    function onQ11(src, ~)
        v = str2double(src.String);
        if ~isnan(v) && v > 0, Q11 = v; end
        src.String = num2str(Q11, '%.3g');
        recalcLQR();
    end

    function onQ22(src, ~)
        v = str2double(src.String);
        if ~isnan(v) && v > 0, Q22 = v; end
        src.String = num2str(Q22, '%.3g');
        recalcLQR();
    end

    function onR(src, ~)
        v = str2double(src.String);
        if ~isnan(v) && v > 0, R = v; end
        src.String = num2str(R, '%.3g');
        recalcLQR();
    end

    function recalcLQR(~, ~)
        Q = diag([Q11, Q22]);
        try
            K_lqr = lqr(A, B, Q, R);
        catch
            warndlg('Parámetros Q o R no válidos para lqr(); se mantiene K anterior.', 'Advertencia');
        end
    end

    % FUZZY
    function onKo(src, ~)
        v = str2double(src.String);
        if ~isnan(v), Ko = v; end
        src.String = num2str(Ko, '%.3g');
    end

    function onKmin(src, ~)
        v = str2double(src.String);
        if ~isnan(v), Kmin = v; end
        src.String = num2str(Kmin, '%.3g');
    end

    function onKmax(src, ~)
        v = str2double(src.String);
        if ~isnan(v), Kmax = v; end
        src.String = num2str(Kmax, '%.3g');
    end

    function onDefuzzChanged(src, ~)
        items = {'centroid', 'bisector'};
        defuzzMethod = items{get(src, 'Value')};
    end

    function onTolFuzzy(src, ~)
        v = str2double(src.String);
        if isnan(v) || v <= 0
            src.String = num2str(tol_deg_fuz, '%.3g');
            return;
        end
        tol_deg_fuz = v;
    end

    function onRefis(~, ~)
        fis = construirFIS_struct(theta_rng, dtheta_rng, [-u_max, u_max]);
        msgbox('FIS recreado exitosamente', 'Información', 'help');
    end

%% ========== SIMULACIÓN (TIMER) ==========
    function onTick(~, ~)
        for k = 1:substeps
            theta  = wrapToPi_local(x(1));
            dtheta = x(2);

            mode = get(popCtrl, 'Value');

            switch mode
                case 1  % PID
                    e  = theta;
                    de = dtheta;
                    u_pi_test = -(Kp*e + Kd*de + Ierr);
                    sat_hit   = (abs(u_pi_test) >= u_max - 1e-9);
                    same_dir  = (sign(u_pi_test) == sign(e) && e ~= 0);
                    if ~(sat_hit && same_dir)
                        Ierr = clip(Ierr + Ki*e*dt, -u_max, u_max);
                    end
                    u_cmd = clip(-(Kp*e + Kd*de + Ierr), -u_max, u_max);

                case 2  % LQR
                    u_cmd = clip(-K_lqr * [theta; dtheta], -u_max, u_max);

                case {3, 4}  % Fuzzy
                    Ko_eff = clip(Ko, Kmin, Kmax);
                    if mode == 3
                        method = 'centroid';
                    else
                        method = 'bisector';
                    end
                    u_fuz = mamdani_eval_crisp_manual(fis, [theta, dtheta], u_rng, method);
                    u_cmd = clip(Ko_eff * u_fuz, -u_max, u_max);
            end

            udot  = clip((u_cmd - u_act)/tau_u, -max_udot, max_udot);
            u_act = clip(u_act + dt*udot, -u_max, u_max);

            xdot = [dtheta; (u_act - b*dtheta - m*g*L*sin(theta))/I];
            x    = x + dt*xdot;
            t    = t + dt;
        end

        mode = get(popCtrl, 'Value');
        if mode >= 3
            thr_th = deg2rad(tol_deg_fuz);
        else
            thr_th = deg2rad(thr_th_deg_pid_lqr);
        end
        thr_dth = deg2rad(thr_dth_deg);

        if (abs(theta) < thr_th) && (abs(dtheta) < thr_dth)
            ok_cnt = ok_cnt + 1;
            if ok_cnt >= dwell_N && autoStop && strcmp(TMR.Running, 'on')
                stop(TMR);
                if mode >= 3
                    condStr = sprintf('|θ| < %g° (Fuzzy) y |dθ| < %g°/s', ...
                                      tol_deg_fuz, thr_dth_deg);
                else
                    condStr = sprintf('|θ| < %g° (PID/LQR) y |dθ| < %g°/s', ...
                                      thr_th_deg_pid_lqr, thr_dth_deg);
                end
                set(hConv, 'String', sprintf('Convergencia: %s durante %.2f s (t = %.3f s)', ...
                    condStr, dwell_s, t), 'Visible', 'on');

                % ➜ Al auto-detener por convergencia, mostrar Error vs Tiempo
                plotErrorAfterStop();
            end
        else
            ok_cnt = 0;
        end

        th(end+1, 1) = wrapToPi_local(x(1));
        uh(end+1, 1) = u_act;
        tt(end+1, 1) = t;

        update_counter = update_counter + 1;
        if update_counter >= 2
            update_counter = 0;
            refreshSim(u_act);
            refreshPlotAll();
        end
    end

    function refreshSim(u_disp)
        [bx, by] = xy_from_theta(L, x(1));
        set(hRod, 'XData', [0 bx], 'YData', [0 by]);
        set(hBob, 'XData', bx, 'YData', by);

        th_deg  = wrapTo180_local(rad2deg(x(1)));
        dth_deg = rad2deg(x(2));

        set(txtTheta,  'String', sprintf('%.2f°', th_deg));
        set(txtDTheta, 'String', sprintf('%.2f°/s', dth_deg));
        set(txtTorque, 'String', sprintf('%.3f N·m', u_disp));
        set(txtTime,   'String', sprintf('%.2f s', t));

        if abs(th_deg) < 5
            set(txtTheta, 'BackgroundColor', [0.8 1 0.8], 'ForegroundColor', [0 0.6 0]);
        elseif abs(th_deg) < 20
            set(txtTheta, 'BackgroundColor', [1 1 0.8], 'ForegroundColor', [0.8 0.5 0]);
        else
            set(txtTheta, 'BackgroundColor', [1 0.9 0.9], 'ForegroundColor', [0.8 0.2 0.2]);
        end

        modeStr = {'PID', 'LQR', 'Fuzzy-Centroid', 'Fuzzy-Bisector'};
        ms = modeStr{get(popCtrl, 'Value')};

        set(hTxt, 'String', sprintf( ...
            'Modo: %s\nθ = %.2f° | dθ = %.2f°/s\nt = %.2f s | u = %.3f N·m', ...
            ms, th_deg, dth_deg, t, u_disp));

        drawnow limitrate;
    end

    function refreshPlotAll()
        if isempty(tt)
            mode_now = get(popCtrl, 'Value');
            xwin = pickXwin(mode_now);
            xlim(axAll, [0 xwin]);
            yyaxis(axAll, 'left');
            ylim(axAll, [-180 180]);
            return;
        end

        Nmax = 5000;
        n = numel(tt);
        idx_step = max(1, ceil(n/Nmax));
        idx = 1:idx_step:n;

        yyaxis(axAll, 'left');
        set(lineTh, 'XData', tt(idx), 'YData', wrapTo180_local(rad2deg(th(idx))));
        ylim(axAll, [-180 180]);

        yyaxis(axAll, 'right');
        set(lineU, 'XData', tt(idx), 'YData', uh(idx));

        mode_now = get(popCtrl, 'Value');
        xwin = pickXwin(mode_now);
        xlim(axAll, [0 xwin]);
    end

    function xw = pickXwin(mode_id)
        switch mode_id
            case 1
                xw = tmax_pid;
            case 2
                xw = tmax_lqr;
            case 3
                xw = tmax_fuzC;
            case 4
                xw = tmax_fuzB;
            otherwise
                xw = 4.0;
        end
    end

    function onClose(~, ~)
        try
            if strcmp(TMR.Running, 'on')
                stop(TMR);
            end
            delete(TMR);
        catch
        end
        delete(fig);
    end

    %% ---------- NUEVO: Gráfica automática Error vs Tiempo ----------
    function plotErrorAfterStop()
        if isempty(tt) || isempty(th)
            return;
        end
        mode_now = get(popCtrl, 'Value');
        xwin = pickXwin(mode_now);

        % Error = θ (respecto a 0 rad), en grados envuelto a [-180,180]
        tplot = tt(:);
        e_deg = wrapTo180_local(rad2deg(th(:)));

        % Recorte al horizonte del modo (por estética)
        mask = (tplot >= 0) & (tplot <= xwin + eps);
        tplot = tplot(mask);
        e_deg = e_deg(mask);

        figE = figure('Color','w','Position',[160 160 920 420], 'Name','Error vs Tiempo');
        axE = axes(figE); hold(axE,'on'); grid(axE,'on'); box(axE,'on');
        plot(axE, tplot, e_deg, 'LineWidth', 2.5);
        yline(axE, 0, '--k', 'LineWidth', 1.5, 'HandleVisibility', 'off');
        xlabel(axE, 'Tiempo (s)', 'FontWeight','bold');
        ylabel(axE, 'Error \theta (grados)', 'FontWeight','bold');

        mnames = {'PID','LQR','Fuzzy - Centroid','Fuzzy - Bisector'};
        title(axE, sprintf('Error angular vs tiempo — %s', mnames{mode_now}), ...
            'FontWeight','bold','FontSize',12);

        xlim(axE, [0, max(xwin, max(tplot))]);
        ylim(axE, 'tight');

        % Texto con error final
        if ~isempty(tplot)
            txt = sprintf('Error final: %.2f° a t=%.2fs', e_deg(end), tplot(end));
            text(axE, 0.98*max(tplot), e_deg(end), txt, ...
                'HorizontalAlignment','right','VerticalAlignment','bottom', ...
                'FontSize',9,'BackgroundColor',[1 1 1 0.8]);
        end
        drawnow;
    end

end  % fin función principal

%% ========== FUNCIONES AUXILIARES ==========

function mkParamRow(parent, label, defaultVal, callback, yPos)
    bgColor = get(parent, 'BackgroundColor');
    uicontrol(parent, 'Style', 'text', 'String', label, ...
        'Position', [20 yPos*380 250 24], ...
        'HorizontalAlignment', 'left', 'FontWeight', 'bold', 'FontSize', 9.5, ...
        'BackgroundColor', bgColor);
    uicontrol(parent, 'Style', 'edit', ...
        'String', num2str(defaultVal, '%.3g'), ...
        'Position', [280 yPos*380 150 28], ...
        'FontSize', 10, ...
        'BackgroundColor', 'w', ...
        'Callback', callback);
end

function fis = construirFIS_struct(theta_rng, dtheta_rng, u_rng)
    fis = struct();

    fis.theta.range = theta_rng;
    fis.theta.c = [-pi/2, -pi/6, 0, pi/6, pi/2];
    fis.theta.s = [pi/6, pi/8, pi/10, pi/8, pi/6];

    fis.dtheta.range = dtheta_rng;
    fis.dtheta.c = [-8, -3, 0, 3, 8];
    fis.dtheta.s = [4, 3, 2, 3, 4];

    fis.u.range = u_rng;
    c = linspace(u_rng(1), u_rng(2), 5);
    fis.u.trimf = [ ...
        u_rng(1), u_rng(1), c(2);
        u_rng(1), c(2),     c(3);
        c(2),     c(3),     c(4);
        c(3),     c(4),     u_rng(2);
        c(4),     u_rng(2), u_rng(2) ...
    ];

    fis.ruleIdx = [ ...
        5 5 4 3 2;
        5 4 3 2 1;
        4 3 3 3 2;
        1 2 3 4 5;
        2 3 4 5 5 ...
    ];
end

function mu = gaussmf_local(x, sigma, c)
    mu = exp(-0.5*((x - c)./max(sigma, eps)).^2);
end

% corregida: no usa && ni || con vectores
function mu = trimf_local(x, a, b, c)
    mu = zeros(size(x));

    if b > a
        asc = (x >= a) & (x <= b);
        mu(asc) = (x(asc) - a) ./ (b - a);
    end

    if c > b
        des = (x > b) & (x <= c);
        mu(des) = (c - x(des)) ./ (c - b);
    end

    idx_peak = (x == b) & (b >= a) & (b <= c);
    mu(idx_peak) = 1;

    if a == b
        mu(x <= a) = 1;
    end
    if b == c
        mu(x >= c) = 1;
    end

    mu = max(mu, 0);
end

function u = mamdani_eval_crisp_manual(fis, in, u_rng, method)
    theta  = in(1);
    dtheta = in(2);

    z = linspace(u_rng(1), u_rng(2), 1001);
    agg = zeros(size(z));

    mu_t = zeros(1, 5);
    mu_d = zeros(1, 5);
    for i = 1:5
        mu_t(i) = gaussmf_local(theta,  fis.theta.s(i),  fis.theta.c(i));
        mu_d(i) = gaussmf_local(dtheta, fis.dtheta.s(i), fis.dtheta.c(i));
    end

    for i = 1:5
        for j = 1:5
            alpha = min(mu_t(i), mu_d(j));
            if alpha <= 0, continue; end
            kOut = fis.ruleIdx(i, j);
            tri  = fis.u.trimf(kOut, :);
            mu_o = trimf_local(z, tri(1), tri(2), tri(3));
            agg  = max(agg, min(alpha, mu_o));
        end
    end

    area = trapz(z, agg);
    if area <= eps
        u = 0;
        return;
    end

    switch lower(method)
        case 'centroid'
            u = trapz(z, z.*agg) / area;
        case 'bisector'
            target = area/2;
            cumA   = cumtrapz(z, agg);
            idx    = find(cumA >= target, 1, 'first');
            if isempty(idx) || idx == 1
                u = z(1);
            else
                u = z(idx-1) + (z(idx) - z(idx-1)) * ...
                    (target - cumA(idx-1)) / (cumA(idx) - cumA(idx-1) + eps);
            end
        otherwise
            u = trapz(z, z.*agg) / area;
    end
end

function [A, B] = linearize_upright(m, L, g, b, I)
    A = [0, 1; -(m*g*L)/I, -b/I];
    B = [0; 1/I];
end

function [x, y] = xy_from_theta(L, theta)
    x = L * sin(theta);
    y = L * cos(theta);
end

function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end

function ang = wrapTo180_local(ang)
    ang = mod(ang + 180, 360) - 180;
end

function v = clip(v, a, b)
    v = min(max(v, a), b);
end

%% ========== BATCH (4 MÉTODOS) ==========
function simulate_compare_and_save(outdir, P, showFigures)
    if nargin < 3, showFigures = false; end

    match_tol_in_batch = true;
    rng_seed_for_fair  = 42;

    modeDefs = { ...
        struct('name', 'PID',            'kind', 'PID',   'defuzz', ''), ...
        struct('name', 'LQR',            'kind', 'LQR',   'defuzz', ''), ...
        struct('name', 'Fuzzy-Centroid', 'kind', 'Fuzzy', 'defuzz', 'centroid'), ...
        struct('name', 'Fuzzy-Bisector', 'kind', 'Fuzzy', 'defuzz', 'bisector') ...
    };

    if isfield(P, 'tmax_pid_batch'),  Tmax_PID  = P.tmax_pid_batch;  else, Tmax_PID  = 5;  end
    if isfield(P, 'tmax_lqr_batch'),  Tmax_LQR  = P.tmax_lqr_batch;  else, Tmax_LQR  = 5;  end
    if isfield(P, 'tmax_fuzC_batch'), Tmax_FC   = P.tmax_fuzC_batch; else, Tmax_FC   = 20; end
    if isfield(P, 'tmax_fuzB_batch'), Tmax_FB   = P.tmax_fuzB_batch; else, Tmax_FB   = 20; end

    nruns = 12;

    deg     = @(x) rad2deg(x);
    colPID  = [0.20 0.40 0.85];
    colLQR  = [0.20 0.65 0.30];
    colFUZC = [0.90 0.35 0.10];
    colFUZB = [0.55 0.20 0.80];

    rng(rng_seed_for_fair);
    theta0_list = -170 + 340*rand(nruns, 1);

    P_batch = P;
    if match_tol_in_batch
        P_batch.tol_deg_fuz = P.thr_th_pid_lqr;
    end

    figErr = figure('Visible', ternary(showFigures, 'on', 'off'), ...
        'Color', 'w', 'Position', [80 80 1250 680]);
    tiledlayout(figErr, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    convTimes = struct('PID', [], 'LQR', [], 'FuzzyC', [], 'FuzzyB', []);

    for im = 1:numel(modeDefs)
        md = modeDefs{im};

        switch md.name
            case 'PID'
                Tmax_mode = Tmax_PID;
            case 'LQR'
                Tmax_mode = Tmax_LQR;
            case 'Fuzzy-Centroid'
                Tmax_mode = Tmax_FC;
            case 'Fuzzy-Bisector'
                Tmax_mode = Tmax_FB;
            otherwise
                Tmax_mode = 20;
        end

        ax = nexttile;
        hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
        title(ax, sprintf('Errores θ (grados) – %s', md.name), ...
            'FontWeight', 'bold', 'FontSize', 11);
        xlabel(ax, 'Tiempo (s)', 'FontWeight', 'bold');
        ylabel(ax, 'θ (grados)', 'FontWeight', 'bold');

        switch md.kind
            case 'PID'
                baseColor = colPID;
            case 'LQR'
                baseColor = colLQR;
            otherwise
                if strcmpi(md.defuzz, 'centroid')
                    baseColor = colFUZC;
                else
                    baseColor = colFUZB;
                end
        end

        cmap = zeros(nruns, 3);
        for r = 1:nruns
            alpha = (r-1)/max(nruns-1,1);
            cmap(r, :) = (1 - alpha) * baseColor + alpha * [1 1 1];
        end

        for r = 1:nruns
            th0_deg = theta0_list(r);
            x0 = [deg2rad(wrapTo180_local(th0_deg)); 0];

            if strcmp(md.kind, 'Fuzzy')
                [t, th, ~, tconv, converged] = sim_single_run('Fuzzy', P_batch, x0, Tmax_mode, md.defuzz);
            else
                [t, th, ~, tconv, converged] = sim_single_run(md.kind, P_batch, x0, Tmax_mode);
            end

            plot(ax, t, deg(wrapToPi_local(th)), ...
                'Color', cmap(r,:), ...
                'LineWidth', 1.4);

            if ~converged, tconv = Tmax_mode; end

            switch md.name
                case 'PID'
                    convTimes.PID(end+1,1) = tconv;
                case 'LQR'
                    convTimes.LQR(end+1,1) = tconv;
                case 'Fuzzy-Centroid'
                    convTimes.FuzzyC(end+1,1) = tconv;
                case 'Fuzzy-Bisector'
                    convTimes.FuzzyB(end+1,1) = tconv;
            end
        end

        yline(ax, 0, '--k', 'LineWidth', 1.2, 'Alpha', 0.6);
        xlim(ax, [0 Tmax_mode]);
    end

    exportgraphics(figErr, fullfile(outdir, 'errores_overlays.png'), 'Resolution', 150);

    figSum = figure('Visible', ternary(showFigures, 'on', 'off'), ...
        'Color', 'w', 'Position', [150 150 1000 550]);

    mnames = {'PID', 'LQR', 'Fuzzy-Centroid', 'Fuzzy-Bisector'};
    cols   = [colPID; colLQR; colFUZC; colFUZB];
    Ntot   = nruns * ones(1,4);
    dataCell = {convTimes.PID, convTimes.LQR, convTimes.FuzzyC, convTimes.FuzzyB};

    means = zeros(1,4);
    stdev = zeros(1,4);
    nconv = zeros(1,4);
    for i = 1:4
        di = dataCell{i};
        if isempty(di)
            di = NaN;
        end
        means(i) = mean(di, 'omitnan');
        stdev(i) = std(di, 'omitnan');
        switch i
            case 1, Tmax_i = Tmax_PID;
            case 2, Tmax_i = Tmax_LQR;
            case 3, Tmax_i = Tmax_FC;
            case 4, Tmax_i = Tmax_FB;
        end
        nconv(i) = sum(di < Tmax_i - 1e-6);
        dataCell{i} = di;
    end

    ax2 = axes(figSum);
    hold(ax2, 'on'); grid(ax2, 'on'); box(ax2, 'on');
    title(ax2, 'Tiempos de Convergencia por Método (Promedio ± DE)', ...
        'FontWeight', 'bold', 'FontSize', 12);
    ylabel(ax2, 'Tiempo de convergencia (s)', 'FontWeight', 'bold', 'FontSize', 11);

    xpos = 1:4;
    hb = bar(ax2, xpos, means, 0.65, 'FaceColor', 'flat', ...
        'EdgeColor', 'k', 'LineWidth', 1.2);
    for i = 1:4
        hb.CData(i,:) = cols(i,:);
    end
    xticks(ax2, xpos);
    xticklabels(ax2, mnames);

    errorbar(ax2, xpos, means, stdev, stdev, ...
        'LineStyle', 'none', 'LineWidth', 2.0, 'Color', [0 0 0 0.7], 'CapSize', 12);

    rng(11);
    for i = 1:4
        di = dataCell{i};
        if all(isnan(di)), continue; end
        jitterX = (rand(size(di)) - 0.5) * 0.28;
        scatter(ax2, xpos(i) + jitterX, di, 32, 'filled', ...
            'MarkerFaceColor', cols(i,:), ...
            'MarkerFaceAlpha', 0.75, ...
            'MarkerEdgeColor', 'k', ...
            'MarkerEdgeAlpha', 0.25, ...
            'LineWidth', 0.5);
    end

    yTop = max(means + stdev);
    if isnan(yTop) || yTop == 0
        yTop = 1;
    end
    ylim(ax2, [0, yTop*1.30]);

    for i = 1:4
        txt = sprintf('%d/%d conv.', nconv(i), Ntot(i));
        ytxt = means(i) + (stdev(i) > 0) * stdev(i);
        if ytxt == 0 || isnan(ytxt), ytxt = 0.02*yTop; end
        text(ax2, xpos(i), ytxt*1.08 + eps, txt, ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', ...
            'FontWeight', 'bold', ...
            'FontSize', 9.5, ...
            'Color', [0.2 0.2 0.2]);
    end

    exportgraphics(figSum, fullfile(outdir, 'tiempos_convergencia_resumen.png'), 'Resolution', 150);

    % ---------- CSV (ARREGLADO) ----------
    T_all = table('Size', [0 4], ...
                  'VariableTypes', {'string','double','double','double'}, ...
                  'VariableNames', {'Metodo','Run','TiempoConv_s','Convergio'});

    for i = 1:4
        switch i
            case 1, Tmax_i = Tmax_PID;
            case 2, Tmax_i = Tmax_LQR;
            case 3, Tmax_i = Tmax_FC;
            case 4, Tmax_i = Tmax_FB;
        end
        di = dataCell{i};
        for r = 1:numel(di)
            T_all = [T_all; {string(mnames{i}), r, di(r), double(di(r) < Tmax_i - 1e-6)}]; %#ok<AGROW>
        end
    end

    writetable(T_all, fullfile(outdir, 'resultados_batch.csv'));

    fprintf('✅ Batch completado. Archivos guardados en: %s\n', outdir);
end

function y = ternary(cond, a, b)
    if cond, y = a; else, y = b; end
end

function [t, th, u, tconv, converged] = sim_single_run(modo, P, x0, Tmax, defuzzOverride)
    if nargin < 5, defuzzOverride = ''; end

    dt = P.dt;
    N  = max(1, round(Tmax/dt));

    x = x0;
    u_act = 0;

    t  = zeros(N,1);
    th = zeros(N,1);
    u  = zeros(N,1);

    Ierr = 0;

    thr_th  = deg2rad(P.thr_th_pid_lqr);
    if strcmpi(modo, 'Fuzzy')
        thr_th = deg2rad(P.tol_deg_fuz);
    end
    thr_dth = deg2rad(P.thr_dth_deg);
    dwell_N = ceil(P.dwell_s/dt);
    ok_cnt  = 0;

    tconv = NaN;
    converged = false;

    for k = 1:N
        theta  = wrapToPi_local(x(1));
        dtheta = x(2);

        switch upper(modo)
            case 'PID'
                e  = theta;
                de = dtheta;
                u_pi_test = -(P.Kp*e + P.Kd*de + Ierr);
                sat_hit   = (abs(u_pi_test) >= P.u_max - 1e-9);
                same_dir  = (sign(u_pi_test) == sign(e) && e ~= 0);
                if ~(sat_hit && same_dir)
                    Ierr = clip(Ierr + P.Ki*e*dt, -P.u_max, P.u_max);
                end
                u_cmd = clip(-(P.Kp*e + P.Kd*de + Ierr), -P.u_max, P.u_max);

            case 'LQR'
                u_cmd = clip(-P.K_lqr * [theta; dtheta], -P.u_max, P.u_max);

            case 'FUZZY'
                Ko_eff = clip(P.Ko, P.Kmin, P.Kmax);
                method = P.defuzzMethod;
                if ~isempty(defuzzOverride)
                    method = defuzzOverride;
                end
                u_fuz = mamdani_eval_crisp_manual(P.fis, [theta, dtheta], P.u_rng, method);
                u_cmd = clip(Ko_eff * u_fuz, -P.u_max, P.u_max);

            otherwise
                error('Modo desconocido.');
        end

        udot  = clip((u_cmd - u_act)/P.tau_u, -P.max_udot, P.max_udot);
        u_act = clip(u_act + dt*udot, -P.u_max, P.u_max);

        xdot = [dtheta; (u_act - P.b*dtheta - P.m*P.g*P.L*sin(theta))/P.I];
        x    = x + dt*xdot;

        t(k)  = (k-1)*dt;
        th(k) = theta;
        u(k)  = u_act;

        if (abs(theta) < thr_th) && (abs(dtheta) < thr_dth)
            ok_cnt = ok_cnt + 1;
            if ok_cnt >= dwell_N && ~converged
                tconv    = t(k);
                converged = true;
            end
        else
            ok_cnt = 0;
        end
    end

    last = find(t > 0 | (1:numel(t))' == 1, 1, 'last');
    t  = t(1:last);
    th = th(1:last);
    u  = u(1:last);
end
