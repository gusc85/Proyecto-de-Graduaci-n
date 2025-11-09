function Controlador_Mamdani

    %======================
    % FIS Mamdani (ventilador)
    %======================
    % Definición de FIS con nombre descriptivo
    fis = mamfis('Name', 'ControlVentilador');

    % --- Entrada 1: Temperatura ---
    % Definición del rango operativo (0 a 100 °C) y construcción de tres MFs triangulares
    fis = addInput(fis, [0 100], 'Name', 'Temperatura');
    fis = addMF(fis, 'Temperatura', 'trimf', [0 0 50],   'Name', 'Frio');        % Definición de MF: Frio
    fis = addMF(fis, 'Temperatura', 'trimf', [25 50 75], 'Name', 'Templado');    % Definición de MF: Templado
    fis = addMF(fis, 'Temperatura', 'trimf', [50 100 100],'Name', 'Caliente');   % Definición de MF: Caliente

    % --- Entrada 2: Humedad ---
    % Definición del rango operativo (0 a 100 %) y construcción de tres MFs triangulares
    fis = addInput(fis, [0 100], 'Name', 'Humedad');
    fis = addMF(fis, 'Humedad', 'trimf', [0 0 50],    'Name', 'Seco');           % Definición de MF: Seco
    fis = addMF(fis, 'Humedad', 'trimf', [25 50 75],  'Name', 'Medio');          % Definición de MF: Medio
    fis = addMF(fis, 'Humedad', 'trimf', [50 100 100],'Name', 'Humedo');         % Definición de MF: Humedo

    % --- Salida: Velocidad del ventilador ---
    % Definición de la salida en 0–10 (escala de velocidad) y construcción de MFs triangulares
    fis = addOutput(fis, [0 10], 'Name', 'Velocidad');
    fis = addMF(fis, 'Velocidad', 'trimf', [0 0 5],      'Name', 'Lento');       % Definición de MF: Lento
    fis = addMF(fis, 'Velocidad', 'trimf', [2.5 5 7.5],  'Name', 'Medio');       % Definición de MF: Medio
    fis = addMF(fis, 'Velocidad', 'trimf', [7 10 10],    'Name', 'Rapido');      % Definición de MF: Rapido

    % --- Reglas ---
    % Construcción de la base de reglas empleando índices de MFs por orden de declaración:
    % [idx_Temp idx_Hum idx_Vel peso conector], con conector=1 (AND), peso=1
    ruleList = [
        1 1 1 1 1;  % Frio & Seco     -> Lento
        1 2 2 1 1;  % Frio & Medio    -> Medio
        1 3 2 1 1;  % Frio & Humedo   -> Medio
        2 1 2 1 1;  % Templado & Seco -> Medio
        2 2 2 1 1;  % Templado & Medio-> Medio
        2 3 3 1 1;  % Templado & Humedo-> Rapido
        3 1 2 1 1;  % Caliente & Seco -> Medio
        3 2 3 1 1;  % Caliente & Medio-> Rapido
        3 3 3 1 1;  % Caliente & Humedo-> Rapido
    ];
    % Inserción de reglas en el FIS
    fis = addRule(fis, ruleList);

    %======================
    % GUI
    %======================
    % Construcción de ventana principal de la interfaz
    fig = uifigure('Name', 'Sistema Difuso Mamdani', 'Position', [100 100 950 520]);

    % Colocación de botón con estado para control de simulación automática
    toggle = uibutton(fig, 'state', 'Text', '▶ Simulación', 'Position', [750 190 150 30], ...
        'ValueChangedFcn', @(btn,~) toggleSimulacion(btn));

    % Configuración de temporizador para barrido periódico de temperatura y humedad
    simTimer = timer('ExecutionMode', 'fixedRate', 'Period', 0.3, 'TimerFcn', @(~,~) simStep());

    % Definición de estado compartido para la lógica de barrido
    shared.temp = 0;
    shared.hum = 0;
    shared.sentidoTemp = 1;
    shared.sentidoHum  = 1;
    shared.lastChanged = 'temp';  % Inicialización de variable activa por defecto

    % Construcción de panel principal para visualización de MFs y LED de estado
    panel = uipanel(fig, 'Position', [25 130 550 260], 'Title', 'Visualización Difusa');

    % --- LED de estado ---
    % Preparación de ejes compactos y creación de indicador circular coloreado
    ledAxes = uiaxes(panel, 'Position', [390 80 80 80]);
    axis(ledAxes, [0 1 0 1]); axis(ledAxes, 'off'); axis(ledAxes, 'equal');
    ledCircle = rectangle(ledAxes, 'Position', [0 0 1 1], 'Curvature', [1 1], ...
        'FaceColor', [0.8 0.8 0.8], 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 2);
    ledText = text(ledAxes, 0.5, 0.5, '', 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontWeight', 'bold', 'FontSize', 10, 'Color', 'w');

    % --- Ejes para funciones de membresía (variable activa) ---
    % Construcción de ejes principales para trazado de MFs
    ax = axes(panel, 'Position', [0.1 0.15 0.65 0.75]);

    % --- Medidor semicircular para salida ---
    % Configuración de gauge (0–10) para reflejar velocidad de ventilador
    medidor = uigauge(fig, 'semicircular', 'Position', [615 360 140 100]);
    medidor.Limits = [0 10];
    medidor.MajorTicks = 0:2:10;
    medidor.ScaleColors = {[0 0.6 1], [0.2 0.8 0.2], [1 0.4 0.4]};
    medidor.ScaleColorLimits = [0 4; 4 7; 7 10];

    % Colocación de etiqueta numérica para la salida evaluada
    salidaLabel = uilabel(fig, 'Position', [710 358 150 22], 'Text', 'Velocidad: 0.00', ...
        'FontSize', 11, 'FontWeight', 'bold');

    % --- Tabla de reglas (ilustrativa) ---
    % Construcción de tabla con evaluaciones puntuales representativas
    tablaReglas = uitable(fig, 'Position', [615 290 190 50], ...
        'ColumnName', {'Regla', 'Grado'}, 'ColumnWidth', {220, 50});

    % --- Panel informativo (pertenencias de variable activa) ---
    % Construcción de panel con resumen de pertenencias actuales
    infoPanel = uipanel(fig, 'Title', '', 'Position', [616 140 160 140], ...
        'BackgroundColor', [0.94 0.97 1], 'BorderType', 'line');

    labelTitulo = uilabel(infoPanel, 'Text', '🧊 Estado Actual', ...
        'FontWeight', 'bold', 'FontSize', 12, 'Position', [10 105 140 20], ...
        'HorizontalAlignment', 'left', 'BackgroundColor', [0.94 0.97 1]);

    labelTemp = uilabel(infoPanel, 'Position', [10 80 140 22], 'Text', 'Temp: 0.0 °C', ...
        'FontSize', 14, 'FontWeight', 'bold', 'FontColor', [0.1 0.2 0.5], 'BackgroundColor', [0.94 0.97 1]);

    labelFrio     = uilabel(infoPanel, 'Position', [10 55 130 18], 'Text', 'Frio: 0.00',     'BackgroundColor', [0.94 0.97 1]);
    labelTemplado = uilabel(infoPanel, 'Position', [10 35 130 18], 'Text', 'Templado: 0.00','BackgroundColor', [0.94 0.97 1]);
    labelCaliente = uilabel(infoPanel, 'Position', [10 15 130 18], 'Text', 'Caliente: 0.00','BackgroundColor', [0.94 0.97 1]);

    % --- Controles tipo “knob” para ajuste manual ---
    % Colocación de controles para modificar temperatura y humedad
    uilabel(fig, 'Position', [680 85 60 20], 'Text', 'Temp.');
    knobTemp = uiknob(fig, 'continuous', 'Position', [680 50 35 35], 'Limits', [0 100], 'MajorTicks', 0:25:100);

    uilabel(fig, 'Position', [750 85 60 20], 'Text', 'Humedad');
    knobHum  = uiknob(fig, 'continuous', 'Position', [750 50 35 35], 'Limits', [0 100], 'MajorTicks', 0:25:100);

    % --- Botón para superficie 3D y exportación ---
    % Colocación de botón para visualizar superficie difusa y exportar como EPS
    botonSuperficie = uibutton(fig, 'Text', 'Ver superficie 3D', ...
        'Position', [750 230 150 30], 'ButtonPushedFcn', @(btn, ~) mostrarSuperficie3D(fis));

    % --- Conexión de callbacks de knobs ---
    % Asignación de funciones para refrescar la visualización según variable activa
    knobTemp.ValueChangedFcn = @onTempChanged;
    knobHum.ValueChangedFcn  = @onHumChanged;

    % Render inicial de la visualización con valores por defecto
    actualizarVisual();

    %======================
    % Funciones internas (callbacks y actualización)
    %======================

    function onTempChanged(~,~)
        % Actualización de variable activa a temperatura y redibujo de elementos
        shared.lastChanged = 'temp';
        actualizarVisual();
    end

    function onHumChanged(~,~)
        % Actualización de variable activa a humedad y redibujo de elementos
        shared.lastChanged = 'hum';
        actualizarVisual();
    end

    function simStep()
        % Actualización periódica de temperatura y humedad (barrido) con inversión de sentido en límites
        paso = 5;

        % Actualización de temperatura con saturación y cambio de sentido
        shared.temp = shared.temp + paso * shared.sentidoTemp;
        if shared.temp >= 100 || shared.temp <= 0
            shared.sentidoTemp = -shared.sentidoTemp;
            shared.temp = max(min(shared.temp, 100), 0);
        end

        % Actualización de humedad con saturación y cambio de sentido
        shared.hum = shared.hum + paso * shared.sentidoHum;
        if shared.hum >= 100 || shared.hum <= 0
            shared.sentidoHum = -shared.sentidoHum;
            shared.hum = max(min(shared.hum, 100), 0);
        end

        % Sincronización de knobs y refresco de visualización
        knobTemp.Value = shared.temp;
        knobHum.Value  = shared.hum;
        actualizarVisual();
    end

    function toggleSimulacion(btn)
        % Conmutación de estado de simulación (inicio/pausa) mediante temporizador
        if btn.Value
            start(simTimer);
            btn.Text = '⏸ Pausar';
        else
            stop(simTimer);
            btn.Text = '▶ Simulación';
        end
    end

    function actualizarVisual()
        % Lectura de entradas actuales, evaluación del FIS y actualización de:
        % medidor, etiqueta numérica, tabla ilustrativa y funciones de membresía.

        % Adquisición de valores de temperatura y humedad desde los knobs
        temp     = knobTemp.Value;
        humedad  = knobHum.Value;

        % Evaluación del sistema difuso con el par [Temperatura, Humedad]
        salida   = evalfis(fis, [temp, humedad]);

        % Refresco de gauge y etiqueta con el valor de salida
        medidor.Value     = salida;
        salidaLabel.Text  = sprintf('Velocidad: %.2f', salida);

        % Preparación de tabla con evaluaciones en puntos representativos
        tablaReglas.Data = {
            'Frio y Seco -- Lento',       evalfis(fis, [0,   0]);
            'Frio y Medio -- Medio',      evalfis(fis, [0,  50]);
            'Frio y Humedo -- Medio',     evalfis(fis, [0, 100]);
            'Templado y Seco -- Medio',   evalfis(fis, [50,  0]);
            'Templado y Medio -- Medio',  evalfis(fis, [50, 50]);
            'Templado y Humedo -- Rapido',evalfis(fis,[50,100]);
            'Caliente y Seco -- Medio',   evalfis(fis, [100, 0]);
            'Caliente y Medio -- Rapido', evalfis(fis,[100,50]);
            'Caliente y Humedo -- Rapido',evalfis(fis,[100,100]);
        };

        % Preparación de eje X para trazado de funciones de membresía
        x = linspace(0, 100, 500);
        cla(ax, 'reset'); hold(ax, 'on');

        if strcmp(shared.lastChanged,'hum')
            %----- HUMEDAD (variable activa) -----
            % Construcción de MFs y cálculo de grados de pertenencia para humedad actual
            mfs    = {trimf(x,[0 0 50]), trimf(x,[25 50 75]), trimf(x,[50 100 100])};
            grados = [trimf(humedad,[0 0 50]), trimf(humedad,[25 50 75]), trimf(humedad,[50 100 100])];

            % Actualización de etiquetas informativas
            labelTemp.Text        = sprintf('Hum: %.1f %%', humedad);
            labelFrio.Text        = sprintf('Seco: %.2f',    grados(1));
            labelTemplado.Text    = sprintf('Medio: %.2f',   grados(2));
            labelCaliente.Text    = sprintf('Humedo: %.2f',  grados(3));

            % Selección de color e icono del LED según MF dominante
            colores = {[0.6 0.8 1.0], [0.5 0.75 0.5], [0.4 0.6 0.9]};
            iconos  = {'💧','💧💧','💧💧💧'};
            [~, idx] = max(grados);
            ledCircle.FaceColor = colores{idx};
            ledText.String      = iconos{idx};
            labelTitulo.Text    = sprintf('%s Estado Actual (Humedad)', iconos{idx});

            % Trazado de MFs y marcado de punto actual
            for i = 1:3
                plot(ax, x, mfs{i}, 'Color', colores{i}, 'LineWidth', 2);
                scatter(ax, humedad, grados(i), 60, colores{i}, 'filled', 'HandleVisibility', 'off');
            end
            % Colocación de línea vertical en valor actual
            xline(ax, humedad, '--k', 'LineWidth', 1, 'Label', sprintf('P=%.1f%%', humedad), ...
                'LabelOrientation', 'horizontal', 'HandleVisibility','off');
            legend(ax, {'Seco','Medio','Humedo'}, 'Location','northeast');

        else
            %----- TEMPERATURA (variable activa) -----
            % Construcción de MFs y cálculo de grados de pertenencia para temperatura actual
            mfs    = {trimf(x,[0 0 50]), trimf(x,[25 50 75]), trimf(x,[50 100 100])};
            grados = [trimf(temp,[0 0 50]), trimf(temp,[25 50 75]), trimf(temp,[50 100 100])];

            % Actualización de etiquetas informativas
            labelTemp.Text        = sprintf('Temp: %.1f °C', temp);
            labelFrio.Text        = sprintf('Frio: %.2f',      grados(1));
            labelTemplado.Text    = sprintf('Templado: %.2f',  grados(2));
            labelCaliente.Text    = sprintf('Caliente: %.2f',  grados(3));

            % Selección de color e icono del LED según MF dominante
            colores = {[0.4 0.6 0.8], [0.5 0.75 0.5], [0.9 0.5 0.5]};
            iconos  = {'🧊','🌡️','🔥'};
            [~, idx] = max(grados);
            ledCircle.FaceColor = colores{idx};
            ledText.String      = iconos{idx};
            labelTitulo.Text    = sprintf('%s Estado Actual (Temperatura)', iconos{idx});

            % Trazado de MFs y marcado de punto actual
            for i = 1:3
                plot(ax, x, mfs{i}, 'Color', colores{i}, 'LineWidth', 2);
                scatter(ax, temp, grados(i), 60, colores{i}, 'filled', 'HandleVisibility', 'off');
            end
            % Colocación de línea vertical en valor actual
            xline(ax, temp, '--k', 'LineWidth', 1, 'Label', sprintf('P=%.1f°C', temp), ...
                'LabelOrientation', 'horizontal', 'HandleVisibility','off');
            legend(ax, {'Frio','Templado','Caliente'}, 'Location','northeast');
        end

        % Curva de salida agregada (referencia visual de la forma resultante)
        [xOut, yOut] = plotmf(fis, 'output', 1, 1000);
        plot(ax, xOut, yOut, ':k', 'HandleVisibility','off');

        % Marcación de línea vertical en la salida actual del FIS
        xline(ax, salida, '--r', 'LineWidth', 2, 'Label', sprintf('Salida = %.2f', salida), ...
            'LabelOrientation', 'horizontal', 'LabelVerticalAlignment', 'bottom', ...
            'HandleVisibility','off');

        % Ajustes finales de ejes y rejilla
        ylim(ax, [0 1.1]); xlim(ax, [0 100]);
        title(ax, 'Funciones de membresía - Variable activa');
        grid(ax, 'on'); hold(ax, 'off');
    end

    %======================
    % Superficie 3D + export EPS
    %======================
    function mostrarSuperficie3D(fisLocal)
        % Preparación de malla (T,H) y evaluación del FIS para construcción de superficie
        tRange = [0 100];    % Rango de temperatura (°C)
        hRange = [0 100];    % Rango de humedad (%)
        step   = 5;          % Resolución de muestreo

        [T, H] = meshgrid(tRange(1):step:tRange(2), hRange(1):step:hRange(2));
        Z = evalfis(fisLocal, [T(:) H(:)]);
        Z = reshape(Z, size(T));

        % Definición de puntos de ejemplo (coincidentes con tabla ilustrativa)
        P = [ 20  20
              30  30
              25  80
              35  70
              45  30
              50  80 ];
        V = evalfis(fisLocal, P);

        % Construcción de figura 3D y ejes
        fig = figure('Name', 'Superficie Difusa 3D', 'NumberTitle', 'off', 'Color', 'w');
        ax  = axes(fig); hold(ax, 'on');

        % Renderizado de superficie con malla
        s = surf(ax, T, H, Z, 'EdgeColor', [0.25 0.25 0.25], 'FaceAlpha', 1.0);
        % (Opcional) Suavizado de superficie:
        % shading interp; set(s,'EdgeColor','none');

        % Configuración de colormap y ejes
        colormap(ax, turbo); colorbar(ax);
        xlabel(ax, 'Temperatura (°C)', 'FontWeight','bold');
        ylabel(ax, 'Humedad (%)',      'FontWeight','bold');
        zlabel(ax, 'Velocidad del Ventilador', 'FontWeight','bold');
        title(ax, 'Superficie del Sistema Difuso');

        % Normalización de rangos y vista
        zlim(ax, [0 10]);
        caxis(ax, [0 10]);
        view(ax, -40, 25);
        grid(ax, 'on'); box(ax, 'on');

        % Marcación de puntos evaluados y rotulación
        scatter3(ax, P(:,1), P(:,2), V, 64, 'k', 'filled', 'MarkerEdgeColor','w');
        for k = 1:size(P,1)
            text(P(k,1), P(k,2), V(k)+0.25, sprintf('%d',k), ...
                'Color','k','FontWeight','bold', 'HorizontalAlignment','center');
        end
        legend(ax, {'Superficie','Casos de ejemplo'}, 'Location','northeast');

        % Exportación vectorial EPS (compatibilidad con LaTeX)
        try
            set(fig, 'Renderer', 'painters');
            print(fig, 'resultados_superficie_puntos.eps', '-depsc', '-r300');
            disp('✅ Figura exportada como resultados_superficie_puntos.eps');
        catch ME
            warning('No se pudo exportar la figura: %s', ME.message);
        end
    end
end
