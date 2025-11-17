function Controlador_Mamdani
%=========================================================
% GUI + FIS Mamdani (ventilador) — versión limpia
% - Sin líneas negras punteadas extra en la gráfica
% - Llamada a actualizarVisual() después de crear los knobs
%=========================================================

%% ---------- FIS Mamdani ----------
fis = mamfis('Name','ControlVentilador');

% Entrada 1: Temperatura (°C)
fis = addInput(fis,[0 100],'Name','Temperatura');
fis = addMF(fis,'Temperatura','trimf',[0 0 50],   'Name','Frio');
fis = addMF(fis,'Temperatura','trimf',[25 50 75], 'Name','Templado');
fis = addMF(fis,'Temperatura','trimf',[50 100 100],'Name','Caliente');

% Entrada 2: Humedad (%)
fis = addInput(fis,[0 100],'Name','Humedad');
fis = addMF(fis,'Humedad','trimf',[0 0 50],    'Name','Seco');
fis = addMF(fis,'Humedad','trimf',[25 50 75],  'Name','Medio');
fis = addMF(fis,'Humedad','trimf',[50 100 100],'Name','Humedo');

% Salida: Velocidad (0–10)
fis = addOutput(fis,[0 10],'Name','Velocidad');
fis = addMF(fis,'Velocidad','trimf',[0 0 5],     'Name','Lento');
fis = addMF(fis,'Velocidad','trimf',[2.5 5 7.5], 'Name','Medio');
fis = addMF(fis,'Velocidad','trimf',[7 10 10],   'Name','Rapido');

% Reglas (conector=1 AND, peso=1)
ruleList = [
    1 1 1 1 1; 1 2 2 1 1; 1 3 2 1 1;
    2 1 2 1 1; 2 2 2 1 1; 2 3 3 1 1;
    3 1 2 1 1; 3 2 3 1 1; 3 3 3 1 1];
fis = addRule(fis,ruleList);

%% ---------- GUI ----------
fig = uifigure('Name','Sistema Difuso Mamdani','Position',[100 100 950 520]);

% Estado compartido
shared.temp = 0; shared.hum = 0;
shared.sentidoTemp = 1; shared.sentidoHum = 1;
shared.lastChanged  = 'temp';

% Panel principal (gráfica)
panel = uipanel(fig,'Position',[25 130 550 260],'Title','Visualización Difusa');
ax    = axes(panel,'Position',[0.1 0.15 0.65 0.75]);

% LED de estado
ledAxes = uiaxes(panel,'Position',[390 80 80 80]);
axis(ledAxes,[0 1 0 1]); axis(ledAxes,'off'); axis(ledAxes,'equal');
ledCircle = rectangle(ledAxes,'Position',[0 0 1 1],'Curvature',[1 1], ...
    'FaceColor',[0.8 0.8 0.8],'EdgeColor',[0.4 0.4 0.4],'LineWidth',2);
ledText = text(ledAxes,0.5,0.5,'','HorizontalAlignment','center', ...
    'VerticalAlignment','middle','FontWeight','bold','FontSize',10,'Color','w');

% Gauge salida
% G A U G E  (un poco más arriba)
medidor = uigauge(fig,'semicircular', ...
    'Position',[615 380 140 100]);   % antes estaba más abajo
medidor.Limits = [0 10]; medidor.MajorTicks = 0:2:10;
medidor.ScaleColors = {[0 0.6 1],[0.2 0.8 0.2],[1 0.4 0.4]};
medidor.ScaleColorLimits = [0 4; 4 7; 7 10];
salidaLabel = uilabel(fig,'Position',[710 358 150 22], ...
    'Text','Velocidad: 0.00','FontSize',11,'FontWeight','bold');

% Tabla mini (ilustrativa)
% Panel para las reglas (mismo lugar que antes, pero scrollable)
% T A B L A  D E  R E G L A S  (más abajo y con scroll)
panelTabla = uipanel(fig, ...
    'Position',[615 260 190 80], ... % ↓ y=260, altura=80
    'BorderType','none', ...
    'Scrollable','on');

tablaReglas = uitable(panelTabla, ...
    'Position',[0 0 155 220], ...
    'ColumnName',{'Regla','Grado'}, ...
    'ColumnWidth',{140,100}, ...
    'FontSize',11);


% Panel info actual
infoPanel = uipanel(fig,'Title','', 'Position',[616 110 160 140], ...
    'BackgroundColor',[0.94 0.97 1],'BorderType','line');
labelTitulo = uilabel(infoPanel,'Text','🧊 Estado Actual', ...
    'FontWeight','bold','FontSize',12,'Position',[10 105 140 20], ...
    'HorizontalAlignment','left','BackgroundColor',[0.94 0.97 1]);
labelTemp = uilabel(infoPanel,'Position',[10 80 140 22],'Text','Temp: 0.0 °C', ...
    'FontSize',14,'FontWeight','bold','FontColor',[0.1 0.2 0.5], ...
    'BackgroundColor',[0.94 0.97 1]);
labelFrio     = uilabel(infoPanel,'Position',[10 55 130 18],'Text','Frio: 0.00',     'BackgroundColor',[0.94 0.97 1]);
labelTemplado = uilabel(infoPanel,'Position',[10 35 130 18],'Text','Templado: 0.00','BackgroundColor',[0.94 0.97 1]);
labelCaliente = uilabel(infoPanel,'Position',[10 15 130 18],'Text','Caliente: 0.00','BackgroundColor',[0.94 0.97 1]);

% Knobs (¡se crean ANTES de llamar a actualizarVisual!)
uilabel(fig,'Position',[680 85 60 20],'Text','Temp.');
knobTemp = uiknob(fig,'continuous','Position',[680 50 35 35], ...
    'Limits',[0 100],'MajorTicks',0:25:100);

uilabel(fig,'Position',[750 85 60 20],'Text','Humedad');
knobHum  = uiknob(fig,'continuous','Position',[750 50 35 35], ...
    'Limits',[0 100],'MajorTicks',0:25:100);

% Botones
toggle = uibutton(fig,'state','Text','▶ Simulación','Position',[750 140 150 30], ...
    'ValueChangedFcn',@(btn,~) toggleSimulacion(btn));
botonSuperficie = uibutton(fig,'Text','Ver superficie 3D', ...
    'Position',[750 180 150 30],'ButtonPushedFcn',@(~,~) mostrarSuperficie3D(fis));

% Timer de simulación
simTimer = timer('ExecutionMode','fixedRate','Period',0.3,'TimerFcn',@(~,~) simStep());

% Callbacks de knobs
knobTemp.ValueChangedFcn = @onTempChanged;
knobHum.ValueChangedFcn  = @onHumChanged;

% -------- Llamada inicial (ya existen los knobs) --------
actualizarVisual();

%% ---------- Funciones internas ----------
    function onTempChanged(~,~)
        shared.lastChanged = 'temp';
        actualizarVisual();
    end

    function onHumChanged(~,~)
        shared.lastChanged = 'hum';
        actualizarVisual();
    end

    function toggleSimulacion(btn)
        if btn.Value, start(simTimer); btn.Text = '⏸ Pausar';
        else,         stop(simTimer);  btn.Text = '▶ Simulación';
        end
    end

    function simStep()
        paso = 5;
        shared.temp = shared.temp + paso*shared.sentidoTemp;
        if shared.temp>=100 || shared.temp<=0
            shared.sentidoTemp = -shared.sentidoTemp;
            shared.temp = max(min(shared.temp,100),0);
        end
        shared.hum  = shared.hum  + paso*shared.sentidoHum;
        if shared.hum>=100 || shared.hum<=0
            shared.sentidoHum = -shared.sentidoHum;
            shared.hum = max(min(shared.hum,100),0);
        end
        knobTemp.Value = shared.temp;
        knobHum.Value  = shared.hum;
        actualizarVisual();
    end

    function actualizarVisual()
        % Valores actuales
        temp    = knobTemp.Value;
        humedad = knobHum.Value;
        salida  = evalfis(fis,[temp,humedad]);

        % Gauge y etiqueta
        medidor.Value    = salida;
        salidaLabel.Text = sprintf('Velocidad: %.2f',salida);

        % Tabla (ejemplos)
        tablaReglas.Data = {
    'Frio y Seco   -- Lento',        evalfis(fis,[0   0  ]);
    'Frio y Medio  -- Medio',        evalfis(fis,[0   50 ]);
    'Frio y Humedo -- Medio',        evalfis(fis,[0   100]);
    'Templado y Seco   -- Medio',    evalfis(fis,[50  0  ]);
    'Templado y Medio  -- Medio',    evalfis(fis,[50  50 ]);
    'Templado y Humedo -- Rapido',   evalfis(fis,[50 100]);
    'Caliente y Seco   -- Medio',    evalfis(fis,[100 0  ]);
    'Caliente y Medio  -- Rapido',   evalfis(fis,[100 50]);
    'Caliente y Humedo -- Rapido',   evalfis(fis,[100 100]);
};

        % Gráfica MFs (solo variable activa)
        x = linspace(0,100,500);
        cla(ax,'reset'); hold(ax,'on');
        ax.XMinorGrid = 'off'; ax.YMinorGrid = 'off';  % <- evita rejillas punteadas
        grid(ax,'on');

        if strcmp(shared.lastChanged,'hum')
            % HUMEDAD activa
            mfs    = {trimf(x,[0 0 50]), trimf(x,[25 50 75]), trimf(x,[50 100 100])};
            grados = [trimf(humedad,[0 0 50]), trimf(humedad,[25 50 75]), trimf(humedad,[50 100 100])];

            labelTemp.Text     = sprintf('Hum: %.1f %%',humedad);
            labelFrio.Text     = sprintf('Seco: %.2f',grados(1));
            labelTemplado.Text = sprintf('Medio: %.2f',grados(2));
            labelCaliente.Text = sprintf('Humedo: %.2f',grados(3));

            colores = {[0.6 0.8 1.0],[0.5 0.75 0.5],[0.4 0.6 0.9]};
            iconos  = {'💧','💧💧','💧💧💧'};
            [~,idx] = max(grados);
            ledCircle.FaceColor = colores{idx}; ledText.String = iconos{idx};
            labelTitulo.Text = sprintf('%s Estado Actual (Humedad)',iconos{idx});

            for i=1:3
                plot(ax,x,mfs{i},'LineWidth',2,'Color',colores{i});
                scatter(ax,humedad,grados(i),60,colores{i},'filled','HandleVisibility','off');
            end
            % ÚNICA línea negra discontinua (valor actual)
            xline(ax,humedad,'--k','LineWidth',1,'Label',sprintf('P=%.1f%%',humedad), ...
                'LabelOrientation','horizontal','HandleVisibility','off');

        else
            % TEMPERATURA activa
            mfs    = {trimf(x,[0 0 50]), trimf(x,[25 50 75]), trimf(x,[50 100 100])};
            grados = [trimf(temp,[0 0 50]), trimf(temp,[25 50 75]), trimf(temp,[50 100 100])];

            labelTemp.Text     = sprintf('Temp: %.1f °C',temp);
            labelFrio.Text     = sprintf('Frio: %.2f',grados(1));
            labelTemplado.Text = sprintf('Templado: %.2f',grados(2));
            labelCaliente.Text = sprintf('Caliente: %.2f',grados(3));

            colores = {[0.4 0.6 0.8],[0.5 0.75 0.5],[0.9 0.5 0.5]};
            iconos  = {'🧊','🌡️','🔥'};
            [~,idx] = max(grados);
            ledCircle.FaceColor = colores{idx}; ledText.String = iconos{idx};
            labelTitulo.Text = sprintf('%s Estado Actual (Temperatura)',iconos{idx});

            for i=1:3
                plot(ax,x,mfs{i},'LineWidth',2,'Color',colores{i});
                scatter(ax,temp,grados(i),60,colores{i},'filled','HandleVisibility','off');
            end
            % ÚNICA línea negra discontinua (valor actual)
            xline(ax,temp,'--k','LineWidth',1,'Label',sprintf('P=%.1f°C',temp), ...
                'LabelOrientation','horizontal','HandleVisibility','off');
        end

        % *** Importante para evitar “líneas negras punteadas” a la izquierda:
        %    NO dibujamos la curva de salida agregada (plotmf de la salida)
        %    y NO añadimos líneas verticales adicionales.
        %    Solo mantenemos la línea roja de la salida y la negra del punto actual.

        % Línea roja (salida) – se dibuja como referencia, sin otras marcas
        xline(ax,salida,'--r','LineWidth',2,'Label',sprintf('Salida = %.2f',salida), ...
            'LabelOrientation','horizontal','LabelVerticalAlignment','bottom', ...
            'HandleVisibility','off');

        % Acabados
        ylim(ax,[0 1.1]); xlim(ax,[0 100]);
        title(ax,'Ventilador con funciones de membresía');
        legend(ax,{'Frio','Templado','Caliente'},'Location','northeast','FontSize',12);
        hold(ax,'off');
    end

    function mostrarSuperficie3D(fisLocal)
        tRange = [0 100]; hRange = [0 100]; step = 5;
        [T,H] = meshgrid(tRange(1):step:tRange(2), hRange(1):step:hRange(2));
        Z = evalfis(fisLocal,[T(:) H(:)]); Z = reshape(Z,size(T));

        f = figure('Name','Superficie Difusa 3D','NumberTitle','off','Color','w');
        ax3 = axes(f); hold(ax3,'on');
        surf(ax3,T,H,Z,'EdgeColor',[0.25 0.25 0.25],'FaceAlpha',1.0);
        colormap(ax3,turbo); colorbar(ax3);
        xlabel(ax3,'Temperatura (°C)','FontWeight','bold');
        ylabel(ax3,'Humedad (%)','FontWeight','bold');
        zlabel(ax3,'Velocidad del Ventilador','FontWeight','bold');
        title(ax3,'Superficie del Sistema Difuso');
        zlim(ax3,[0 10]); caxis(ax3,[0 10]); view(ax3,-40,25); grid(ax3,'on'); box(ax3,'on');
    end
end
