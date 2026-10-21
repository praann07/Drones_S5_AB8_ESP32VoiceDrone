function DroneFlightLab_2026
% DRONE FLIGHT LAB 2026
% Fresh Windows MATLAB package: visible quadcopter, dynamic outdoor scene,
% command-driven flight, city obstacles, hard-stop safety, and telemetry analysis.
%
% Run: DroneFlightLab_2026
% Voice bridge: listen_voice.ps1 must be beside this file.
% Software simulation only; it does not control real ESP32 hardware.

clc; close all; rng(7);

%% Simulation parameters
dt = 0.02;                 % 50 Hz update
g = 9.81; mass = 1.0;
maxTilt = deg2rad(22);
maxSpeed = 3.2;              % conservative outdoor multirotor speed
maxAlt = 25;
geofenceRadius = 75;
windLevel = 1;             % 1 light, 2 strong, 3 storm
gustEnd = -1;
t = 0; running = true; paused = false;
armed = false; landing = false; missionOn = false; orbitOn = false; randomFlightOn = false; cruiseOn = false; resumeCruiseAfterTurn = false; cruiseDirection = 1;
mode = "DISARMED";
commandText = "Ready. Use TAKE OFF, TURN LEFT, TURN RIGHT, or RETURN TO HOME.";
lastCommand = "NONE"; lastVoice = "Waiting for microphone";
battery = 100; alarmText = "OK";
% Position order is [East; North; Up] in metres.
pos = [0;0;0]; vel = [0;0;0];
att = [0;0;0];               % roll, pitch, yaw
attRate = [0;0;0];
accState = [0;0;0];             % filtered acceleration for actuator/airframe lag
home = [0;0;0]; target = [0;0;0];
yawTarget = 0; missionIdx = 2; orbitPhase = 0;
orbitCenter = pos; orbitRadius = 7; % Initialise in parent workspace for nested functions
posInt = [0;0]; altInt = 0;
% Waypoints in [East North Up] coordinates; last waypoint returns home.
W = [0 0 0; 18 20 7; 40 -4 8; 5 -26 8; -25 15 7; 0 0 6];
Wname = ["HOME","A","B","C","D","HOME"];

% Fixed obstacles (East, North), physical radius and height.
obstacleXY = [0 8; 9 0; -9 0; 0 -9; 15 8];
obstacleR = [0.34; 0.34; 0.34; 0.34; 0.42];
obstacleH = [2.2; 2.2; 2.2; 2.2; 2.8];
obstacleNames = ["NORTH BARRIER","EAST BARRIER","WEST BARRIER", ...
                 "SOUTH BARRIER","NORTHEAST BARRIER"];
obstacleSafety = 1.65; % covers rotor-to-rotor footprint plus a conservative stopping margin
wind = [1.2;0.8;0]; motors = zeros(4,1);
logT=[]; logPos=[]; logVel=[]; logAtt=[]; logTarget=[]; logMot=[]; logWind=[]; logBat=[];
lastTelemetry = -1;

%% Figure and scene
fig = figure('Name','DRONE FLIGHT LAB 2026 | Command-Driven City Flight', ...
    'NumberTitle','off','Color',[0.055 0.07 0.095], ...
    'Position',[35 35 1500 900],'MenuBar','none','ToolBar','none', ...
    'CloseRequestFcn',@closeApp);

ax = axes('Parent',fig,'Position',[0.025 0.18 0.655 0.79], ...
    'Color',[0.68 0.82 0.91],'XColor',[0.18 0.23 0.27], ...
    'YColor',[0.18 0.23 0.27],'ZColor',[0.18 0.23 0.27], ...
    'LineWidth',0.8);
hold(ax,'on'); grid(ax,'off'); axis(ax,'equal'); axis(ax,'vis3d');
xlabel(ax,'East (m)'); ylabel(ax,'North (m)'); zlabel(ax,'Altitude (m)');
title(ax,'URBAN CITY DRONE SIMULATION — LIVE 3D WORLD','FontSize',14, ...
    'FontWeight','bold','Color',[0.10 0.16 0.20]);
view(ax,48,24); camproj(ax,'perspective');
xlim(ax,[-38 38]); ylim(ax,[-38 38]); zlim(ax,[0 36]);
set(ax,'Projection','perspective','SortMethod','depth','Box','off');
set(ax,'Clipping','on');
lighting(ax,'gouraud'); material(ax,'dull');

% Continuous urban ground plane and city-block street network.
groundV=[-100 -100 0.02; 100 -100 0.02; 100 100 0.02; -100 100 0.02];
patch(ax,'Vertices',groundV,'Faces',[1 2 3 4], ...
    'FaceColor',[0.34 0.36 0.37],'EdgeColor','none');
[gx,gy]=meshgrid(-90:3:90,-90:3:90);
gz=0.13 + 0.008*sin(gx/13).*cos(gy/17);
surf(ax,gx,gy,gz,'FaceColor',[0.34 0.36 0.37], ...
    'EdgeColor','none','FaceAlpha',1,'FaceLighting','gouraud');

% Urban street grid: asphalt, lane markings, sidewalks and zebra crossings.
roadZ=0.17;
roadCenters=-30:15:30;
roadHalf=2.7;
for rc=roadCenters
    patch(ax,[rc-roadHalf rc+roadHalf rc+roadHalf rc-roadHalf],[-100 -100 100 100], ...
        roadZ*ones(1,4),[0.16 0.18 0.20],'EdgeColor','none');
    patch(ax,[-100 100 100 -100],[rc-roadHalf rc-roadHalf rc+roadHalf rc+roadHalf], ...
        roadZ*ones(1,4),[0.16 0.18 0.20],'EdgeColor','none');
    % Broken centre lines.
    for q=-96:7:96
        plot3(ax,[rc rc],[q q+3.2],[roadZ+0.025 roadZ+0.025], ...
            'Color',[0.95 0.82 0.38],'LineWidth',1.1);
        plot3(ax,[q q+3.2],[rc rc],[roadZ+0.025 roadZ+0.025], ...
            'Color',[0.95 0.82 0.38],'LineWidth',1.1);
    end
    % Pale sidewalk strips beside each street.
    for side=[-1 1]
        sx=rc+side*(roadHalf+0.42);
        patch(ax,[sx-0.28 sx+0.28 sx+0.28 sx-0.28],[-100 -100 100 100], ...
            (roadZ+0.008)*ones(1,4),[0.55 0.56 0.54],'EdgeColor','none');
        sy=rc+side*(roadHalf+0.42);
        patch(ax,[-100 100 100 -100],[sy-0.28 sy-0.28 sy+0.28 sy+0.28], ...
            (roadZ+0.009)*ones(1,4),[0.55 0.56 0.54],'EdgeColor','none');
    end
end
% Crosswalk stripes at selected intersections.
for ix=[-15 0 15]
    for iy=[-15 0 15]
        for stripe=-2:0.8:2
            patch(ax,[ix-3.1 ix-2.65 ix-2.65 ix-3.1], ...
                [iy+stripe iy+stripe iy+stripe+0.38 iy+stripe+0.38], ...
                (roadZ+0.035)*ones(1,4),[0.83 0.84 0.82],'EdgeColor','none');
        end
    end
end
% Home pad and markings
[Xp,Yp]=meshgrid([-3 3],[-3 3]);
surf(ax,Xp,Yp,0.19*ones(2),'FaceColor',[0.13 0.16 0.19], ...
    'EdgeColor',[0.95 0.96 0.98],'LineWidth',1.5);
aa=linspace(0,2*pi,100);
for rr=[1.0 2.0]
    plot3(ax,rr*cos(aa),rr*sin(aa),0.22*ones(size(aa)), ...
        'Color',[0.9 0.92 0.94],'LineWidth',1.3);
end
plot3(ax,[-0.8 0.8],[0 0],[0.24 0.24],'w-','LineWidth',2.5);
plot3(ax,[0 0],[-0.8 0.8],[0.24 0.24],'w-','LineWidth',2.5);

% Secondary landing/inspection pad away from HOME.
patch(ax,[12 17 17 12],[-17 -17 -12 -12],[0.20 0.20 0.20 0.20], ...
    [0.27 0.29 0.30],'EdgeColor',[0.90 0.92 0.90],'LineWidth',1.1);
plot3(ax,[12.8 16.2],[-14.5 -14.5],[0.23 0.23], ...
    'Color',[0.92 0.92 0.86],'LineWidth',1.4);
plot3(ax,[14.5 14.5],[-16.2 -12.8],[0.23 0.23], ...
    'Color',[0.92 0.92 0.86],'LineWidth',1.4);

% Dense city blocks. Buildings occupy the blocks between the street grid;
% heights, footprints and facade colours vary to avoid a repetitive toy scene.
Bld=zeros(0,5);
cityCenters=[-22.5 -7.5 7.5 22.5];
for bi=1:numel(cityCenters)
    for bj=1:numel(cityCenters)
        en=cityCenters(bi); no=cityCenters(bj);
        % Keep the central home plaza clear for take-off and landing.
        if abs(en)<10 && abs(no)<10, continue; end
        ww=5.2+mod(2*bi+bj,3)*0.65;
        dd=5.1+mod(bi+2*bj,3)*0.60;
        hh=8.5+mod(3*bi+5*bj,7)*2.25;
        Bld(end+1,:)=[en no ww dd hh]; %#ok<AGROW>
    end
end
% Add a few taller towers near the outer edge of the visible city.
Bld=[Bld; -31 -21 5.8 6.2 25; 31 21 6.2 5.8 29; ...
           -31 21 6.0 5.6 21; 31 -21 5.6 6.0 24; ...
            0 31 6.0 5.5 18; 0 -31 5.5 6.0 20];
for k=1:size(Bld,1)
    en=Bld(k,1); no=Bld(k,2); ww=Bld(k,3); dd=Bld(k,4); hh=Bld(k,5);
    palette=[0.46 0.50 0.55; 0.55 0.54 0.50; 0.42 0.47 0.51; ...
             0.60 0.58 0.53; 0.38 0.44 0.49; 0.52 0.56 0.58];
    facade=palette(mod(k-1,size(palette,1))+1,:);
    drawBox(ax,en-ww/2,en+ww/2,no-dd/2,no+dd/2,0,hh,facade);
    % Repeated windows on two facades, scaled to each building height.
    for zwin=1.8:2.1:hh-0.8
        for xx=en-ww/2+0.75:1.35:en+ww/2-0.55
            plot3(ax,[xx xx],[no-dd/2-0.035 no-dd/2-0.035],[zwin zwin], ...
                'Color',[0.96 0.80 0.42],'LineWidth',1.25);
        end
        for yy=no-dd/2+0.7:1.35:no+dd/2-0.5
            plot3(ax,[en+ww/2+0.035 en+ww/2+0.035],[yy yy],[zwin zwin], ...
                'Color',[0.77 0.85 0.90],'LineWidth',1.15);
        end
    end
    % Small rooftop service unit / HVAC box adds scale to the skyline.
    drawBox(ax,en-0.7,en+0.7,no-0.55,no+0.55,hh,hh+0.65,[0.30 0.33 0.36]);
end

% Trees, excluding nearby waypoint corridors.
treeXY=[-14 -10;-14 5;-12 14;13 -12;14 3;13 14; ...
         -7 16;7 -16;-22 -8;22 6;-26 18;27 -18; ...
         31 23;-31 -22;18 -31;-5 -38;45 -12;-46 16];
for k=1:size(treeXY,1)
    ex=treeXY(k,1); ny=treeXY(k,2);
    if min(vecnorm(W(:,1:2)-[ex ny],2,2))<7, continue; end
    if min(vecnorm(obstacleXY-[ex ny],2,2))<4.5, continue; end
    if min(vecnorm(Bld(:,1:2)-[ex ny],2,2))<5.5, continue; end
    [cx,cy,cz]=cylinder([0.16 0.12],12); cz=cz*1.8;
    surf(ax,ex+cx,ny+cy,cz+0.12,'FaceColor',[0.34 0.20 0.10],'EdgeColor','none');
    % Layered rounded canopy, with subtle colour variation.
    [sxTree,syTree,szTree]=sphere(12);
    surf(ax,ex+0.95*sxTree,ny+0.95*syTree,2.0+1.05*szTree, ...
        'FaceColor',[0.10 0.32 0.13],'EdgeColor','none','FaceLighting','gouraud');
    surf(ax,ex+0.68*sxTree,ny+0.68*syTree,2.65+0.78*szTree, ...
        'FaceColor',[0.15 0.39 0.16],'EdgeColor','none','FaceLighting','gouraud');
end

% Visible red/orange obstacle cylinders and dashed safety footprints.
[obsX,obsY,obsZ] = cylinder(1,24);
for oi = 1:numel(obstacleR)
    ox = obstacleXY(oi,1); oy = obstacleXY(oi,2);
    surf(ax,ox+obstacleR(oi)*obsX,oy+obstacleR(oi)*obsY, ...
        0.14+obstacleH(oi)*obsZ, ...
        'FaceColor',[0.82 0.17 0.10],'EdgeColor',[0.34 0.06 0.04], ...
        'FaceAlpha',0.94,'LineWidth',0.8);
    for bandZ = [0.75 1.45 2.05]
        if bandZ < obstacleH(oi)
            aaBand = linspace(0,2*pi,40);
            plot3(ax,ox+(obstacleR(oi)+0.025)*cos(aaBand), ...
                oy+(obstacleR(oi)+0.025)*sin(aaBand), ...
                (0.14+bandZ)*ones(size(aaBand)), ...
                'Color',[1.0 0.78 0.12],'LineWidth',2.0);
        end
    end
    safetyR = obstacleR(oi)+obstacleSafety;
    plot3(ax,ox+safetyR*cos(aa),oy+safetyR*sin(aa), ...
        0.19*ones(size(aa)),'Color',[0.95 0.28 0.18],'LineStyle','--','LineWidth',0.9);
    text(ax,ox+0.45,oy+0.45,obstacleH(oi)+0.25,char(obstacleNames(oi)), ...
        'Color',[0.45 0.04 0.03],'FontWeight','bold','FontSize',6, ...
        'BackgroundColor',[1.0 0.83 0.62],'Margin',1);
end

% Command-driven flight: no displayed or pre-programmed waypoint route.
% The home pad remains the sole navigation reference for RETURN HOME.
plot3(ax,geofenceRadius*cos(aa),geofenceRadius*sin(aa),0.03*ones(size(aa)), ...
    'r--','LineWidth',1.3);
trail=animatedline(ax,'Color',[1 0.18 0.12],'LineWidth',2.1);
targetMark=plot3(ax,0,0,0,'o','MarkerSize',7,'LineWidth',1.8,'MarkerFaceColor',[0.1 1 0.5], ...
    'MarkerEdgeColor',[0.02 0.15 0.07]);
windArrow=quiver3(ax,0,0,0,0,0,0,'Color',[0.08 0.18 0.95],'LineWidth',2,'MaxHeadSize',1.3);
shadow=plot3(ax,nan,nan,nan,'k:','LineWidth',1.4);

% Recognisable quadcopter visual: central fuselage, thick X arms, motor pods,
% spinning two-blade propellers, camera, LED lights and landing skids.
VIS=1.0; armLen=1.65; bodyScale=1.0;
arm1=plot3(ax,nan,nan,nan,'Color',[0.08 0.09 0.11],'LineWidth',8);
arm2=plot3(ax,nan,nan,nan,'Color',[0.08 0.09 0.11],'LineWidth',8);
armAccent1=plot3(ax,nan,nan,nan,'Color',[0.05 0.48 0.88],'LineWidth',3);
armAccent2=plot3(ax,nan,nan,nan,'Color',[0.05 0.48 0.88],'LineWidth',3);
nose=plot3(ax,nan,nan,nan,'Color',[1 0.13 0.08],'LineWidth',4);
camera=plot3(ax,nan,nan,nan,'o','MarkerSize',9,'MarkerFaceColor',[0.05 0.65 1], ...
    'MarkerEdgeColor','k');
leg1=plot3(ax,nan,nan,nan,'Color',[0.08 0.08 0.08],'LineWidth',4);
leg2=plot3(ax,nan,nan,nan,'Color',[0.08 0.08 0.08],'LineWidth',4);
[spx,spy,spz]=sphere(14);
body=surf(ax,nan(size(spx)),nan(size(spy)),nan(size(spz)), ...
    'FaceColor',[0.10 0.12 0.15],'EdgeColor',[0.02 0.02 0.02]);
pods=gobjects(4,1); blades=gobjects(8,1); rotorRings=gobjects(4,1);
propBlur=gobjects(4,1); leds=gobjects(4,1);
for k=1:4
    pods(k)=surf(ax,nan(size(spx)),nan(size(spy)),nan(size(spz)), ...
        'FaceColor',[0.13 0.14 0.16],'EdgeColor',[0.02 0.02 0.02]);
    propBlur(k)=surf(ax,nan(18),nan(18),nan(18), ...
        'FaceColor',[0.10 0.12 0.13],'FaceAlpha',0.12,'EdgeColor','none');
    rotorRings(k)=plot3(ax,nan,nan,nan,'Color',[0.2 0.23 0.26],'LineWidth',0.9);
    blades(2*k-1)=plot3(ax,nan,nan,nan,'Color',[0.025 0.03 0.035],'LineWidth',3);
    blades(2*k)=plot3(ax,nan,nan,nan,'Color',[0.025 0.03 0.035],'LineWidth',3);
    leds(k)=plot3(ax,nan,nan,nan,'o','MarkerSize',5,'MarkerFaceColor',[0.2 1 0.35],'MarkerEdgeColor','k');
end
% Moving clouds
clouds=gobjects(4,1); cloudSeed=[-12 8 16; -4 13 18; 10 11 17; 12 -9 19];
for k=1:4
    clouds(k)=surf(ax,nan(8),nan(8),nan(8),'FaceColor',[0.97 0.98 1], ...
        'EdgeColor','none','FaceAlpha',0.55);
end
camlight(ax,35,35); camlight(ax,-120,20); lighting(ax,'gouraud');
material(ax,'dull');

%% Dashboard
panel=uipanel('Parent',fig,'Units','normalized','Position',[0.69 0.18 0.285 0.78], ...
    'BackgroundColor',[0.08 0.11 0.15],'ForegroundColor',[0.94 0.97 1], ...
    'Title','FLIGHT CONTROL / TELEMETRY','FontWeight','bold','FontSize',11);
hud=uicontrol(panel,'Style','text','Units','normalized','Position',[0.04 0.69 0.92 0.28], ...
    'BackgroundColor',[0.08 0.11 0.15],'ForegroundColor',[0.86 0.94 1], ...
    'HorizontalAlignment','left','FontName','Consolas','FontSize',11,'String','Starting...');
voiceHud=uicontrol(panel,'Style','text','Units','normalized','Position',[0.04 0.61 0.92 0.07], ...
    'BackgroundColor',[0.12 0.18 0.23],'ForegroundColor',[0.5 1 0.72], ...
    'HorizontalAlignment','left','FontName','Consolas','FontSize',10,'String','VOICE: Ready');
uicontrol(panel,'Style','text','Units','normalized','Position',[0.04 0.535 0.92 0.065], ...
    'BackgroundColor',[0.08 0.11 0.15],'ForegroundColor',[0.85 0.9 0.98], ...
    'HorizontalAlignment','left','FontSize',9,'String', ...
    'Commands: TAKE OFF | TURN LEFT/RIGHT | BACKWARD | HOVER | LAND | RETURN TO HOME');
mkbtn('LISTEN TO VOICE',[.04 .45 .92 .065],@listenVoice,[.10 .32 .62]);
mkbtn('TAKE OFF',[.04 .355 .44 .065],@(~,~)command("take off"),[.10 .40 .25]);
mkbtn('TURN LEFT',[.52 .355 .44 .065],@(~,~)command("turn left"),[.12 .29 .42]);
mkbtn('TURN RIGHT',[.04 .275 .44 .065],@(~,~)command("turn right"),[.12 .29 .42]);
mkbtn('BACKWARD',[.52 .275 .44 .065],@(~,~)command("backward"),[.12 .29 .42]);
mkbtn('HOVER',[.04 .195 .44 .065],@(~,~)command("hover"),[.16 .36 .28]);
mkbtn('LAND',[.52 .195 .44 .065],@(~,~)command("land"),[.55 .20 .16]);
mkbtn('RETURN TO HOME',[.04 .105 .92 .065],@(~,~)command("return to home"),[.48 .27 .12]);

axAlt=axes('Parent',fig,'Position',[.045 .035 .26 .105],'Color',[.08 .10 .14], ...
    'XColor',[.78 .84 .9],'YColor',[.78 .84 .9]);
hAlt=animatedline(axAlt,'Color',[.2 .95 .62],'LineWidth',1.4); grid(axAlt,'on');
title(axAlt,'ALTITUDE (m)','Color','w'); xlabel(axAlt,'time (s)');
axErr=axes('Parent',fig,'Position',[.36 .035 .26 .105],'Color',[.08 .10 .14], ...
    'XColor',[.78 .84 .9],'YColor',[.78 .84 .9]);
hErr=animatedline(axErr,'Color',[1 .68 .16],'LineWidth',1.4); grid(axErr,'on');
title(axErr,'POSITION ERROR (m)','Color','w'); xlabel(axErr,'time (s)');
axMot=axes('Parent',fig,'Position',[.70 .035 .26 .105],'Color',[.08 .10 .14], ...
    'XColor',[.78 .84 .9],'YColor',[.78 .84 .9]);
motorBars=bar(axMot,1:4,zeros(1,4),'FaceColor',[.15 .75 .95]); ylim(axMot,[0 1]);
set(axMot,'XTick',1:4); grid(axMot,'on'); title(axMot,'MOTOR OUTPUTS','Color','w');

%% Main simulation loop
while isgraphics(fig) && running
    frame=tic;
    if ~paused
        t=t+dt;
        % Dynamic wind plus optional time-limited gust.
        windBase=[1.2 2.0 3.0]; if windLevel==2, windBase(1)=6; elseif windLevel==3, windBase(1)=10; end
        wind=[windBase(windLevel)*cos(.45)+.7*sin(.8*t); ...
              windBase(windLevel)*sin(.45)+.6*cos(.65*t); .05*sin(.7*t)];
        if t<gustEnd
            env=sin(pi*(gustEnd-t)/8);
            wind=wind+[7*env;4*env;0.6*env];
        end

        % Mission target progression only after accurate arrival and settling.
        if missionOn
            target=W(missionIdx,:).';
            if norm(pos(1:2)-target(1:2))<0.45 && abs(pos(3)-target(3))<0.30 && norm(vel)<0.35
                pos(1:2)=target(1:2); pos(3)=target(3); vel(:)=0; % precise arrival lock
                if missionIdx<size(W,1)
                    missionIdx=missionIdx+1;
                    commandText="Waypoint reached exactly: "+Wname(missionIdx-1)+"; proceeding to "+Wname(missionIdx);
                else
                    missionOn=false; target=pos; mode="HOVER";
                    commandText="Mission complete: final HOME waypoint reached";
                end
            end
        end
        % Random roaming: select a fresh, obstacle-clear target whenever the
        % current target is reached. This is not a fixed waypoint route.
        if randomFlightOn && armed && ~landing && ~missionOn && ~orbitOn
            if norm(pos-target)<1.15 && norm(vel)<0.70
                target=chooseRandomTarget();
                mode="RANDOM FLIGHT";
                commandText="Random roaming: new safe target selected";
            end
        end

        if orbitOn
            orbitPhase=orbitPhase+.28*dt;
            target=[orbitCenter(1)+orbitRadius*cos(orbitPhase); ...
                    orbitCenter(2)+orbitRadius*sin(orbitPhase); orbitCenter(3)];
            yawTarget=atan2(orbitCenter(2)-pos(2),orbitCenter(1)-pos(1));
        end

        % Once take-off altitude is reached, move in the commanded heading or
        % reverse direction. There is no pre-programmed waypoint route.
        if mode=="TAKEOFF" && pos(3)>=5.75 && abs(vel(3))<0.45
            cruiseOn=true;
            mode="CRUISE";
            commandText="Take-off complete — following command-directed heading";
        end
        if resumeCruiseAfterTurn && armed && pos(3)>0.5
            yawGap=atan2(sin(yawTarget-att(3)),cos(yawTarget-att(3)));
            if abs(yawGap)<deg2rad(3)
                resumeCruiseAfterTurn=false;
                cruiseOn=true;
                mode="CRUISE";
                commandText="Turn complete — continuing along the new heading";
            end
        end
        if cruiseOn && armed && ~landing && mode~="RTH" && mode~="OBSTACLE STOP"
            lookAhead=4.0;
            target=[pos(1)+cruiseDirection*lookAhead*cos(yawTarget); ...
                    pos(2)+cruiseDirection*lookAhead*sin(yawTarget); max(6,pos(3))];
        end

        % Sensor measurements: GPS, barometer, IMU (noise is intentionally small).
        gps=pos(1:2)+0.10*randn(2,1);
        baro=pos(3)+0.04*randn;
        imu=att+deg2rad(.15)*randn(3,1);

        % Safety systems.
        if norm(pos(1:2)-home(1:2))>geofenceRadius
            missionOn=false; orbitOn=false; mode="RTH"; target=[home(1:2);6];
            alarmText="GEOFENCE"; commandText="GEOFENCE FAILSAFE: RETURN HOME";
        end
        if battery<20 && ~landing
            missionOn=false; orbitOn=false; mode="RTH"; target=[home(1:2);6];
            alarmText="LOW BATTERY"; commandText="LOW BATTERY: RETURN HOME";
        end
        if battery<10
            missionOn=false; orbitOn=false; landing=true; mode="LANDING";
            target=[home(1:2);0]; alarmText="CRITICAL BATTERY";
        end
        if mode=="RTH" && norm(pos(1:2)-home(1:2))<1 && pos(3)<7
            landing=true; mode="LANDING"; target=[home(1:2);0];
        end

        if armed
            % Cascaded position/velocity control with a tilt-limited horizontal
            % acceleration command. This keeps the displayed attitude consistent
            % with the acceleration the simplified airframe can produce.
            e=target-pos;
            aCmd=[1.55*e(1)-1.75*vel(1); ...
                  1.55*e(2)-1.75*vel(2); ...
                  2.35*e(3)-2.0*vel(3)];
            aMaxXY=g*tan(maxTilt);
            aCmd(1:2)=max(-aMaxXY,min(aMaxXY,aCmd(1:2)));
            aCmd(3)=max(-2.5,min(2.5,aCmd(3)));

            % Attitude has finite angular-rate response rather than changing
            % instantaneously. Yaw uses the shortest angular path.
            yawErr=atan2(sin(yawTarget-att(3)),cos(yawTarget-att(3)));
            pitchCmd=max(-maxTilt,min(maxTilt,-atan2(aCmd(1),g)));
            rollCmd=max(-maxTilt,min(maxTilt, atan2(aCmd(2),g)));
            attCmd=[rollCmd;pitchCmd;att(3)+max(-0.9,min(0.9,1.8*yawErr))*dt];
            desiredRate=(attCmd-att)/max(dt,eps);
            desiredRate(1:2)=max(-deg2rad(75),min(deg2rad(75),desiredRate(1:2)));
            desiredRate(3)=max(-deg2rad(100),min(deg2rad(100),desiredRate(3)));
            attRate=attRate+min(1,dt*8)*(desiredRate-attRate);
            att=att+attRate*dt;
            att(3)=atan2(sin(att(3)),cos(att(3)));

            % Translational motion comes from the actual simulated attitude:
            % pitch/roll tilt the thrust vector, with drag and wind as disturbances.
            % Vertical thrust compensates for tilt to avoid artificial altitude loss.
            windAcc=0.10*wind(1:2);
            dragAcc=-0.055*vel(1:2).*abs(vel(1:2));
            axBody=-g*tan(att(2));
            ayBody= g*tan(att(1));
            thrustZ=(g+aCmd(3))/max(0.72,cos(att(1))*cos(att(2)));
            azBody=thrustZ-g-0.20*vel(3)+0.025*wind(3);
            rawAcc=[axBody+windAcc(1)+dragAcc(1); ...
                    ayBody+windAcc(2)+dragAcc(2); azBody];
            rawAcc(1:2)=max(-aMaxXY,min(aMaxXY,rawAcc(1:2)));
            rawAcc(3)=max(-2.8,min(2.8,rawAcc(3)));
            accState=accState+min(1,dt*5)*(rawAcc-accState);
            acc=accState;

            % Predict the next position before applying motion. Stop at the last safe
            % position if the horizontal path intersects any obstacle envelope.
            candidateVel=vel+acc*dt;
            sp=norm(candidateVel(1:2));
            if sp>maxSpeed, candidateVel(1:2)=candidateVel(1:2)*(maxSpeed/sp); end
            candidateVel(3)=max(-2.2,min(2.2,candidateVel(3)));
            candidatePos=pos+candidateVel*dt;
            if (mode=="OBSTACLE STOP" && ~cruiseOn) || resumeCruiseAfterTurn
                % Latched stop / rotate-in-place after an obstacle: hold the exact last-safe position until the user
                % issues a new turn command or RETURN HOME.
                candidateVel(:)=0;
                candidatePos=pos;
                accState(:)=0;
            end
            obstacleHit=0;
            if candidatePos(3)>0.5
                for oi=1:numel(obstacleR)
                    obsCenter=obstacleXY(oi,:).';
                    segment=candidatePos(1:2)-pos(1:2);
                    segLen2=dot(segment,segment);
                    if segLen2>eps
                        tau=max(0,min(1,dot(obsCenter-pos(1:2),segment)/segLen2));
                        closestXY=pos(1:2)+tau*segment;
                    else
                        closestXY=pos(1:2);
                    end
                    horizontalDistance=norm(closestXY-obsCenter);
                    safetyRadius=obstacleR(oi)+obstacleSafety;
                    currentDistance=norm(pos(1:2)-obsCenter);
                    nextDistance=norm(candidatePos(1:2)-obsCenter);
                    % Conservative city-navigation rule: treat configured
                    % obstacles as blocked regions in the horizontal flight
                    % plane. Do not allow the drone to pass over a low barrier
                    % just because the simplified altitude estimate is higher.
                    % If already inside a safety envelope due to inertia, only
                    % allow movement that increases clearance.
                    movingAwayFromExistingIntrusion=(currentDistance<safetyRadius && ...
                        nextDistance>currentDistance+1e-3);
                    enteringSafetyEnvelope=(currentDistance>=safetyRadius && ...
                        horizontalDistance<=safetyRadius);
                    movingCloserInsideEnvelope=(currentDistance<safetyRadius && ...
                        nextDistance<=currentDistance+1e-3);
                    if (enteringSafetyEnvelope || movingCloserInsideEnvelope) ...
                            && ~movingAwayFromExistingIntrusion
                        obstacleHit=oi;
                        break;
                    end
                end
            end

            % Building collision check: approximate each rectangular footprint by
            % a conservative bounding circle and include the drone's safety margin.
            buildingHit=0;
            if obstacleHit==0 && candidatePos(3)>0.5
                % Buildings are treated as no-fly footprints at all simulated
                % flight altitudes; conservative for this city-map demo.
                for bi=1:size(Bld,1)
                    bCenter=Bld(bi,1:2).';
                    seg=candidatePos(1:2)-pos(1:2);
                    seg2=dot(seg,seg);
                    if seg2>eps
                        btau=max(0,min(1,dot(bCenter-pos(1:2),seg)/seg2));
                        bClosest=pos(1:2)+btau*seg;
                    else
                        bClosest=pos(1:2);
                    end
                    bRadius=0.5*hypot(Bld(bi,3),Bld(bi,4))+obstacleSafety+0.25;
                    bCurrent=norm(pos(1:2)-bCenter);
                    bNext=norm(candidatePos(1:2)-bCenter);
                    bIntrusion=bCurrent<bRadius && bNext>bCurrent+1e-3;
                    bEntering=(bCurrent>=bRadius && norm(bClosest-bCenter)<=bRadius);
                    bCloser=(bCurrent<bRadius && bNext<=bCurrent+1e-3);
                    if (bEntering || bCloser) && ~bIntrusion
                        buildingHit=bi;
                        break;
                    end
                end
            end

            if obstacleHit>0 || buildingHit>0
                % Hard stop at the last safe position. Never back away, detour,
                % or continue through an obstacle automatically.
                vel(:)=0;
                accState(:)=0;
                target=pos;
                missionOn=false;
                orbitOn=false;
                randomFlightOn=false;
                cruiseOn=false;
                resumeCruiseAfterTurn=false;
                if obstacleHit>0
                    hitName=obstacleNames(obstacleHit);
                else
                    hitName="BUILDING "+string(buildingHit);
                end
                mode="OBSTACLE STOP";
                alarmText="OBSTACLE AHEAD";
                lastCommand="OBSTACLE STOP";
                commandText="Obstacle ahead: "+hitName+" — stopped before safety boundary";
                voiceHud.String=['OBSTACLE STOP: ' char(hitName)];
            else
                vel=candidateVel;
                pos=candidatePos;
            end
            % motor values are a visual proxy for thrust/control effort
            collective=max(0,min(1,(g+aCmd(3))/(2*g)));
            motorMix=[rollCmd; -rollCmd; pitchCmd; -pitchCmd]*0.42;
            motorTarget=max(0,min(1,collective+motorMix));
            motors=motors+min(1,dt*10)*(motorTarget-motors); % motor spool-up lag
            battery=max(0,battery-(0.008+0.045*sum(motors.^2))*dt);
        else
            motors(:)=0;
        end

        % Ground contact and landing logic.
        if pos(3)<0, pos(3)=0; vel(3)=max(0,vel(3)); end
        if landing && pos(3)<0.12 && norm(vel)<0.25
            pos(3)=0; vel(:)=0; armed=false; landing=false; mode="DISARMED";
            motors(:)=0; commandText="Touchdown confirmed — motors disarmed";
        end
        if pos(3)==0 && ~armed, vel(:)=0; end

        % Telemetry and render at 10 Hz.
        if t-lastTelemetry>=0.1
            lastTelemetry=t;
            logT(end+1,1)=t; logPos(end+1,:)=pos.'; logVel(end+1,:)=vel.';
            logAtt(end+1,:)=att.'; logTarget(end+1,:)=target.';
            logMot(end+1,:)=motors.'; logWind(end+1,:)=wind.'; logBat(end+1,1)=battery;
            addpoints(hAlt,t,pos(3)); addpoints(hErr,t,norm(target-pos));
            motorBars.YData=motors.';
            addpoints(trail,pos(1),pos(2),pos(3));
            set(targetMark,'XData',target(1),'YData',target(2),'ZData',target(3)+0.18);
            if norm(target-pos)<1.0
                set(targetMark,'Visible','off'); % do not hide the drone when target and aircraft coincide
            else
                set(targetMark,'Visible','on');
            end
            set(windArrow,'XData',pos(1),'YData',pos(2),'ZData',pos(3), ...
                'UData',wind(1)*.6,'VData',wind(2)*.6,'WData',wind(3)*.6);
            set(shadow,'XData',[pos(1)-.55 pos(1)+.55],'YData',[pos(2)-.55 pos(2)+.55],'ZData',[.16 .16]);
            renderDrone();
            updateHud();
        end
    end
    drawnow limitrate;
    if isgraphics(fig), pause(max(0,dt-toc(frame))); else, break; end
end

    function mkbtn(label,where,cb,color)
        uicontrol(panel,'Style','pushbutton','String',label,'Units','normalized', ...
            'Position',where,'Callback',cb,'FontWeight','bold','FontSize',9, ...
            'BackgroundColor',color,'ForegroundColor',[.97 .98 1]);
    end

    function renderDrone()
        % Visual frame: arms are wide enough to read as a quadcopter on screen.
        R=rotZYX(att(1),att(2),att(3)); c=pos;
        c(3)=pos(3)+0.34; % body centre sits above the landing-gear contact plane
        L=armLen;
        local=[ L/sqrt(2) L/sqrt(2) -L/sqrt(2) -L/sqrt(2); ...
                L/sqrt(2) -L/sqrt(2) L/sqrt(2) -L/sqrt(2); zeros(1,4)];
        mpos=R*local+c;
        set(arm1,'XData',[mpos(1,1) mpos(1,2)],'YData',[mpos(2,1) mpos(2,2)],'ZData',[mpos(3,1) mpos(3,2)]);
        set(arm2,'XData',[mpos(1,3) mpos(1,4)],'YData',[mpos(2,3) mpos(2,4)],'ZData',[mpos(3,3) mpos(3,4)]);
        set(armAccent1,'XData',[mpos(1,1) mpos(1,2)],'YData',[mpos(2,1) mpos(2,2)],'ZData',mpos(3,[1 2])+.03);
        set(armAccent2,'XData',[mpos(1,3) mpos(1,4)],'YData',[mpos(2,3) mpos(2,4)],'ZData',mpos(3,[3 4])+.03);
        [sx,sy,sz]=sphere(14);
        Q=R*[.48*sx(:)';.34*sy(:)';.19*sz(:)']+c;
        set(body,'XData',reshape(Q(1,:),size(sx)),'YData',reshape(Q(2,:),size(sy)),'ZData',reshape(Q(3,:),size(sz)));
        noseVec=R*[.58;0;.04]+c;
        set(nose,'XData',[c(1) noseVec(1)],'YData',[c(2) noseVec(2)],'ZData',[c(3)+.04 noseVec(3)]);
        cp=R*[.58;0;-.18]+c; set(camera,'XData',cp(1),'YData',cp(2),'ZData',cp(3));
        lg1=R*[-.42 .42; .30 .30; -.32 -.32]+c;
        lg2=R*[-.42 .42; -.30 -.30; -.32 -.32]+c;
        set(leg1,'XData',lg1(1,:),'YData',lg1(2,:),'ZData',lg1(3,:));
        set(leg2,'XData',lg2(1,:),'YData',lg2(2,:),'ZData',lg2(3,:));
        [sx,sy,sz]=sphere(10);
        for j=1:4
            mc=mpos(:,j);
            Q=R*[.14*sx(:)';.14*sy(:)';.10*sz(:)']+mc;
            set(pods(j),'XData',reshape(Q(1,:),size(sx)),'YData',reshape(Q(2,:),size(sy)),'ZData',reshape(Q(3,:),size(sz)));
            aa2=linspace(0,2*pi,48); rad=.36;
            ring=R*[rad*cos(aa2);rad*sin(aa2);.03*ones(size(aa2))]+mc;
            set(rotorRings(j),'XData',ring(1,:),'YData',ring(2,:),'ZData',ring(3,:));
            [pr,pt]=meshgrid(linspace(0,rad,18),linspace(0,2*pi,18));
            propLocal=[pr(:)'.*cos(pt(:))'; pr(:)'.*sin(pt(:))'; 0.035*ones(1,numel(pr))];
            propWorld=R*propLocal+mc;
            set(propBlur(j),'XData',reshape(propWorld(1,:),size(pr)), ...
                'YData',reshape(propWorld(2,:),size(pr)), ...
                'ZData',reshape(propWorld(3,:),size(pr)));
            phase=t*(13+40*motors(j))*(-1)^j;
            b1=R*[-rad*cos(phase) rad*cos(phase);-rad*sin(phase) rad*sin(phase);.04 .04]+mc;
            b2=R*[-rad*cos(phase+pi/2) rad*cos(phase+pi/2); ...
                  -rad*sin(phase+pi/2) rad*sin(phase+pi/2);.045 .045]+mc;
            set(blades(2*j-1),'XData',b1(1,:),'YData',b1(2,:),'ZData',b1(3,:));
            set(blades(2*j),'XData',b2(1,:),'YData',b2(2,:),'ZData',b2(3,:));
            set(leds(j),'XData',mc(1),'YData',mc(2),'ZData',mc(3)+.13);
        end
        % Clouds drift with time; camera gently follows the aircraft.
        for j=1:4
            cc=cloudSeed(j,:); cc(1)=mod(cc(1)+.045*t+30,60)-30;
            [cx,cy,cz]=sphere(7);
            set(clouds(j),'XData',cc(1)+1.9*cx,'YData',cc(2)+1.2*cy,'ZData',cc(3)+0.55*cz);
        end
        xlim(ax,[pos(1)-15 pos(1)+15]); ylim(ax,[pos(2)-15 pos(2)+15]);
        zlim(ax,[0 max(22,pos(3)+12)]);
    end

    function updateHud()
        hud.String=sprintf(['SIM TIME    %7.1f s\nMODE        %s\nARMED       %s\nCOMMAND     %s\nSTATUS      %s\nTARGET      %s\nEAST        %7.2f m\nNORTH       %7.2f m\nALTITUDE    %7.2f m\nSPEED       %7.2f m/s\nROLL        %7.1f deg\nPITCH       %7.1f deg\nYAW         %7.1f deg\nERROR       %7.2f m\nWIND        %7.2f m/s\nBATTERY     %7.1f %%\nSAFETY      %s'], ...
            t,mode,string(armed),lastCommand,char(commandText),targetName(),pos(1),pos(2),pos(3), ...
            norm(vel),rad2deg(att(1)),rad2deg(att(2)),rad2deg(att(3)),norm(target-pos),norm(wind),battery,alarmText);
    end

    function name=targetName()
        if randomFlightOn, name='RANDOM ROAM';
        elseif missionOn, name=char(Wname(missionIdx));
        elseif orbitOn, name='ORBIT';
        elseif mode=="RTH", name='HOME / RTH';
        else, name='HOLD / MANUAL'; end
    end

    function armToggle(~,~)
        if ~armed
            armed=true; mode="ARMED / HOVER"; target=pos; yawTarget=att(3);
            commandText="Armed"; lastCommand="ARM";
        elseif pos(3)<.15
            armed=false; mode="DISARMED"; motors(:)=0; lastCommand="DISARM";
        else
            commandText="Do not disarm in flight — use LAND"; return;
        end
    end

    function listenVoice(~,~)
        voiceHud.String='VOICE: Listening... speak one command now'; drawnow;
        base=fileparts(mfilename('fullpath'));
        helper=fullfile(base,'listen_voice.ps1');
        resultFile=fullfile(tempdir,'droneflightlab_voice.txt');
        if ~isfile(helper)
            voiceHud.String='VOICE ERROR: listen_voice.ps1 missing';
            errordlg(['Voice helper not found:' newline helper],'Voice input error');
            return;
        end
        if isfile(resultFile), delete(resultFile); end

        % -STA helps System.Speech run consistently in Windows PowerShell.
        psCmd=sprintf('powershell.exe -NoProfile -STA -ExecutionPolicy Bypass -File "%s" -OutputFile "%s"',helper,resultFile);
        [rc,cmdout]=system(psCmd);

        spoken="";
        if isfile(resultFile)
            spoken=strtrim(string(fileread(resultFile)));
        end
        if startsWith(upper(spoken),"ERROR:")
            voiceHud.String=char(extractBefore(spoken,min(strlength(spoken)+1,75)));
            errordlg(char(spoken),'Windows voice recognition');
            return;
        elseif rc==0 && strlength(spoken)>0
            lastVoice=char(spoken);
            voiceHud.String=['HEARD: ' char(spoken)];
            command(spoken);
        elseif contains(upper(spoken),"NO_MATCH")
            voiceHud.String='VOICE: No command heard. Click and speak clearly.';
        elseif contains(upper(spoken),"NO_ENGLISH_RECOGNIZER")
            voiceHud.String='VOICE ERROR: Install Windows English speech recognition';
            errordlg(['Windows has no installed English speech recognizer.' newline ...
                'Open Settings > Time & language > Speech and install an English speech language.'], ...
                'Speech recognition unavailable');
        elseif contains(upper(cmdout),"MICROPHONE") || contains(upper(cmdout),"AUDIO DEVICE")
            voiceHud.String='VOICE ERROR: microphone unavailable';
            errordlg(char(cmdout),'Microphone error');
        else
            voiceHud.String='VOICE ERROR: see diagnostic message';
            if strlength(spoken)==0, spoken=string(cmdout); end
            if strlength(spoken)==0
                spoken="No result returned. Check microphone permission and Windows speech language.";
            end
            errordlg(char(spoken),'Voice input failed');
        end
    end

    function command(raw)
        q=lower(strtrim(string(raw)));
        q=replace(q,[".","!",",","?"],"");
        q=regexprep(q,'\s+',' ');
        lastCommand=char(q);
        commandText="Command: "+q;

        % Only seven flight commands are accepted by manual buttons and voice.
        missionOn=false; orbitOn=false; randomFlightOn=false;

        if q=="take off"
            if ~armed, armed=true; end
            landing=false; cruiseOn=false; resumeCruiseAfterTurn=false;
            cruiseDirection=1;
            target=[pos(1);pos(2);6];
            yawTarget=att(3);
            mode="TAKEOFF";
            alarmText="OK";
            commandText="Take-off requested — climbing to 6 m";
        elseif q=="turn left"
            if ~armed || pos(3)<0.5
                commandText="Turn command ignored: take off first";
                return;
            end
            landing=false; cruiseDirection=1;
            yawTarget=yawTarget+deg2rad(45);
            cruiseOn=true; resumeCruiseAfterTurn=false;
            mode="CRUISE";
            commandText="Turning left 45 degrees; moving along the new heading";
            alarmText="OK";
        elseif q=="turn right"
            if ~armed || pos(3)<0.5
                commandText="Turn command ignored: take off first";
                return;
            end
            landing=false; cruiseDirection=1;
            yawTarget=yawTarget-deg2rad(45);
            cruiseOn=true; resumeCruiseAfterTurn=false;
            mode="CRUISE";
            commandText="Turning right 45 degrees; moving along the new heading";
            alarmText="OK";
        elseif q=="backward"
            if ~armed || pos(3)<0.5
                commandText="Backward command ignored: take off first";
                return;
            end
            landing=false; resumeCruiseAfterTurn=false;
            cruiseDirection=-1; cruiseOn=true;
            target=[pos(1)-4*cos(yawTarget); pos(2)-4*sin(yawTarget); max(6,pos(3))];
            mode="BACKWARD";
            alarmText="OK";
            commandText="Moving backward relative to the current heading; obstacle checks active";
        elseif q=="hover"
            landing=false; cruiseOn=false; cruiseDirection=1;
            resumeCruiseAfterTurn=false;
            target=pos;
            vel(:)=0; accState(:)=0;
            mode="HOVER";
            alarmText="OK";
            commandText="Hovering at the current position";
        elseif q=="land"
            cruiseOn=false; cruiseDirection=1; resumeCruiseAfterTurn=false;
            landing=true;
            target=[pos(1:2);0];
            mode="LANDING";
            alarmText="OK";
            commandText="Landing vertically at the current position";
        elseif q=="return to home"
            cruiseOn=false; cruiseDirection=1; resumeCruiseAfterTurn=false;
            if pos(3)<0.5 && norm(pos(1:2)-home(1:2))<0.5
                target=home;
                mode="DISARMED";
                commandText="Already at home";
                return;
            end
            if ~armed, armed=true; end
            landing=false;
            target=[home(1:2);6];
            mode="RTH";
            alarmText="OK";
            commandText="Returning to home; stops if an obstacle blocks the path";
        else
            commandText="Unsupported command. Use TAKE OFF, TURN LEFT, TURN RIGHT, BACKWARD, HOVER, LAND, or RETURN TO HOME";
            voiceHud.String='Unsupported command — use one of the seven listed commands';
        end
    end

    function p=chooseRandomTarget()
        % Pick a new target inside the local flight area, away from obstacles
        % and sufficiently separated from the current position for visible travel.
        p=pos;
        for attempt=1:250
            candidate=[-31+62*rand; -31+62*rand; 5.0+7.0*rand];
            if norm(candidate(1:2)-home(1:2))>geofenceRadius-12
                continue;
            end
            if norm(candidate(1:2)-pos(1:2))<5.0
                continue;
            end
            safe=true;
            for oi=1:numel(obstacleR)
                clearance=obstacleR(oi)+obstacleSafety+1.25;
                if norm(candidate(1:2)-obstacleXY(oi,:).')<clearance
                    safe=false; break;
                end
            end
            if safe
                for bi=1:size(Bld,1)
                    bClearance=0.5*hypot(Bld(bi,3),Bld(bi,4))+obstacleSafety+1.0;
                    if norm(candidate(1:2)-Bld(bi,1:2).')<bClearance
                        safe=false; break;
                    end
                end
            end
            if safe
                p=candidate;
                return;
            end
        end
        % Conservative fallback if random sampling did not find a clear point.
        theta=2*pi*rand;
        p=[pos(1)+8*cos(theta); pos(2)+8*sin(theta); max(5,min(12,pos(3)))];
    end

    function setWind(level)
        windLevel=max(1,min(3,level));
        if windLevel==1, commandText="Light wind enabled";
        elseif windLevel==2, commandText="Strong wind enabled";
        else, commandText="Storm wind enabled"; end
    end

    function gust(~,~)
        gustEnd=t+8; commandText="Wind gust injected for 8 seconds"; lastCommand="GUST TEST";
    end

    function saveCsv(~,~)
        if isempty(logT), msgbox('Run the simulation for a few seconds first.'); return; end
        filename=fullfile(pwd,'DroneFlightLab_Telemetry.csv');
        T=table(logT,logPos(:,1),logPos(:,2),logPos(:,3),logVel(:,1),logVel(:,2),logVel(:,3), ...
            logAtt(:,1),logAtt(:,2),logAtt(:,3),logTarget(:,1),logTarget(:,2),logTarget(:,3), ...
            logMot(:,1),logMot(:,2),logMot(:,3),logMot(:,4),logWind(:,1),logWind(:,2),logBat, ...
            'VariableNames',{'Time_s','East_m','North_m','Altitude_m','VE','VN','VU','Roll_rad','Pitch_rad','Yaw_rad', ...
            'TargetEast_m','TargetNorth_m','TargetAltitude_m','Motor1','Motor2','Motor3','Motor4','WindEast','WindNorth','Battery_pct'});
        writetable(T,filename); msgbox(['Saved telemetry to:' newline filename],'CSV export');
    end

    function analysis(~,~)
        if size(logPos,1)<10, msgbox('Run a flight first to collect telemetry.'); return; end
        e=sqrt(mean(sum((logTarget-logPos).^2,2)));
        peak=max(vecnorm(logVel,2,2));
        figure('Name','Flight Analysis','Color','w');
        tiledlayout(2,2);
        nexttile; plot(logT,logPos,'LineWidth',1.1); grid on; title('Position'); legend('East','North','Altitude');
        nexttile; plot(logT,vecnorm(logTarget-logPos,2,2),'LineWidth',1.2); grid on; title('3D target error'); ylabel('m');
        nexttile; plot(logT,logAtt*180/pi,'LineWidth',1.1); grid on; title('Attitude'); legend('Roll','Pitch','Yaw');
        nexttile; plot(logT,logMot,'LineWidth',1.1); grid on; title('Motor outputs'); legend('M1','M2','M3','M4');
        msgbox(sprintf('Flight summary\nSamples: %d\nRMS 3D position error: %.3f m\nPeak speed: %.2f m/s\nFinal battery: %.1f%%', ...
            numel(logT),e,peak,battery),'Performance report');
    end

    function closeApp(~,~)
        running=false; if isgraphics(fig), delete(fig); end
    end
end

function drawBox(ax,x1,x2,y1,y2,z1,z2,color)
% Draw a closed rectangular prism with visible faces.
V=[x1 y1 z1; x2 y1 z1; x2 y2 z1; x1 y2 z1; ...
   x1 y1 z2; x2 y1 z2; x2 y2 z2; x1 y2 z2];
F=[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
patch(ax,'Vertices',V,'Faces',F,'FaceColor',color,'EdgeColor',[.25 .29 .34], ...
    'FaceAlpha',.98);
end

function R=rotZYX(phi,theta,psi)
cr=cos(phi); sr=sin(phi); cp=cos(theta); sp=sin(theta); cy=cos(psi); sy=sin(psi);
R=[cy*cp cy*sp*sr-sy*cr cy*sp*cr+sy*sr; ...
   sy*cp sy*sp*sr+cy*cr sy*sp*cr-cy*sr; ...
   -sp cp*sr cp*cr];
end
