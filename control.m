function mousetest_sender
clc;

% 先强制清理上一次可能残留的图窗和定时器
close all force;
delete(timerfindall);

disp("MATLAB sender with mouse/keyboard started.");

% =========================
% 状态初始化
% =========================
state.x = 0;
state.y = 0;
state.z = 0;
state.error_deg = 0;

state.stepXY = 0.05;
state.stepZ = 0.2;
state.stepError = 1.0;

state.prevMouse = [];

% 限幅范围（保留）
state.xMin = -20;
state.xMax = 20;
state.yMin = -20;
state.yMax = 20;
state.zMin = -20;
state.zMax = 20;
state.errorMin = -150;
state.errorMax = 150;

% 球壳范围
state.rMin = 2;
state.rMax = 20;

% =========================
% 串口初始化
% =========================
s = [];
hFig = [];
hPlotFig = [];
hCurrent = [];
hControlAx = [];
hControlCircle = [];
hControlPoint = [];
hControlOrigin = [];
hControlXAxis = [];
hControlYAxis = [];
hControlTextX = [];
hControlTextY = [];
stopRequested = false;

cleanupObj = onCleanup(@cleanupAll); %#ok<NASGU>

port = "COM5";      % 改成你的 Arduino 串口号
baud = 38400;

try
    s = serialport(port, baud);
    configureTerminator(s, "LF");
    flush(s);
    pause(2);  % 等待 Arduino 重启完成
    disp("Serial connected.");
catch ME
    cleanupAll();
    rethrow(ME);
end

% =========================
% 创建控制窗口（正方形）
% =========================
screenSize = get(0, 'ScreenSize');
winSize = 800;
left = max(1, round((screenSize(3) - winSize) / 2));
bottom = max(1, round((screenSize(4) - winSize) / 2));

hFig = figure( ...
    'Name', 'Mouse / Keyboard Input', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'Position', [left, bottom, winSize, winSize], ...
    'KeyPressFcn', @onKey, ...
    'WindowScrollWheelFcn', @onScroll, ...
    'WindowButtonMotionFcn', @onMouseMove, ...
    'CloseRequestFcn', @onCloseControl);

% 控制面板：二维坐标系，X 轴向下，Y 轴向右
hControlAx = axes('Parent', hFig, 'Position', [0.08 0.08 0.84 0.84]);
hold(hControlAx, 'on');
grid(hControlAx, 'on');
box(hControlAx, 'on');
axis(hControlAx, 'equal');
axis(hControlAx, 'ij');   % 关键：让纵轴正方向向下
xlim(hControlAx, [-22 22]);
ylim(hControlAx, [-22 22]);

% 注意：这里把横轴当作 Y，把纵轴当作 X
xlabel(hControlAx, 'Y');
ylabel(hControlAx, 'X');
title(hControlAx, 'Mouse Control Plane');

% 坐标轴正方向
hControlXAxis = plot(hControlAx, [0 0], [-22 22], 'k-', 'LineWidth', 1.0);   % X 轴：向下
hControlYAxis = plot(hControlAx, [-22 22], [0 0], 'k-', 'LineWidth', 1.0);   % Y 轴：向右

% 原点黑点
hControlOrigin = plot(hControlAx, 0, 0, 'k.', 'MarkerSize', 25);

% 当前点红点（横轴=Y，纵轴=X）
hControlPoint = plot(hControlAx, state.y, state.x, 'r.', 'MarkerSize', 25);

% 正方向标注
hControlTextX = text(hControlAx, 0.6, 20.5, 'X+', 'Color', 'k', 'FontWeight', 'bold');
hControlTextY = text(hControlAx, 20.5, 0.6, 'Y+', 'Color', 'k', 'FontWeight', 'bold');

% 随 Z 变化的可达截面蓝圈
thetaCircle = linspace(0, 2*pi, 200);
rSlice = sqrt(max(0, state.rMax^2 - state.z^2));
circleY = rSlice * cos(thetaCircle);
circleX = rSlice * sin(thetaCircle);
hControlCircle = plot(hControlAx, circleY, circleX, 'b-', 'LineWidth', 1.2);

drawnow;

% =========================
% 创建 3D 显示窗口
% =========================
hPlotFig = figure( ...
    'Name', '3D Position View', ...
    'NumberTitle', 'off', ...
    'MenuBar', 'none', ...
    'ToolBar', 'none', ...
    'Color', [1 1 1], ...
    'CloseRequestFcn', @onClosePlot);

hAx = axes('Parent', hPlotFig);
hold(hAx, 'on');
grid(hAx, 'on');
box(hAx, 'on');
axis(hAx, 'equal');

xlim(hAx, [-22 22]);
ylim(hAx, [-22 22]);
zlim(hAx, [-22 22]);

xlabel(hAx, 'X');
ylabel(hAx, 'Y');
zlabel(hAx, 'Z');
view(hAx, 3);

% 外球：半径20，蓝色稀疏网格，透明
[sx1, sy1, sz1] = sphere(12);
surf(hAx, 20*sx1, 20*sy1, 20*sz1, ...
    'FaceColor', 'none', ...
    'EdgeColor', [0 0 1], ...
    'LineWidth', 0.5);

% 内球：半径2，蓝色稀疏网格，透明
[sx2, sy2, sz2] = sphere(12);
surf(hAx, 2*sx2, 2*sy2, 2*sz2, ...
    'FaceColor', 'none', ...
    'EdgeColor', [0 0 1], ...
    'LineWidth', 0.5);

% 原点黑点
plot3(hAx, 0, 0, 0, 'k.', 'MarkerSize', 25);

% 当前点红点
hCurrent = plot3(hAx, 0, 0, 0, 'r.', 'MarkerSize', 25);

title(hAx, '3D Position');
drawnow;

disp('Click the control window, then move mouse / scroll / press arrow keys.');
printStateAndSend();
updateControlPanel();
updatePlot(0, 0, 0);

% =========================
% 主循环
% =========================
while ~stopRequested && isvalid(hFig) && isvalid(hPlotFig)
    pause(0.05);
end

% ============================================================
% 打印、发送、更新图
% ============================================================
    function printStateAndSend()
        clampState();

        fprintf('X=%.3f, Y=%.3f, Z=%.3f, error=%.3f\n', ...
            state.x, state.y, state.z, state.error_deg);

        [sx, sy, sz] = projectToShell(state.x, state.y, state.z);
        sendXYZError(s, sx, sy, sz, state.error_deg);

        updateControlPanel();
        updatePlot(sx, sy, sz);
    end

    function updatePlot(px, py, pz)
        if ~isempty(hCurrent) && isgraphics(hCurrent)
            set(hCurrent, 'XData', px, 'YData', py, 'ZData', pz);
            drawnow limitrate;
        end
    end

    function updateControlPanel()
        if isempty(hControlAx) || ~isgraphics(hControlAx)
            return;
        end

        % 更新当前点（横轴=Y，纵轴=X）
        if ~isempty(hControlPoint) && isgraphics(hControlPoint)
            set(hControlPoint, 'XData', state.y, 'YData', state.x);
        end

        % 更新随 Z 变化的可达截面圆
        rSlice = sqrt(max(0, state.rMax^2 - state.z^2));
        thetaCircle = linspace(0, 2*pi, 200);
        circleY = rSlice * cos(thetaCircle);
        circleX = rSlice * sin(thetaCircle);

        if ~isempty(hControlCircle) && isgraphics(hControlCircle)
            set(hControlCircle, 'XData', circleY, 'YData', circleX);
        end

        title(hControlAx, sprintf('Mouse Control Plane   |   Z = %.2f   |   Reach Radius = %.2f', ...
            state.z, rSlice));

        drawnow limitrate;
    end

% ============================================================
% 限幅 + 球壳投影
% ============================================================
    function clampState()
        state.x = min(max(state.x, state.xMin), state.xMax);
        state.y = min(max(state.y, state.yMin), state.yMax);
        state.z = min(max(state.z, state.zMin), state.zMax);
        state.error_deg = min(max(state.error_deg, state.errorMin), state.errorMax);
    end

    function [px, py, pz] = projectToShell(x, y, z)
        v = [x, y, z];
        r = norm(v);

        if r > state.rMax
            v = v * (state.rMax / r);
        elseif r < state.rMin
            if r < eps
                % 原点没有唯一最近点，固定选一个方向
                v = [state.rMin, 0, 0];
            else
                v = v * (state.rMin / r);
            end
        end

        px = v(1);
        py = v(2);
        pz = v(3);
    end

% ============================================================
% 鼠标移动
% ============================================================
    function onMouseMove(~, ~)
        if isempty(hFig) || ~isgraphics(hFig)
            return;
        end

        cp = get(hFig, 'CurrentPoint');

        if isempty(state.prevMouse)
            state.prevMouse = cp;
            return;
        end

        delta = cp - state.prevMouse;
        state.prevMouse = cp;

        if abs(delta(1)) < 1 && abs(delta(2)) < 1
            return;
        end

        % 鼠标前后 -> X 增加/减小
        state.x = state.x - delta(2) * state.stepXY;

        % 鼠标左右 -> Y 增加/减小
        state.y = state.y + delta(1) * state.stepXY;

        printStateAndSend();
    end

% ============================================================
% 滚轮
% ============================================================
    function onScroll(~, evt)
        state.z = state.z + evt.VerticalScrollCount * state.stepZ;
        printStateAndSend();
    end

% ============================================================
% 键盘：方向键控制 error
% ============================================================
    function onKey(~, evt)
        switch evt.Key
            case 'uparrow'
                state.error_deg = state.error_deg + state.stepError;
            case 'downarrow'
                state.error_deg = state.error_deg - state.stepError;
            otherwise
                return;
        end

        printStateAndSend();
    end

% ============================================================
% 发送函数：每次调用只发送一次
% ============================================================
    function sendXYZError(serialObj, x, y, z, error_deg)
        if isempty(serialObj) || ~isvalid(serialObj)
            return;
        end

        msg = sprintf('#%+06.1f,%+06.1f,%+06.1f,%+06.1f$', x, y, z, error_deg);
        writeline(serialObj, msg);
    end

% ============================================================
% 关闭窗口
% ============================================================
    function onCloseControl(src, ~)
        stopRequested = true;
        if isgraphics(src)
            delete(src);
        end
        if ~isempty(hPlotFig) && isgraphics(hPlotFig)
            delete(hPlotFig);
        end
    end

    function onClosePlot(src, ~)
        stopRequested = true;
        if isgraphics(src)
            delete(src);
        end
        if ~isempty(hFig) && isgraphics(hFig)
            delete(hFig);
        end
    end

% ============================================================
% 统一清理：关闭图窗 + 释放串口
% ============================================================
    function cleanupAll()
        stopRequested = true;

        try
            if ~isempty(hCurrent) && isgraphics(hCurrent)
                delete(hCurrent);
            end
        catch
        end

        try
            if ~isempty(hPlotFig) && isgraphics(hPlotFig)
                delete(hPlotFig);
            end
        catch
        end

        try
            if ~isempty(hFig) && isgraphics(hFig)
                delete(hFig);
            end
        catch
        end

        try
            if ~isempty(s) && isvalid(s)
                delete(s);
            end
        catch
        end
    end
end