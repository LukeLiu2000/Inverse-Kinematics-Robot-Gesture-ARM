function mousetest
clear;
clc;

% 初始值
state.x = 5;
state.y = 5;
state.z = 5;
state.error_deg = 0;

state.stepXY = 0.05;
state.stepZ = 0.1;
state.stepError = 1.0;

state.prevMouse = [];

% 限幅范围
state.xMin = -20;
state.xMax = 20;
state.yMin = -20;
state.yMax = 20;
state.zMin = -20;
state.zMax = 20;
state.errorMin = -150;
state.errorMax = 150;

% 创建正方形窗口，稍微大一点
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
    'CloseRequestFcn', @onClose);

axis off;

disp('Click the window, then move mouse / scroll / press arrow keys.');
printState();

    function printState()
        fprintf('X=%.3f, Y=%.3f, Z=%.3f, error=%.3f\n', ...
            state.x, state.y, state.z, state.error_deg);
    end

    function clampState()
        state.x = min(max(state.x, state.xMin), state.xMax);
        state.y = min(max(state.y, state.yMin), state.yMax);
        state.z = min(max(state.z, state.zMin), state.zMax);
        state.error_deg = min(max(state.error_deg, state.errorMin), state.errorMax);
    end

    function onMouseMove(~, ~)
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

        % 鼠标前后 -> x
        state.x = state.x + delta(2) * state.stepXY;

        % 鼠标左右 -> y
        state.y = state.y - delta(1) * state.stepXY;

        clampState();
        printState();
    end

    function onScroll(~, evt)
        % 鼠标滚轮：向前滚 -> z 减少；向后滚 -> z 增加
        state.z = state.z + evt.VerticalScrollCount * state.stepZ;

        clampState();
        printState();
    end

    function onKey(~, evt)
        switch evt.Key
            case 'uparrow'
                state.error_deg = state.error_deg + state.stepError;
            case 'downarrow'
                state.error_deg = state.error_deg - state.stepError;
            otherwise
                return;
        end

        clampState();
        printState();
    end

    function onClose(src, ~)
        delete(src);
    end
end