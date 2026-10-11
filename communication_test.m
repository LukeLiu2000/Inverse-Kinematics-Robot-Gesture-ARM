clear;
clc;

disp("MATLAB sender started.");

% =========================
% 串口初始化
% =========================
port = "COM5";      % 改成你的 Arduino 串口号
baud = 57600;

s = serialport(port, baud);
configureTerminator(s, "LF");
flush(s);

pause(2);  % 等待 Arduino 重启完成

disp("Serial connected.");

% =========================
% 第一次发送
% =========================
sendXYZError(s, 5.0, 5.0, 5.0, 30.0);
disp("Sent first packet.");
pause(2);

% =========================
% 第二次发送
% =========================
sendXYZError(s, 8.0, 0.0, 6.0, 0.0);
disp("Sent second packet.");
pause(2);

% =========================
% 第三次发送
% =========================
sendXYZError(s, 5.0, -5.0, 7.0, -30.0);
disp("Sent third packet.");

disp("Demo finished.");

% ============================================================
% 发送函数：每次调用只发送一次
% 参数：
%   x, y, z, error
% ============================================================
function sendXYZError(s, x, y, z, error_deg)
    msg = sprintf('%.4f,%.4f,%.4f,%.4f', x, y, z, error_deg);
    writeline(s, msg);
    fprintf("Sent -> X: %.4f, Y: %.4f, Z: %.4f, error: %.4f\n", ...
            x, y, z, error_deg);
end