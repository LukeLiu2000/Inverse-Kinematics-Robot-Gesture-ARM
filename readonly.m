clear;
clc;

% =========================
% 改成你的 Arduino 串口号
% =========================
port = "COM5";
baud = 115200;

% 打开串口
s = serialport(port, baud);
configureTerminator(s, "LF");
s.Timeout = 5;

% 清空缓冲区，等待 Arduino 重启
flush(s);
pause(2);

disp("MATLAB is reading from Arduino...");

while true
    try
        line = readline(s);   % 读一整行
        line = strtrim(line); % 去掉换行和空格
        fprintf("Received: %s\n", line);
    catch ME
        fprintf("Read error: %s\n", ME.message);
        pause(0.2);
    end
end