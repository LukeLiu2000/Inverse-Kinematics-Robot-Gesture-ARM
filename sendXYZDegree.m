function sendXYZDegree(X, Y, Z, error_deg)
% sendXYZDegree 通过串口向 Arduino 发送四个数值
%
% 调用方式：
%   sendXYZDegree(X, Y, Z, error_deg)
%
% 发送格式：
%   X,Y,Z,error_deg\n

    persistent s isInit

    if isempty(isInit) || isempty(s) || ~isvalid(s)
        % ------------------------------
        % 修改成你的串口号
        % ------------------------------
        port = "COM5";
        baud = 115200;

        s = serialport(port, baud);
        configureTerminator(s, "LF");
        flush(s);

        pause(2);  % 等待 Arduino 重启并准备好
        isInit = true;
    end

    msg = sprintf('%.4f,%.4f,%.4f,%.4f\n', X, Y, Z, error_deg);
    write(s, msg, "string");
end