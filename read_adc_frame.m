% read_adc_frame.m
% 从单片机串口读取一帧 ADC 原始数据(4096点)，画时域波形 + 频谱，显示频率和幅度
%
% 协议(二进制, 小端):  AA 55 | len(2字节=4096) | 4096 * uint16
% 采样率 500 kHz, 参考电压 3.3 V, 12位 ADC
%
% 用法: 先改下面的 PORT 为你的串口号，然后直接运行。
%       脚本会自动发送 0D 09 命令触发单片机导出一帧，然后读取并绘图。

clear; clc;

PORT = "COM7";      % <<< 改成你的串口号(设备管理器查看)
fs   = 500000;      % 采样率 500 kHz
Vref = 3.3;         % ADC 参考电压
N    = 4096;        % 点数

% ---- 1. 打开串口 ----
s = serialport(PORT, 115200, "Timeout", 5);
flush(s);           % 清空缓冲区

% ---- 2. 触发导出一帧 (命令 0D 09) ----
write(s, uint8([0x0D 0x09]), "uint8");

% ---- 3. 找同步头 AA 55 ----
while true
    if read(s, 1, "uint8") == 0xAA
        if read(s, 1, "uint8") == 0x55
            break;
        end
    end
end

% ---- 4. 读长度(2字节小端) ----
lenB = read(s, 2, "uint8");
len  = double(lenB(1)) + 256*double(lenB(2));

% ---- 5. 读数据 len*2 字节 ----
dataBytes = read(s, len*2, "uint8");
clear s;

% ---- 6. 转 uint16 并去直流(2048 = 1.65V) ----
adc = double(typecast(uint8(dataBytes(:)), 'uint16')) - 2048;
t   = (0:numel(adc)-1) / fs * 1e6;   % us

% ---- 7. 时域波形 ----
figure('Name','ADC 时域波形','NumberTitle','off');
subplot(2,1,1);
plot(t, adc); grid on;
xlabel('时间 (us)'); ylabel('ADC code (去直流)');
title('时域波形');

% ---- 8. 频谱 + 峰值频率 ----
Nfft = numel(adc);
X = fft(adc);
f = (0:Nfft/2-1) * fs / Nfft;            % 0 ~ 250kHz
P = 2*abs(X(1:Nfft/2)) / Nfft;           % 单边幅度
P(f < 10e3) = 0;                         % 屏蔽直流/低频
[~, idx] = max(P);
f_peak = f(idx);

subplot(2,1,2);
plot(f/1e3, P); grid on;
xlabel('频率 (kHz)'); ylabel('幅度 (code)');
title('频谱');

% ---- 9. 幅度 Vpp + 结果输出 ----
Vpp = (max(adc) - min(adc)) * Vref / 4096;

fprintf('样本数     = %d\n', numel(adc));
fprintf('峰值频率   = %.2f kHz\n', f_peak/1e3);
fprintf('峰峰值幅度 = %.3f V\n', Vpp);
