function [Gnp] = Calgnp(h,lat)
%Calgnp： 函数计算在给定高度和纬度下的重力加速度，考虑了地球的椭球形状、自转和高度的影响。

%   重力模型见牛小骥老师PPT第二部分最后一页
a = 6378137; %m  地球长半轴（赤道半径）
% b = 6356752.3141; %m
We = 7.292115e-5; %rad/s 地球自转角速度，单位为弧度每秒
f = 1.0/298.257222101;  % 地球扁率，表示地球的扁平程度
b=(1-f)*a; % 地球短半轴（极半径）
GM = 3.986005e14; %m3/s2 地球的引力常数，单位为立方米每平方秒
gamaa = 9.7803267715; %m/s2 赤道处的重力加速度
gamab = 9.8321863685; %m/s2 极地处的重力加速度

m = We^2 * a^2 * b/GM; %一个与地球自转和引力常数相关的无量纲参数，用于修正重力加速度

gama = CalGama(lat);
%Gnp:计算在给定高度和纬度下的重力加速度
Gnp = gama * (1 - 2 * (1 + f + m - 2 * f * (sin(lat)^2)) * h / a + 3 * h^2 /(a^2) );
end

function [gama] = CalGama(lat)
%CalGama：函数计算在给定纬度下的重力加速度，考虑了地球的椭球形状和赤道、极地重力加速度的差异。
a = 6378137; %m
% b = 6356752.3141; %m
We = 7.292115e-5; %rad/s
f = 1.0/298.257222101;
b=(1-f)*a;
GM = 3.986005e14; %m3/s2
gamaa = 9.7803267715; %m/s2
gamab = 9.8321863685; %m/s2
% firstNum：赤道和极地重力加速度的加权平均值，权重有纬度的余弦和正弦的平方决定
firstNum = (a * gamaa * (cos(lat)^2) + b * gamab * (sin(lat)^2) );
% secondNum：在给定纬度下的半径，考虑了地球的椭球形状
secondNum = (sqrt(a^2 * (cos(lat)^2) + b^2 * (sin(lat)^2)));
gama = firstNum/secondNum;
end

