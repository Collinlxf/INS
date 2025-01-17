function [phim, dvbm, dphim, rotm, scullm] = cnscl(imu, coneoptimal)
% Coning & sculling compensation.
%
% Prototype: [phim, dvbm, dphim, rotm, scullm] = cnscl(imu, coneoptimal)
% Inputs:  imu(:,1:3) - gyro angular increments
%          imu(:,4:6) - acc velocity increments (may not exist)
%          coneoptimal - 0 for optimal coning compensation method, 最优圆锥补偿方法
%                           描述：使用最优的圆锥补偿方法，通常基于优化算法来最小化圆锥误差。
%                           优点：精度较高，适用于高精度惯性导航系统。
%                           缺点：计算复杂度较高，可能需要更多的计算资源。
%                           适用于对精度要求极高的场景，如航空航天、精密导航等。
%                        1 for polinomial compensation method. 多项式补偿方法
%                           描述：使用多项式方法对圆锥误差进行补偿，常见的多项式包括二次和三次多项式。
%                           优点：计算较为简单，适用于计算资源有限的情况。
%                           缺点：精度可能不如最优补偿方法。
%                           适用于精度要求中等且计算资源有限的场景，如消费级导航设备。
%                        2 single sample+previous sample 单样本 + 前一个样本补偿方法
%                           描述：使用当前样本和前一个样本的数据进行圆锥补偿，通常基于简化的补偿公式。
%                           优点：计算速度快，适用于实时性要求高的场景。
%                           缺点：精度较低，可能无法完全消除圆锥误差。
%                           适用于实时性要求高且计算资源有限的场景，如无人机、机器人等。
%                        3 high order coning compensation 高阶圆锥补偿方法
%                           描述：使用高阶补偿方法，考虑更多的历史样本数据，对圆锥误差进行更精细的补偿。
%                           优点：精度较高，适用于高精度惯性导航系统。
%                           缺点：计算复杂度较高，可能需要更多的计算资源和存储资源。
%                           适用于对精度要求高且计算资源充足的场景，如高精度定位系统。。        
% Outputs: phim - rotation vector after coning compensation 经过圆锥补偿后的旋转向量。
%          dvbm - velocity increment after rotation & sculling compensation  经过旋转和震荡补偿后的速度增量。
%          dphim - attitude coning error 姿态圆锥误差。
%          rotm - velocity rotation error 速度旋转误差。
%          scullm - velocity sculling error 速度震荡误差。
global glv
    if nargin<2,  coneoptimal=0;  end
    [n, m] = size(imu);
    %% coning compensation
    %  圆锥补偿
    wm = imu(:,1:3);
	if n==1
        wmm = wm;
        if coneoptimal==2
            % wm当前帧数据，wm_1上一帧数据
            dphim = 1/12*cros(glv.wm_1,wm);  if m<6, glv.wm_1 = wm; end
        else
            dphim = [0, 0, 0];
        end
	end
    phim = (wmm+dphim)';  dvbm = [0; 0; 0];
    %% sculling compensation
    % 震荡补偿
    if m>=6
        vm = imu(:,4:6); 
        if n==1
            vmm = vm;
            if coneoptimal==0
                scullm = [0, 0, 0];
            else
                % wm当前帧数据，wm_1上一帧数据
                scullm = 1/12*(cros(glv.wm_1,vm)+cros(glv.vm_1,wm));  glv.wm_1 = wm; glv.vm_1 = vm;
            end
        rotm = 1.0/2*cros(wmm,vmm);
        dvbm = (vmm+rotm+scullm)';
        end
    end