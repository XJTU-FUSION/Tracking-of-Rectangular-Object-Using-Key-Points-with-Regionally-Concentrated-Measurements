function [X_pre,P_pre] = Kinematic_prediction(X_update,P_update,T)
%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements 
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

X0_e = X_update; % [x1; v1; x2; v2; w]
P0_e = P_update;

w = X0_e(5,1);

X_temp_e = ...
[X0_e(1,1) + X0_e(2,1)*sin(X0_e(5,1)*T)/X0_e(5,1) - X0_e(4,1)*(1 - cos(X0_e(5,1)*T))/X0_e(5,1);
 X0_e(2,1)*cos(X0_e(5,1)*T) - X0_e(4,1)*sin(X0_e(5,1)*T);
 X0_e(3,1) + X0_e(2,1)*(1 - cos(X0_e(5,1)*T))/X0_e(5,1) + X0_e(4,1)*sin(X0_e(5,1)*T)/X0_e(5,1);
 X0_e(2,1)*sin(X0_e(5,1)*T) + X0_e(4,1)*cos(X0_e(5,1)*T);
 X0_e(5,1);
];

a1 = X0_e(2,1)*(T*cos(X0_e(5,1)*T)*X0_e(5,1) - sin(X0_e(5,1)*T))/(X0_e(5,1)^2) - X0_e(4,1)*(T*sin(X0_e(5,1)*T)*X0_e(5,1) - (1 - cos(X0_e(5,1)*T)))/(X0_e(5,1)^2);
a2 = X0_e(2,1)*(T*sin(X0_e(5,1)*T)*X0_e(5,1) - (1 - cos(X0_e(5,1)*T)))/(X0_e(5,1)^2) + X0_e(4,1)*(T*cos(X0_e(5,1)*T)*X0_e(5,1) - sin(X0_e(5,1)*T))/(X0_e(5,1)^2);
a3 = -X0_e(2,1)*sin(X0_e(5,1)*T)*T - X0_e(4,1)*cos(X0_e(5,1)*T)*T;
a4 = X0_e(2,1)*cos(X0_e(5,1)*T)*T - X0_e(4,1)*sin(X0_e(5,1)*T)*T;
 
A_5ct = ...
[1  sin(X0_e(5,1)*T)/X0_e(5,1) 0 -(1 - cos(X0_e(5,1)*T))/X0_e(5,1) a1;
0  cos(X0_e(5,1)*T) 0 -sin(X0_e(5,1)*T) a3;
0  (1 - cos(X0_e(5,1)*T))/X0_e(5,1) 1 sin(X0_e(5,1)*T)/X0_e(5,1) a2;
0  sin(X0_e(5,1)*T) 0 cos(X0_e(5,1)*T)  a4;
0  0                0 0                 1;
];

X_pre = X_temp_e;
P_pre = A_5ct*P0_e*A_5ct';
