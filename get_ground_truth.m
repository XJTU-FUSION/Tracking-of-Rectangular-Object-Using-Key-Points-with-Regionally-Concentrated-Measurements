function [gt_state, gt_orientation,gt_length,gt_width,steps,T] = get_ground_truth
%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements 
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan*, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

steps = 130;                                                          
T = 1; 
gt_length = 4.8;                                                       
gt_width = 1.8;

F_cv1 = [1,T;
         0,1];                                                 
F_cv = blkdiag(F_cv1,F_cv1);

gt_angular_rate = pi/240;                                                  %% the object angular rate in the CT motion
F_ct = [1,    inv(gt_angular_rate)*sin(gt_angular_rate*T),0,-inv(gt_angular_rate)*(1-cos(gt_angular_rate*T));
        0,                         cos(gt_angular_rate*T),0,                         -sin(gt_angular_rate*T);
        0,inv(gt_angular_rate)*(1-cos(gt_angular_rate*T)),1,     inv(gt_angular_rate)*sin(gt_angular_rate*T);
        0,                         sin(gt_angular_rate*T),0,                          cos(gt_angular_rate*T)];

gt_state = zeros(5,steps);  %%  gt_state contains the object position, velocity, angular rate at each time
gt_orientation = zeros(1,steps);
I2 = [1;0];
for k = 1:1:steps
     if k==1
         gt_pos = [0;0];                                                   %% the object initial position
         gt_vel = [5;5];                                                   %% the object initial velocity
         gt_pos_vel = [gt_pos(1);gt_vel(1);gt_pos(2);gt_vel(2)];
         gt_angular_rate = 0;         
     elseif k>1&k<=30                                                                    
         gt_pos_vel = F_cv*gt_pos_vel;                                     
         gt_angular_rate = 0;
     elseif k>30&k<=90           
         gt_pos_vel = F_ct*gt_pos_vel;
         gt_angular_rate = pi/240;
     else           
         gt_pos_vel = F_cv*gt_pos_vel;  
         gt_angular_rate = 0;
     end  
     gt_state(:,k) = [gt_pos_vel;gt_angular_rate];       
     
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  get the object orientation
    gt_vel = [gt_pos_vel(2);gt_pos_vel(4)];
    if gt_pos_vel(4)>=0
        gt_orientation(k) = acos(dot(gt_vel,I2)/norm(gt_vel));      
    else
        gt_orientation(k) = 2*pi-acos(dot(gt_vel,I2)/norm(gt_vel));
    end
    
end
        
            
