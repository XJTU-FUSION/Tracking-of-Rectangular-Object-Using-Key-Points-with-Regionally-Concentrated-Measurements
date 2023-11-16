%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements 
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan*, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

close all;
clear all;
clc;

%%%%%% generate ground truth
[gt_state, gt_orientation,gt_length,gt_width,steps,T] = get_ground_truth;

%%%%%% generate measurements over the target
n = 6;                     % average number of measurements per scan-1
R = (0.1).^2;              % variance of measurement noise  
%% Three choices for the distribuion of measurements
[meas_all] = get_measurements(gt_state,gt_orientation,gt_length,gt_width,n,R,steps,'regionally concentrated measurements'); 
% [meas_all] = get_measurements(gt_state,gt_orientation,gt_length,gt_width,n,R,steps,'boundary concentrated measurements'); 
% [meas_all] = get_measurements(gt_state,gt_orientation,gt_length,gt_width,n,R,steps,'uniformly distributed measurements'); 

I2 = [1;0];

%%%%%% parameter setting of the tracking algorithm
L_evol = 50;               % the shape parameter in the dynamic model for the shape state L
var_s = 1/10;              % variance of the scaling factor s
lamda = 1.05;              % in measurement association, the gate of each region is lamda times that of the region

G_1 = [T^2/2;T]; 
G = blkdiag(G_1,G_1,1);
Q = diag([0.003,0.003,0.000003]);
Q_noise = G*Q*G.';
                  
        
%% intial estimate       
P_update = diag([0.25,0.01,0.25,0.01,0.0001]);     
X_update = [0;5;0;5;0.0001];
l_0_est = 2;   % intial estimated extension is a 2m x 2m square centered at the initial position of the key point
u1_update = 3;
L1_update = 2*(l_0_est/2).^2;
u2_update = 3;
L2_update = 2*(l_0_est/2).^2;
u3_update = 3;
L3_update = 2*(l_0_est/2).^2;
u4_update = 3;
L4_update = 2*(l_0_est/2).^2; 

figure(1);
for k  = 1:1:steps-1         
       k
       meas = meas_all{k};                 
       [X_pre,X_update,P_update,u1_update,L1_update,u2_update,L2_update,u3_update,L3_update,u4_update,L4_update] = KPA(T,X_update,P_update,Q_noise,L_evol,u1_update,L1_update,u2_update,L2_update,u3_update,L3_update,u4_update,L4_update,meas,var_s,R,lamda);               
            
      %% plot truth                  
      gt_pos = gt_state([1 3],k+1);
      gt_ori = gt_orientation(k+1);
      rotation = [cos(gt_ori),-sin(gt_ori);
                 sin(gt_ori),cos(gt_ori)];             
      P1 = rotation*[-gt_length/2;-gt_width/2]+gt_pos;                                                                           
      P2 = rotation*[gt_length/2;-gt_width/2]+gt_pos;                                                                           
      P3 = rotation*[-gt_length/2;gt_width/2]+gt_pos;                                                                           
      P4 = rotation*[gt_length/2;gt_width/2]+gt_pos;    
      truth = [P1,P2,P4,P3,P1];
      h1 = fill(truth(1,:),truth(2,:),'c');
      set(h1,'LineStyle','none');
      hold on
      %% plot measurements
      num_meas = size(meas,2);
      for j = 1:1:num_meas           
            h2 = plot(meas(1,j),meas(2,j),'LineStyle','none','Marker','o','MarkerSize',4,...
                  'MarkerEdgeColor','b','MarkerFaceColor','b','LineWidth',5); 
            hold on                  
       end
       hold on       
      %% plot estimate
      l1 = sqrt(L1_update/(u1_update-1));
      l2 = sqrt(L2_update/(u2_update-1));
      l3 = sqrt(L3_update/(u3_update-1)); 
      l4 = sqrt(L4_update/(u4_update-1));            
       
      vel_pre = X_pre([2 4]);
      if vel_pre(2)>=0                                                
           angular = acos(dot(vel_pre,I2)/norm(vel_pre));
      else
           angular = 2*pi-acos(dot(vel_pre,I2)/norm(vel_pre)); 
      end  
      rotation = [cos(angular),-sin(angular);
                   sin(angular),cos(angular)];
      corner1 = X_update([1 3])+rotation*[l1;l3];                                                  
      corner2 = X_update([1 3])+rotation*[-l2;l3];                                                 
      corner3 = X_update([1 3])+rotation*[-l2;-l4];                                                 
      corner4 = X_update([1 3])+rotation*[l1;-l4];                                                             
      vertices = [corner1,corner2,corner3,corner4,corner1];           
      h3 = plot(vertices(1,:),vertices(2,:),'k','LineWidth',2);
      hold on
end
legend([h1 h2 h3],'truth','measurement','estimate');
xlim([-50,900]);
ylim([-50,900]);
xlabel('x/m');
ylabel('y/m');
title('Estimation result in a single run');

%%% plot measurements accumulated in a run over the target         
figure(2);
rotation_1 = [cos(pi/2),-sin(pi/2);sin(pi/2),cos(pi/2)];  
accu_meas = [];
for k = 1:steps-1           
      meas = meas_all{k};  
      num_meas = size(meas,2);      
      gt_ori = gt_orientation(k+1);    
      rotation_2 = [cos(-gt_ori),-sin(-gt_ori);
                    sin(-gt_ori),cos(-gt_ori)];                                                      
      accu_meas = [accu_meas,rotation_1*rotation_2*[meas-gt_state([1 3],k+1)*ones(1,num_meas)]];                               
end
 P1 = [-gt_width/2;-gt_length/2];                                                                        
 P2 = [gt_width/2;-gt_length/2];                                                  
 P3 = [-gt_width/2;gt_length/2];                                                           
 P4 = [gt_width/2;gt_length/2];
 truth = [P1,P2,P4,P3,P1];
 h1 = fill(truth(1,:),truth(2,:),'c');
 set(h1,'LineStyle','none');
 hold on                                                           
num = size(accu_meas,2);
 for j = 1:1:num          
       h2 = plot(accu_meas(1,j),accu_meas(2,j),'LineStyle','none','Marker','o','MarkerSize',2,...
                  'MarkerEdgeColor',[0.25,0.41,0.88],'MarkerFaceColor',[0.25,0.41,0.88],'LineWidth',2); 
       hold on        
 end                
legend([h1 h2],'true extension','measurement');
xlim([-4,4]);
ylim([-4,4]);  
xlabel('x/m');
ylabel('y/m');
title('Accumulated measurements in a single run');
