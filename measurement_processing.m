function [meas_in_1,meas_in_2,meas_in_3,meas_in_4,meas_in_1and2,meas_in_2and3,meas_in_3and4,meas_in_1and4,meas_in_1and2and3and4,bar_H] = measurement_processing(X_pre,mean_L1_pre,mean_L2_pre,mean_L3_pre,mean_L4_pre,meas,lamda)
%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements 
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan*, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

%%%%%%%% Project measurements 
pre_vel = X_pre([2 4]);
I2 = [1;0];
if pre_vel(2)>=0
    orien_est = acos(dot(pre_vel,I2)/norm(pre_vel));  
else
    orien_est = 2*pi-acos(dot(pre_vel,I2)/norm(pre_vel));
end
gamma_inverse = [cos(-orien_est),-sin(-orien_est);
                 sin(-orien_est), cos(-orien_est)];
 
proj_meas = gamma_inverse*meas; % the projected measurements 

H = zeros(2,size(X_pre,1));
H(1,1) = 1;
H(2,3) = 1;
% H = [1,0,0,0,0;
%      0,0,1,0,0];
bar_H = gamma_inverse*H;                                                                 
%%%%%%%%%

meas_in_1 = [];
meas_in_2 = [];
meas_in_3 = [];
meas_in_4 = [];
meas_in_1and2 = [];    % 'meas_in_1and2' denotes the measurements that are inside the gates of region 1 and region 2
meas_in_3and4 = [];
meas_in_2and3 = [];
meas_in_1and4 = [];
meas_in_1and2and3and4 = [];

num_meas = size(meas,2);
for i = 1:1:num_meas  
    O_meas = gamma_inverse*(meas(:,i)-X_pre([1 3]));
    %%%%%%%% to determine which gate of the region meas(:,i) falls in
    judge1 = judge_inside_which_gate(mean_L1_pre,mean_L2_pre,mean_L3_pre,mean_L4_pre,O_meas,lamda,1);
    judge2 = judge_inside_which_gate(mean_L1_pre,mean_L2_pre,mean_L3_pre,mean_L4_pre,O_meas,lamda,2);    
    judge3 = judge_inside_which_gate(mean_L1_pre,mean_L2_pre,mean_L3_pre,mean_L4_pre,O_meas,lamda,3);  
    judge4 = judge_inside_which_gate(mean_L1_pre,mean_L2_pre,mean_L3_pre,mean_L4_pre,O_meas,lamda,4);    
    
    %%%%%%%%% inside a single gate of the region
    if judge1 == 1&judge2+judge3+judge4==0
        meas_in_1 = [meas_in_1,proj_meas(:,i)];
    end
    if judge2 == 1&judge1+judge3+judge4==0
        meas_in_2 = [meas_in_2,proj_meas(:,i)];
    end
    if judge3 == 1&judge1+judge2+judge4==0 
        meas_in_3 = [meas_in_3,proj_meas(:,i)];
    end
    if judge4 == 1&judge1+judge2+judge3==0 
        meas_in_4 = [meas_in_4,proj_meas(:,i)];
    end
    %%%%%%% inside two gates of regions
    if judge1 == 1&judge2==1&judge3+judge4==0
        meas_in_1and2 = [meas_in_1and2,proj_meas(:,i)];
    end
    if judge2 == 1&judge3==1&judge1+judge4==0
        meas_in_2and3 = [meas_in_2and3,proj_meas(:,i)];
    end        
    if judge3 == 1&judge4==1&judge1+judge2==0
        meas_in_3and4 = [meas_in_3and4,proj_meas(:,i)];
    end
    if judge1 == 1&judge4==1&judge2+judge3==0
        meas_in_1and4 = [meas_in_1and4,proj_meas(:,i)];
    end    
    %%%%%%% inside four gates of regions
    if judge1 == 1&judge2==1&judge3==1&judge4==1
        meas_in_1and2and3and4 = [meas_in_1and2and3and4,proj_meas(:,i)];
    end
    %%%%%%% inside no gate, associate to its nearest region
    if judge1+judge2+judge3+judge4==0
        p1 = [(sqrt(mean_L1_pre))/2;(sqrt(mean_L3_pre))/2];
        p2 = [-(sqrt(mean_L2_pre))/2;(sqrt(mean_L3_pre))/2];
        p3 = [-(sqrt(mean_L2_pre))/2;-(sqrt(mean_L4_pre))/2];
        p4 = [(sqrt(mean_L1_pre))/2;-(sqrt(mean_L4_pre))/2];
        [min_value,min_index] = min([norm(p1-O_meas),norm(p2-O_meas),norm(p3-O_meas),norm(p4-O_meas)]);
        if min_index == 1           
            meas_in_1 = [meas_in_1,proj_meas(:,i)];
        elseif min_index == 2              
            meas_in_2 = [meas_in_2,proj_meas(:,i)];
        elseif min_index == 3
            meas_in_3 = [meas_in_3,proj_meas(:,i)];
        else 
            meas_in_4 = [meas_in_4,proj_meas(:,i)];
        end
    end    
end

function judge = judge_inside_which_gate(mean_L1_pre,mean_L2_pre,mean_L3_pre,mean_L4_pre,measurement,lamda,index)
half_l1_pre = (sqrt(mean_L1_pre))/2;                                                          
half_l2_pre = (sqrt(mean_L2_pre))/2;                                                          
half_l3_pre = (sqrt(mean_L3_pre))/2;   
half_l4_pre = (sqrt(mean_L4_pre))/2; 
if index == 1
    point = [half_l1_pre;half_l3_pre];
    p1 = point+lamda*[-half_l1_pre;half_l3_pre];
    p2 = point+lamda*[-half_l1_pre;-half_l3_pre];
    p3 = point+lamda*[half_l1_pre;-half_l3_pre];
    p4 = point+lamda*[half_l1_pre;half_l3_pre];
elseif index == 2
    point = [-half_l2_pre;half_l3_pre];
    p1 = point+lamda*[-half_l2_pre;half_l3_pre];
    p2 = point+lamda*[-half_l2_pre;-half_l3_pre];
    p3 = point+lamda*[half_l2_pre;-half_l3_pre];
    p4 = point+lamda*[half_l2_pre;half_l3_pre];
elseif index == 3
    point = [-half_l2_pre;-half_l4_pre];
    p1 = point+lamda*[-half_l2_pre;half_l4_pre];
    p2 = point+lamda*[-half_l2_pre;-half_l4_pre];
    p3 = point+lamda*[half_l2_pre;-half_l4_pre];
    p4 = point+lamda*[half_l2_pre;half_l4_pre];
elseif index == 4
    point = [half_l1_pre;-half_l4_pre];
    p1 = point+lamda*[-half_l1_pre;half_l4_pre];
    p2 = point+lamda*[-half_l1_pre;-half_l4_pre];
    p3 = point+lamda*[half_l1_pre;-half_l4_pre];
    p4 = point+lamda*[half_l1_pre;half_l4_pre];
end
judge = judge_if_inside_rectangle(p1,p2,p3,p4,measurement);  


function judge = judge_if_inside_rectangle(p1,p2,p3,p4,measurement)
%% p1,p2,p3,p4 are the four vertexes of a rectangular region
A = cross([p4-p3;0],[measurement-p3;0]);
B = cross([p2-p1;0],[measurement-p1;0]);
C = A(3)*B(3);
D = cross([p3-p2;0],[measurement-p2;0]);
E = cross([p1-p4;0],[measurement-p4;0]);
F = D(3)*E(3);
if C>=0 & F>=0
    judge = 1;     %% measurement is inside the region
else
    judge = 0;     %% measurement is not inside the region
end
