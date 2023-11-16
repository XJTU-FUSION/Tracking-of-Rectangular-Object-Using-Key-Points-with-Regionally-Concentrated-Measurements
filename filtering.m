function [X_update,P_update,u1_update,L1_update,u2_update,L2_update,u3_update,L3_update,u4_update,L4_update,likelihood,n_likelihood] = filtering(X_pre,P_pre,bar_H,u1_pre,L1_pre,u2_pre,L2_pre,u3_pre,L3_pre,u4_pre,L4_pre,n1,n2,n3,n4,mean_meas_in_1and4_line1,mean_meas_in_2and3_line1,mean_meas_in_1and2_line2,mean_meas_in_3and4_line2,var_meas_in_1and4_line1,var_meas_in_2and3_line1,var_meas_in_1and2_line2,var_meas_in_3and4_line2,var_s,R)        
%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements 
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan*, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

%%%%%%% 
mean_L1_pre = L1_pre/(u1_pre-1);
mean_L2_pre = L2_pre/(u2_pre-1);
mean_L3_pre = L3_pre/(u3_pre-1);
mean_L4_pre = L4_pre/(u4_pre-1);
%%%%%%%

%%%%% extension update
if (n1+n4)~=0
    E_L1 = var_s+R/mean_L1_pre; 
    u1_update = u1_pre+(n1+n4)/2-0.5;   
    L1_update = var_meas_in_1and4_line1/(2*E_L1)+L1_pre;
else
    u1_update = u1_pre;           
    L1_update = L1_pre;
end

if (n2+n3)~=0
   E_L2 = var_s+R/mean_L2_pre; 
   u2_update = u2_pre+(n2+n3)/2-0.5;    
   L2_update = var_meas_in_2and3_line1/(2*E_L2)+L2_pre;
else
    u2_update = u2_pre;           
    L2_update = L2_pre;
end

if (n1+n2)~=0
   E_L3 = var_s+R/mean_L3_pre; 
   u3_update = u3_pre+(n1+n2)/2-0.5;    
   L3_update = var_meas_in_1and2_line2/(2*E_L3)+L3_pre;
else
    u3_update = u3_pre;           
    L3_update = L3_pre;
end

if (n3+n4)~=0
   E_L4 = var_s+R/mean_L4_pre; 
   u4_update = u4_pre+(n3+n4)/2-0.5;    
   L4_update = var_meas_in_3and4_line2/(2*E_L4)+L4_pre;
else
    u4_update = u4_pre;           
    L4_update = L4_pre;
end

%%%%% kinematic state update
H_stack = [bar_H(1,:);bar_H(1,:);bar_H(2,:);bar_H(2,:)];
bar_z = [mean_meas_in_1and4_line1;mean_meas_in_2and3_line1;mean_meas_in_1and2_line2;mean_meas_in_3and4_line2];
Y = [sqrt(mean_L1_pre);-sqrt(mean_L2_pre);sqrt(mean_L3_pre);-sqrt(mean_L4_pre)]/2;
U = [(var_s*mean_L1_pre+R)/(n1+n4),(var_s*mean_L2_pre+R)/(n2+n3),(var_s*mean_L3_pre+R)/(n1+n2),(var_s*mean_L4_pre+R)/(n3+n4)];

index = find([(n1+n4),(n2+n3),(n1+n2),(n3+n4)]==0);
H_stack(index,:) = []; 
U(index) = [];
bar_z(index) = [];
Y(index) = [];

S =H_stack*P_pre*H_stack.'+diag(U);    
residual = bar_z-H_stack*X_pre-Y;
K = P_pre*H_stack.'*inv(S);
X_update = X_pre+K*residual;

P_update = P_pre-K*S*K.';

%%%%%%%%%%%%%%%%% calculate the likelihood
if (n1+n4)>=1
    a1 = pi^((1-(n1+n4))/2);
    alpha1 = a1/sqrt(n1+n4);
else
    alpha1 = 1;
end

if (n2+n3)>=1
    a2 = pi^((1-(n2+n3))/2);
    alpha2 = a2/sqrt(n2+n3);
else
    alpha2 = 1;
end

if (n1+n2)>=1
    a3 = pi^((1-(n1+n2))/2);
    alpha3 = a3/sqrt(n1+n2);
else
    alpha3 = 1;
end

if (n3+n4)>=1
    a4 = pi^((1-(n3+n4))/2);
    alpha4 = a4/sqrt(n3+n4);
else
    alpha4 = 1;
end

if (n1+n4)>1
    b1 = L1_update^(-u1_update);
    b2 = gamma(u1_update);
    b3 = (2*E_L1)^((1-(n1+n4))/2);
    b4 = L1_pre^(u1_pre);
    b5 = (gamma(u1_pre))^(-1);
    beta1 = b1*b2*b3*b4*b5;
else
    beta1 = 1;
end

if (n2+n3)>1
    b1 = L2_update^(-u2_update);
    b2 = gamma(u2_update);
    b3 = (2*E_L2)^((1-(n2+n3))/2);
    b4 = L2_pre^(u2_pre);
    b5 = (gamma(u2_pre))^(-1);
    beta2 = b1*b2*b3*b4*b5;
else
    beta2 = 1;
end

if (n1+n2)>1
    b1 = L3_update^(-u3_update);
    b2 = gamma(u3_update);
    b3 = (2*E_L3)^((1-(n1+n2))/2);
    b4 = L3_pre^(u3_pre);
    b5 = (gamma(u3_pre))^(-1);
    beta3 = b1*b2*b3*b4*b5;
else
    beta3 = 1;
end

if (n3+n4)>1
    b1 = L4_update^(-u4_update);
    b2 = gamma(u4_update);
    b3 = (2*E_L4)^((1-(n3+n4))/2);
    b4 = L4_pre^(u4_pre);
    b5 = (gamma(u4_pre))^(-1);
    beta4 = b1*b2*b3*b4*b5;
else
    beta4 = 1;
end

Gu = sqrt(det(2*pi*S));
likelihood = alpha1*alpha2*alpha3*alpha4*beta1*beta2*beta3*beta4/Gu;
n_likelihood = residual.'*inv(S)*residual/2;
   
