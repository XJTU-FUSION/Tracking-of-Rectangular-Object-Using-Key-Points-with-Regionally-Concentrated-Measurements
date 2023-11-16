function [X_pre,X_update,P_update,u1_update,L1_update,u2_update,L2_update,u3_update,L3_update,u4_update,L4_update] = KPA(T,X_update,P_update,Q_noise,L_evol,u1_update,L1_update,u2_update,L2_update,u3_update,L3_update,u4_update,L4_update,meas,var_s,R,lamda)
%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements 
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan*, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

%%%%%%%%%%%%%%%%%% Prediction
% kinematic state
[X_pre,P_pre] = Kinematic_prediction(X_update,P_update,T);
P_pre = P_pre+Q_noise;                                              
% extension         
u1_pre = (L_evol*u1_update+2*u1_update-2)/(L_evol+u1_update-1);
L1_pre = (L_evol+1)*L1_update/(L_evol+u1_update-1);                       
u2_pre = (L_evol*u2_update+2*u2_update-2)/(L_evol+u2_update-1);
L2_pre = (L_evol+1)*L2_update/(L_evol+u2_update-1);            
u3_pre = (L_evol*u3_update+2*u3_update-2)/(L_evol+u3_update-1);
L3_pre = (L_evol+1)*L3_update/(L_evol+u3_update-1);
u4_pre = (L_evol*u4_update+2*u4_update-2)/(L_evol+u4_update-1);
L4_pre = (L_evol+1)*L4_update/(L_evol+u4_update-1);

%%%%%%% 
mean_L1_pre = L1_pre/(u1_pre-1);
mean_L2_pre = L2_pre/(u2_pre-1);
mean_L3_pre = L3_pre/(u3_pre-1);
mean_L4_pre = L4_pre/(u4_pre-1);
%%%%%%%

%%%%%%%%%%%%%%%%%%% associate measurements to different regions
[meas_in_1,meas_in_2,meas_in_3,meas_in_4,meas_in_1and2,meas_in_2and3,meas_in_3and4,meas_in_1and4,meas_in_1and2and3and4,bar_H] = measurement_processing(X_pre,mean_L1_pre,mean_L2_pre,mean_L3_pre,mean_L4_pre,meas,lamda);

% If some measurements are inside multiple gates, all possible associations with the regions need to be considered
[meas_ass] = associate(meas_in_1and2,meas_in_2and3,meas_in_3and4,meas_in_1and4,meas_in_1and2and3and4);

num_ass_events_1and2 = meas_ass{1};
meas_in_1and2_to_1 = meas_ass{2};
meas_in_1and2_to_2 = meas_ass{3};
num_ass_events_2and3 = meas_ass{4};
meas_in_2and3_to_2 = meas_ass{5};
meas_in_2and3_to_3 = meas_ass{6};
num_ass_events_3and4 = meas_ass{7};
meas_in_3and4_to_3 = meas_ass{8};
meas_in_3and4_to_4 = meas_ass{9};
num_ass_events_1and4 = meas_ass{10};
meas_in_1and4_to_1 = meas_ass{11};
meas_in_1and4_to_4 = meas_ass{12};
num_ass_events_1and2and3and4 = meas_ass{13};
meas_in_1and2and3and4_to_1 = meas_ass{14};
meas_in_1and2and3and4_to_2 = meas_ass{15};
meas_in_1and2and3and4_to_3 = meas_ass{16};
meas_in_1and2and3and4_to_4 = meas_ass{17};

%%%%%%%%%%%%%%%%%%
num_ass_events = num_ass_events_1and2*num_ass_events_2and3*num_ass_events_3and4*num_ass_events_1and4*num_ass_events_1and2and3and4;

X_update = zeros(5,num_ass_events);
P_update = zeros(5,5,num_ass_events);
u1_update = zeros(1,num_ass_events);
L1_update = zeros(1,num_ass_events);
u2_update = zeros(1,num_ass_events);
L2_update = zeros(1,num_ass_events);
u3_update = zeros(1,num_ass_events);
L3_update = zeros(1,num_ass_events);
u4_update = zeros(1,num_ass_events);
L4_update = zeros(1,num_ass_events);
likelihood = zeros(1,num_ass_events);
n_likelihood = zeros(1,num_ass_events);
%%%%%%%%%%%%%%%%%% filtering in each association event
i = 0;
for ii = 1:num_ass_events_1and2
   for jj = 1:num_ass_events_2and3
       for num = 1:num_ass_events_3and4
           for index = 1:num_ass_events_1and4  
              for numnum = 1:num_ass_events_1and2and3and4    
                i = i+1;
                % in the ith association event, the projected measurements associated to regions 1-4, respectively 
                meas_to_1 = [meas_in_1,meas_in_1and2_to_1{ii},meas_in_1and4_to_1{index},meas_in_1and2and3and4_to_1{numnum}];
                meas_to_2 = [meas_in_2,meas_in_1and2_to_2{ii},meas_in_2and3_to_2{jj},meas_in_1and2and3and4_to_2{numnum}];
                meas_to_3 = [meas_in_3,meas_in_2and3_to_3{jj},meas_in_3and4_to_3{num},meas_in_1and2and3and4_to_3{numnum}];
                meas_to_4 = [meas_in_4,meas_in_3and4_to_4{num},meas_in_1and4_to_4{index},meas_in_1and2and3and4_to_4{numnum}];  
                
                n1 = size(meas_to_1,2);
                n2 = size(meas_to_2,2);
                n3 = size(meas_to_3,2);
                n4 = size(meas_to_4,2);
      
                var_meas_to_1and2_line2 = 0;      
                var_meas_to_3and4_line2 = 0;
                var_meas_to_2and3_line1 = 0;
                var_meas_to_1and4_line1 = 0;
      
                if n1+n2>=1
                   meas_to_1and2 = [meas_to_1,meas_to_2];
                   mean_meas_to_1and2_line2 = mean(meas_to_1and2(2,:).');            
                   for j = 1:n1+n2
                      var_meas_to_1and2_line2 = var_meas_to_1and2_line2+(meas_to_1and2(2,j)-mean_meas_to_1and2_line2).^2;              
                   end
                else
                   mean_meas_to_1and2_line2 = 0;
                end                        
      
                if n3+n4>=1
                    meas_to_3and4 = [meas_to_3,meas_to_4];
                    mean_meas_to_3and4_line2 = mean(meas_to_3and4(2,:).');            
                    for j = 1:n3+n4
                       var_meas_to_3and4_line2 = var_meas_to_3and4_line2+(meas_to_3and4(2,j)-mean_meas_to_3and4_line2).^2;              
                    end
                else
                    mean_meas_to_3and4_line2 = 0;
                end    
      
                if n2+n3>=1
                   meas_to_2and3 = [meas_to_2,meas_to_3];
                   mean_meas_to_2and3_line1 = mean(meas_to_2and3(1,:).');            
                   for j = 1:n2+n3
                       var_meas_to_2and3_line1 = var_meas_to_2and3_line1+(meas_to_2and3(1,j)-mean_meas_to_2and3_line1).^2;              
                   end
                else
                    mean_meas_to_2and3_line1 = 0;
                end
      
                if n1+n4>=1
                    meas_to_1and4 = [meas_to_1,meas_to_4];
                    mean_meas_to_1and4_line1 = mean(meas_to_1and4(1,:).');            
                    for j = 1:n1+n4
                       var_meas_to_1and4_line1 = var_meas_to_1and4_line1+(meas_to_1and4(1,j)-mean_meas_to_1and4_line1).^2;              
                    end
               else
                    mean_meas_to_1and4_line1 = 0;
                end                
      
               [X_update(:,i),P_update(:,:,i),u1_update(i),L1_update(i),u2_update(i),L2_update(i),u3_update(i),L3_update(i),u4_update(i),L4_update(i),likelihood(i),n_likelihood(i)] = filtering(X_pre,P_pre,bar_H,u1_pre,L1_pre,u2_pre,L2_pre,u3_pre,L3_pre,u4_pre,L4_pre,n1,n2,n3,n4,mean_meas_to_1and4_line1,mean_meas_to_2and3_line1,mean_meas_to_1and2_line2,mean_meas_to_3and4_line2,var_meas_to_1and4_line1,var_meas_to_2and3_line1,var_meas_to_1and2_line2,var_meas_to_3and4_line2,var_s,R);
              end
           end
       end
   end
end

%%%%%%%% probabilities of association events
[max_value,max_index] = max(n_likelihood); 
Prob = zeros(1,num_ass_events);
sum_prob = 0;
for i = 1:num_ass_events
     Prob(i) = likelihood(i)*exp(-n_likelihood(i)+max_value);
     sum_prob = sum_prob+Prob(i);
end
Prob = Prob/sum_prob;

%%%%%%%% overall estimate for kinematic state      
X_update_sum = X_update*Prob.';
P_update_sum = zeros(5,5);
for i = 1:1:num_ass_events  
      P_update_sum = P_update_sum+Prob(i)*(P_update(:,:,i)+(X_update(:,i)-X_update_sum)*(X_update(:,i)-X_update_sum).');
end
X_update = X_update_sum;
P_update = P_update_sum;

%%%%%%%% overall estimate for extension state
mean_L1_update = 0;
var_L1_update = 0;
mean_L2_update = 0;
var_L2_update = 0;
mean_L3_update = 0;
var_L3_update = 0;
mean_L4_update = 0;
var_L4_update = 0;
for i = 1:1:num_ass_events  
   mean_L1_update = mean_L1_update+Prob(i)*L1_update(i)/(u1_update(i)-1);
   mean_L2_update = mean_L2_update+Prob(i)*L2_update(i)/(u2_update(i)-1);
   mean_L3_update = mean_L3_update+Prob(i)*L3_update(i)/(u3_update(i)-1);
   mean_L4_update = mean_L4_update+Prob(i)*L4_update(i)/(u4_update(i)-1);
end

for i = 1:1:num_ass_events 
   add_L1 = L1_update(i)*L1_update(i)/((u1_update(i)-1)*(u1_update(i)-1)*(u1_update(i)-2));
   var_L1_update = var_L1_update+Prob(i)*(add_L1+(L1_update(i)/(u1_update(i)-1)-mean_L1_update)^2);
   add_L2 = L2_update(i)*L2_update(i)/((u2_update(i)-1)*(u2_update(i)-1)*(u2_update(i)-2));
   var_L2_update = var_L2_update+Prob(i)*(add_L2+(L2_update(i)/(u2_update(i)-1)-mean_L2_update)^2);
   add_L3 = L3_update(i)*L3_update(i)/((u3_update(i)-1)*(u3_update(i)-1)*(u3_update(i)-2));
   var_L3_update = var_L3_update+Prob(i)*(add_L3+(L3_update(i)/(u3_update(i)-1)-mean_L3_update)^2);
   add_L4 = L4_update(i)*L4_update(i)/((u4_update(i)-1)*(u4_update(i)-1)*(u4_update(i)-2));
   var_L4_update = var_L4_update+Prob(i)*(add_L4+(L4_update(i)/(u4_update(i)-1)-mean_L4_update)^2);
end

u1_update = ((mean_L1_update^2)/var_L1_update)+2;
L1_update = mean_L1_update*(u1_update-1);
u2_update = ((mean_L2_update^2)/var_L2_update)+2;
L2_update = mean_L2_update*(u2_update-1);
u3_update = ((mean_L3_update^2)/var_L3_update)+2;
L3_update = mean_L3_update*(u3_update-1);
u4_update = ((mean_L4_update^2)/var_L4_update)+2;
L4_update = mean_L4_update*(u4_update-1);
            
