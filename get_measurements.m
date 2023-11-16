function [meas_all] = get_measurements(gt_state,gt_orientation,gt_length,gt_width,n,R,steps,situation) 
%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements  
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan*, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

if strcmp(situation,'regionally concentrated measurements')==1
   k1 = 0.3;    % k1 and k2 are the proportion values to partition a rectangle into four regions
   k2 = 0.3;
   dect_prob = [0.3,0.4,0.2,0.1];   % the detection probabilities for different regions
end
if strcmp(situation,'boundary concentrated measurements')==1
   k1 = 0.01;
   k2 = 0.01;
   dect_prob = [0.3,0.01,0.3,0.39];
end
if strcmp(situation,'uniformly distributed measurements')==1
   k1 = 0.5;
   k2 = 0.5;
   dect_prob = [0.25,0.25,0.25,0.25];
end

meas_all = cell(steps-1);
for k = 1:1:steps-1
    num_meas = random('Poisson',n);              
    num_meas = num_meas+1;      % make sure the number of measurements per scan >=1
    meas_all{k} = zeros(2,num_meas);

    %%%%%%% get the vertexes of the rectangular target and the four regions at each time
    gt_pos = gt_state([1 3],k+1);
    gt_ori = gt_orientation(k+1);    
    rotation = [cos(gt_ori),-sin(gt_ori);
                 sin(gt_ori),cos(gt_ori)];   
    P1 = [-gt_length/2;-gt_width/2];                                                                          
    P2 = [gt_length/2;-gt_width/2];                                                                       
    P3 = [-gt_length/2;gt_width/2];                                                                           
    P4 = [gt_length/2;gt_width/2];  
    key_point = (0.5-k1)*(P3-P1)+(0.5-k2)*(P1-P2);   % the ground truth of the key point in the target¡¯s body coordinate system 

    %%% the four vertexes of the region 1
    A1 = key_point;                                                        
    B1 = P2*k1+P4*(1-k1);
    C1 = P4;
    D1 = P3*(1-k2)+P4*k2;
    %%% the four vertexes of the region 2
    A2 = P1*k1+P3*(1-k1);
    B2 = key_point;
    C2 = P3*(1-k2)+P4*k2;
    D2 = P3;
    %%% the four vertexes of the region 3
    A3 = P1;                      
    B3 = P1*(1-k2)+P2*k2;
    C3 = key_point;
    D3 = P1*k1+P3*(1-k1);
    %%% the four vertexes of the region 4
    A4 = P1*(1-k2)+P2*k2;
    B4 = P2;
    C4 = P2*k1+P4*(1-k1);
    D4 = key_point;
    %%%%%%%%%%%%
    
    sigma = rand(1,num_meas);     
    for j = 1:1:num_meas
        a = rand(1);
        b = rand(1);
        if sigma(j)<=dect_prob(1)            
            mea = b*(a*A1+(1-a)*B1)+(1-b)*(a*D1+(1-a)*C1);
        elseif sigma(j)>dect_prob(1)&sigma(j)<=dect_prob(1)+dect_prob(2)
            mea = b*(a*A2+(1-a)*B2)+(1-b)*(a*D2+(1-a)*C2);
        elseif sigma(j)>dect_prob(1)+dect_prob(2)&sigma(j)<=dect_prob(1)+dect_prob(2)+dect_prob(3)
            mea = b*(a*A3+(1-a)*B3)+(1-b)*(a*D3+(1-a)*C3);     
        else    
            mea = b*(a*A4+(1-a)*B4)+(1-b)*(a*D4+(1-a)*C4);   
        end
        meas_all{k}(:,j) = rotation*mea+gt_pos+sqrtm(R)*randn(2,1);
    end
end
