function [meas_ass] = associate(meas_in_1and2,meas_in_2and3,meas_in_3and4,meas_in_1and4,meas_in_1and2and3and4) 
%% Implementation of the Key-Point-Based Extended Object Tracking Approach (KPA) with Regionally Concentrated Measurements 
% reference:
% "Tracking of Rectangular Object Using Key Points with Regionally Concentrated Measurements" 
% (IEEE Transactions on Intelligent Transportation Systems) by Xiaomeng Cao, Jian Lan*, Yushuang Liu, Boyi Tan, 2023.
% Copyright by Xiaomeng Cao, Shaanxi University of Science & Technology; Jian Lan, Xi'an Jiaotong University
% Email:  xmcao911@163.com; lanjian@mail.xjtu.edu.cn

% consider associations with regions if some measurements are inside multiple gates

% 'meas_in_1and2_to_1' denotes the measurements that are in 'meas_in_1and2' and are associated to region 1 in an association event
 
if size(meas_in_1and2, 2) == 0
    num_ass_events_1and2 = 1;
    meas_in_1and2_to_1 = cell(num_ass_events_1and2);
    meas_in_1and2_to_2 = cell(num_ass_events_1and2);
    meas_in_1and2_to_1{1} = [];
    meas_in_1and2_to_2{1} = [];
else
    num_ass_events_1and2 = 2;
    meas_in_1and2_to_1 = cell(num_ass_events_1and2);
    meas_in_1and2_to_2 = cell(num_ass_events_1and2);
    meas_in_1and2_to_1{1} = meas_in_1and2;
    meas_in_1and2_to_2{1} = [];
    meas_in_1and2_to_1{2} = [];
    meas_in_1and2_to_2{2} = meas_in_1and2;
end

if size(meas_in_2and3, 2) == 0
    num_ass_events_2and3 = 1;
    meas_in_2and3_to_2 = cell(num_ass_events_2and3);
    meas_in_2and3_to_3 = cell(num_ass_events_2and3);
    meas_in_2and3_to_2{1} = [];
    meas_in_2and3_to_3{1} = [];
else
    num_ass_events_2and3 = 2;
    meas_in_2and3_to_2 = cell(num_ass_events_2and3);
    meas_in_2and3_to_3 = cell(num_ass_events_2and3);
    meas_in_2and3_to_2{1} = meas_in_2and3;
    meas_in_2and3_to_3{1} = [];
    meas_in_2and3_to_2{2} = [];
    meas_in_2and3_to_3{2} = meas_in_2and3;
end    

if size(meas_in_3and4, 2) == 0
    num_ass_events_3and4 = 1;
    meas_in_3and4_to_3 = cell(num_ass_events_3and4);
    meas_in_3and4_to_4 = cell(num_ass_events_3and4);
    meas_in_3and4_to_3{1} = [];
    meas_in_3and4_to_4{1} = [];
else
    num_ass_events_3and4 = 2;
    meas_in_3and4_to_3 = cell(num_ass_events_3and4);
    meas_in_3and4_to_4 = cell(num_ass_events_3and4);
    meas_in_3and4_to_3{1} = meas_in_3and4;
    meas_in_3and4_to_4{1} = [];
    meas_in_3and4_to_3{2} = [];
    meas_in_3and4_to_4{2} = meas_in_3and4;
end
  

if size(meas_in_1and4, 2) == 0
    num_ass_events_1and4 = 1;
    meas_in_1and4_to_1 = cell(num_ass_events_1and4);
    meas_in_1and4_to_4 = cell(num_ass_events_1and4);
    meas_in_1and4_to_1{1} = [];
    meas_in_1and4_to_4{1} = [];
else
    num_ass_events_1and4 = 2;
    meas_in_1and4_to_1 = cell(num_ass_events_1and4);
    meas_in_1and4_to_4 = cell(num_ass_events_1and4);
    meas_in_1and4_to_1{1} = meas_in_1and4;
    meas_in_1and4_to_4{1} = [];
    meas_in_1and4_to_1{2} = [];
    meas_in_1and4_to_4{2} = meas_in_1and4;
end 

if size(meas_in_1and2and3and4, 2) == 0
    num_ass_events_1and2and3and4 = 1;  
    meas_in_1and2and3and4_to_1 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_2 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_3 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_4 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_1{1} = [];
    meas_in_1and2and3and4_to_2{1} = [];
    meas_in_1and2and3and4_to_3{1} = [];
    meas_in_1and2and3and4_to_4{1} = [];
else 
    num_ass_events_1and2and3and4 = 4;
    meas_in_1and2and3and4_to_1 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_2 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_3 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_4 = cell(num_ass_events_1and2and3and4);
    meas_in_1and2and3and4_to_1{1} = meas_in_1and2and3and4;
    meas_in_1and2and3and4_to_2{1} = [];
    meas_in_1and2and3and4_to_3{1} = [];
    meas_in_1and2and3and4_to_4{1} = [];
    meas_in_1and2and3and4_to_1{2} = [];
    meas_in_1and2and3and4_to_2{2} = meas_in_1and2and3and4;
    meas_in_1and2and3and4_to_3{2} = [];
    meas_in_1and2and3and4_to_4{2} = [];
    meas_in_1and2and3and4_to_1{3} = [];
    meas_in_1and2and3and4_to_2{3} = [];
    meas_in_1and2and3and4_to_3{3} = meas_in_1and2and3and4;
    meas_in_1and2and3and4_to_4{3} = [];
    meas_in_1and2and3and4_to_1{4} = [];
    meas_in_1and2and3and4_to_2{4} = [];
    meas_in_1and2and3and4_to_3{4} = [];
    meas_in_1and2and3and4_to_4{4} = meas_in_1and2and3and4;
end
meas_ass = cell(17);
meas_ass{1} = num_ass_events_1and2;
meas_ass{2} = meas_in_1and2_to_1;
meas_ass{3} = meas_in_1and2_to_2;
meas_ass{4} = num_ass_events_2and3;
meas_ass{5} = meas_in_2and3_to_2;
meas_ass{6} = meas_in_2and3_to_3;
meas_ass{7} = num_ass_events_3and4;
meas_ass{8} = meas_in_3and4_to_3;
meas_ass{9} = meas_in_3and4_to_4;
meas_ass{10} = num_ass_events_1and4;
meas_ass{11} = meas_in_1and4_to_1;
meas_ass{12} = meas_in_1and4_to_4;
meas_ass{13} = num_ass_events_1and2and3and4;
meas_ass{14} = meas_in_1and2and3and4_to_1;
meas_ass{15} = meas_in_1and2and3and4_to_2;
meas_ass{16} = meas_in_1and2and3and4_to_3;
meas_ass{17} = meas_in_1and2and3and4_to_4;
