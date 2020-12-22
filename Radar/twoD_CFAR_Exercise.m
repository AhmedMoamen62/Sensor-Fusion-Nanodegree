% Implement 1D CFAR using lagging cells on the given noise and target scenario.

% Close and delete all currently open figures
close all;

% Data_points
range = 1000;
doppler = 800;

% Generate random noise
s=abs(randn(doppler,range));

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s(100,100) = 8;
s(200,200) = 9;
s(300,300) = 4;
s(700,700) = 11;

%plot the output
figure,imagesc(s);

% TODO: Apply CFAR to detect the targets by filtering the noise.

% 1. Define the following:
% 1a. Training Cells
% 1b. Guard Cells 
Tr = 8;
Td = 4;
Gr = 4;
Gd = 2;

% Offset : Adding room above noise threshold for desired SNR 
offset=5;

% calculate the total size of training cells
total_size_of_training = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1) - (2*Gr+1)*(2*Gr+1);

% 2. Slide window across the signal length
for i = 1:(doppler-(Gd+Td))  
  for j = 1:(range-(Gr+Tr))   
      % 2. - 5. Determine the noise threshold by measuring it within the training cells
      sum_tr_lead = sum(sum(s(i:i+Td-1,j:j+Tr-1)));
      avg_tr = sum_tr_lead/total_size_of_training;
      threshold = offset * avg_tr;
      % 6. Measuring the signal within the CUT
      signal = s(i+Td+Gd,j+Tr+Gr);
      % 8. Filter the signal above the threshold
      if (signal > threshold)
        s(i+Td+Gd,j+Tr+Gr) = 1;
      else
        s(i+Td+Gd,j+Tr+Gr) = 0;
      end
  end
end

s(s != 0 || s != 1) = 0;
figure,imagesc(s);
