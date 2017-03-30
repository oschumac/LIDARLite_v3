function y = LIDARtime(filepath)



fileID = fopen(filepath,'r');


formatSpec = '%f';
sizeA = [1 Inf];

M = fscanf(fileID,formatSpec,sizeA);
M = M';
M = M(M<0.1);
MEAN = mean(M);
STDEV = std(M);
SIZE = length(M);

fclose(fileID);

figure

histogram(M)

titletext = sprintf('%s Mean: %f STDEV: %f SIZE: %d',filepath,MEAN,STDEV,SIZE )

title(titletext)

xlabel('0.008 degree/s') % x-axis label
ylabel('Counts') % y-axis label
y = [MEAN , STDEV , SIZE];




end