function y = LIDARmap(filepath)



fileID = fopen(filepath,'r');


formatSpec = 'Distance:%f,step:%f\n';
sizeA = [2 Inf];

M = fscanf(fileID,formatSpec,sizeA);
M = M';
R = M(:,1);
TH = M(:,2);

R = R - 13.779252;
TH = TH *((2*pi)/6400);


polarscatter(TH,R,'filled');

y= [R TH];





end