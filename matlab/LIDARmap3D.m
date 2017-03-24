function y = LIDARmap3D(filepath)



fileID = fopen(filepath,'r');


formatSpec = 'Distance:%f,step1:%f,step2:%f\n';
sizeA = [3 Inf];

M = fscanf(fileID,formatSpec,sizeA);
M = M';
R = M(:,1);
PHI = M(:,2);
TH = M(:,3);

R = R - 13.779252;g
PHI = PHI*((2*pi)/800.0);
TH = -TH*(((1.221730)/800.0)) + 4.25424;


RT = [];
THT = [];
PHIT = [];
for i = 1:1:length(R)
   if R(i) < 2500  
       RT = [RT;R(i)];
       THT = [THT;TH(i)];
       PHIT = [PHIT;PHI(i)];
   end
end
R = RT;
TH = THT;
PHI = PHIT;


X = R .* sin(TH) .* cos(PHI);
Y = R .* sin(TH) .* sin(PHI);
Z = R .* cos(TH);


C = (1*R)/mean(R);
scatter3(X,Y,Z,15,C,'filled');

y= PHI;





end