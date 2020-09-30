close all
clear all

% x = 0:0.001:1.5;
% num = x.^2;
% denom = x.^4;
% 
% y = num./denom;
% 
% figure;
% plot(x,y);
% 
% figure
% plot(x,num);
% hold on
% plot(x,denom);


% figure();
% hold on;
% 
% for n = 0.0001:0.0001:0.001
%     cov = n*eye(3);
%     covariance = log(det(cov));
%     
%     plot(n,covariance,'*');
%     
% end

clear all

W = [1,0,0;0,1,0;0,0,1];
E = [2.39217382251193e-05,5.21883754580822e-08,3.38468986580312e-07;5.21883754580881e-08,2.59465977272257e-07,4.10679495372692e-07;3.38468986580398e-07,4.10679495372535e-07,2.66347192047882e-06];

WE = diag(diag(E))
WE = W*WE

log(det(E))
log(det(WE))

