clc
clear
global A a
a=1;
x0=[0,0];
for i=1:91
A=pi/2*(i-1)/90;
%% Start with the default options
options = optimset;
options = optimset(options,'TolFun', 10e-6);
options = optimset(options,'TolX', 10e-6);
%% Modify options setting
options = optimset(options,'Display', 'off');
[x,fval,exitflag,output] = fminsearch(@obj,x0,options);
result(i,1)=A;result(i,2)=x(1);result(i,3)=x(2);result(i,4)=fval;
x0=[x(1),x(2)];

%% 计算AB距离
B=x(1);C=x(2);
PA=[a 0 0];
%% A绕PC旋转
PCM=[0 (sqrt(3)/3)*a*cos(A) (sqrt(3)/3)*a*sin(A)];
PC=PCM/norm(PCM);%单位向量
PA1=PA*cos(B)+cross(PC,PA)*sin(-B)+PC*dot(PA,PC)*(1-cos(B));
%% PF绕PC旋转,旋转60
PF=[a 2*(sqrt(3)/3)*a*cos(A) 2*(sqrt(3)/3)*a*sin(A)];
PF1=PF*cos(B)+cross(PC,PF)*sin(-B)+PC*dot(PF,PC)*(1-cos(B));
% 以轮毂中心为坐标系旋转
PPF1=PF1+[a/2 a*sqrt(3)/2 0];
T=[cos(-pi/3) -sin(-pi/3) 0; sin(-pi/3) cos(-pi/3) 0;0 0 1];
PPB=T*PPF1';
PB=PPB'-[a/2 a*sqrt(3)/2 0];
result(i,5)=norm(PB-PA1);
end
subplot(1,2,1)
plot(result(:,1),result(:,2),'b',result(:,1),result(:,3),'r')
subplot(1,2,2)
plot(result(:,1),result(:,5))