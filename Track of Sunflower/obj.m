function f=obj(x)
B=x(1);
C=x(2);
global A a
PA=[a 0 0];
%% 1��PC��ת
PCM=[0 (sqrt(3)/3)*a*cos(A) (sqrt(3)/3)*a*sin(A)];
PC=PCM/norm(PCM);%��λ����
PA1=PA*cos(B)+cross(PC,PA)*sin(-B)+PC*dot(PA,PC)*(1-cos(B));
%% 2��PE��ת
PEM=[a/2,-a*sqrt(3)/2,0];
PE=PEM/norm(PEM);
PAA=PA*cos(A)+cross(PE,PA)*sin(A)+PE*dot(PA,PE)*(1-cos(A));
%% 2��PD��ת
PDM=[a*0.5+a*0.5*cos(A) (sqrt(3)/6)*a*cos(A)-sqrt(3)*0.5*a (sqrt(3)/3)*a*sin(A)];
PD=PDM/norm(PDM);
PA2=PAA*cos(C)+cross(PD,PAA)*sin(-C)+PD*dot(PAA,PD)*(1-cos(C));
%% �������=0��������
f=norm(PA1-PA2);
end