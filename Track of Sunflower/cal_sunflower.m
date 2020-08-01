clc
clear
tic
%{
----3-----------4--------5------
-----     -     -       --------
------       -   -     ---------
-------1-----------2(0,0,0)-----
--------         ---------------
----------    ------------------  
------------0-------------------
Definition:
Angle between Plane123 and Plane012 is Alpha (Given)
Angle between Plane123 and Plane234 is Beta (Calculated)
Angle between Plane234 and Plane245 is Gamma (Calculated Directly)

Geometric parameters:
D12: distance between Points 1 and 2
D13: distance between Points 1 and 3
D24: distance between Points 2 and 4
fi213: angle between Lines 12 and 13 in rigid Plane123 (obtuse angle)
fi421: angle between Lines 21 and 24 in initial plane Plane1234 (acute angle)
Angle: Rotation array angle pi/3
Constraint equation: 
the distance between Points 4 and 5 is constant
%}
D12=2;D13=3;D24=3;
fi213=120/180*pi;fi423=60/180*pi;
Angle=pi/3;
for i=1:180
Alpha=-i/180*pi;
InitBeta=[0/180*pi];
lb=[0];ub=[pi];%outer
% lb=[-pi];ub=[0];%inter
options = optimset;
options = optimset(options,'TolFun', 10e-6);
options = optimset(options,'TolX', 10e-6);
%% Modify options setting
options = optimset(options,'Display', 'off');
fun_Beta=@(Beta)obj_sunflower(D12,D13,D24,fi213,fi423,Angle,Alpha,Beta);
[x,fval,exitflag]=fmincon(fun_Beta,InitBeta,[],[],[],[],lb,ub,[],options);
Beta=x;
%% initial position 
P2=[0 0 0]';P1=[-D12,0,0]';P0=[-D12*cos(Angle),-D12*sin(Angle) 0]';
P3=[-D12+D13*cos(fi213) D13*sin(fi213) 0]';
P4=[-D24*cos(fi423) D24*sin(fi423) 0]';
R=[cos(-Angle) -sin(-Angle) 0; sin(-Angle) cos(-Angle) 0; 0 0 1];
P5=R*(P3-P0)+P0;

%% motion process
%Plane1234 rotates Alpha along Line21 (Alpha<0)
Vax=(P1-P2)/norm(P1-P2);
V23=(P3-P2);
V23New=V23*cos(Alpha)+cross(Vax,V23)*sin(Alpha)+Vax*dot(V23,Vax)*(1-cos(Alpha));
P3New=V23New+P2;
V24=(P4-P2);
V24New=V24*cos(Alpha)+cross(Vax,V24)*sin(Alpha)+Vax*dot(V24,Vax)*(1-cos(Alpha));

%Plane234 rotates Beta along Line23 
Vax=V23New/norm(V23New);
V24New2=V24New*cos(Beta)+cross(Vax,V24New)*sin(Beta)+Vax*dot(V24New,Vax)*(1-cos(Beta));
P4New=V24New2+P2;

%P5New (P3N rotates -Angle to P5New)
R=[cos(-Angle) -sin(-Angle) 0; sin(-Angle) cos(-Angle) 0; 0 0 1];
P5New=R*(P3New-P0)+P0;

%Angle between Plane 234 and Plane 245 Gammar
% normal vector
V25New=P5New-P2;
NV245=cross(V25New,V24New2);
NV234=cross(V24New2,V23New);
flag=dot(cross(NV234,NV245)/(norm(NV245)*norm(NV234)),V24New2);
if flag<0
    Gamma= -acos(dot(NV245,NV234)/(norm(NV245)*norm(NV234)));
else
    Gamma= acos(dot(NV245,NV234)/(norm(NV245)*norm(NV234)));
end

Radius=min(norm(P3New(1:end-1)-P0(1:end-1)),norm(P4New(1:end-1)-P0(1:end-1)));
Result(i,:)=[Alpha Beta Gamma P3New(1) P3New(2) P3New(3) P4New(1) P4New(2) P4New(3) Radius fval exitflag];
end
toc
%{
Result and Discussion
(1) There are two motion paths for the initial plane state. So there is a
bifurcation point for folding inward and outward.
In the program, the two paths can be calculated with different boundary
constraint.
(2) During the motion process, the rotational angle will be up to 180. For
the inward folding, the folding angle of valley crease 24 will be up to
-180. In contrast, that of mountain crease 23 will be up to 180. 
(3) Maybe for other parameters, there will be an intersect behavior for
Point4 and Plane 134'(in the next array).
(4) Interestingly, there will be another rigid region for the approximate
complete folding. it starts from the folding state with Alpha between
178-180 got inward folding. And the angle region for outward folding is
179-180, which can be discussed further.


%}