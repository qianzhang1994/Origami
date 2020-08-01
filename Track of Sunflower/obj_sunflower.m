function DisErr=obj_sunflower(D12,D13,D24,fi213,fi423,Angle,Alpha,Beta)
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

% D12=2;D13=3;D24=3;
% fi213=120/180*pi;fi423=60/180*pi;
% Angle=pi/3;
%% initial position 
P2=[0 0 0]';P1=[-D12,0,0]';P0=[-D12*cos(Angle),-D12*sin(Angle) 0]';
P3=[-D12+D13*cos(fi213) D13*sin(fi213) 0]';
P4=[-D24*cos(fi423) D24*sin(fi423) 0]';
R=[cos(-Angle) -sin(-Angle) 0; sin(-Angle) cos(-Angle) 0; 0 0 1];
P5=R*(P3-P0)+P0;

%% motion process
%Plane1234 rotates Alpha along Line21 (Alpha<0)
% Alpha=-10/180*pi;
Vax=(P1-P2)/norm(P1-P2);
V23=(P3-P2);
V23New=V23*cos(Alpha)+cross(Vax,V23)*sin(Alpha)+Vax*dot(V23,Vax)*(1-cos(Alpha));
P3New=V23New+P2;
V24=(P4-P2);
V24New=V24*cos(Alpha)+cross(Vax,V24)*sin(Alpha)+Vax*dot(V24,Vax)*(1-cos(Alpha));

%Plane234 rotates Beta along Line23 
% Beta=-10/180*pi;
Vax=V23New/norm(V23New);
V24New2=V24New*cos(Beta)+cross(Vax,V24New)*sin(Beta)+Vax*dot(V24New,Vax)*(1-cos(Beta));
P4New=V24New2+P2;

%P5New (P3N rotates -Angle to P5New)
R=[cos(-Angle) -sin(-Angle) 0; sin(-Angle) cos(-Angle) 0; 0 0 1];
P5New=R*(P3New-P0)+P0;

%% distance error between Points 4 and 5 for initial state and motion process
DisErr=abs(norm(P5-P4)-norm(P5New-P4New));

end