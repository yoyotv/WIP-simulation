
close all;clear all;clc;

p_s=zeros(2,10);
rw=zeros(2,10);
%===============================================
%initial setup
ts=0.005*2;
v=0;
w=0;
wc=0.05;
d_w=0;
d_theta=0;  
theta=0.2;

x=1.3;
y=-0.3;

xr=5;
yr=0;
    
delta=2*pi/3;
d_delta=0;
deltar=0;

wd=0.5*0;
vd=0.5*0;

error_w=0;
error_v=0; 
error_delta=0;
error_xv=0;

%===============================================

%===============================================
%WIP set
g=9.8;
Mw=0.03;
m=2;
r=0.04;
l=0.1;
j_w=3.17*(10^-5);
j_c=5;
j_theta=0.003;
j_delta=0.002;
D=0.17;
P=m*l^2+j_theta;
Q=2*Mw+2*j_w/(r^2)+m;


%===================================
%sliding function paraneter set
c1=2;
% c2=4;
c2=2;
c3=4;
c4=0.2;

%     sliding(1)=c1*error_delta+error_w;
%     sliding(2)=alphaa*(c2*error_xv+error_v)+(c3*theta+d_theta);
%===================================


%===================================
%position controller parameter set
k=5;
lamda1=2;
lamda2=1;
lamda3=1;
lamda4=1;
%===================================



%===================================
%CMAC parameter set
mean=[-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8;-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8];
var=ones(2,10)*5;
receptive=zeros(1,10);
inputmatrix=zeros(2,10);
weight=zeros(2,10)*0.001;
% weight=rand(3,10);
value=10;
uc=[0;0];
%===================================

S1=[];
S2=[];
TT=[];
W=[];
V=[];
Theta=[];
X=[];
Y=[];
XD=[];
YD=[];
EV=[];
EW=[];
ED=[];
ET=[];
S3=[];
S4=[];

TW=[];
TV=[];

for t=0:ts:20
    
       alpha=(D^2)*(2*Mw+j_w/(r^2))+j_delta;
       betta=P*Q-m^2*l^2*cos(theta)^2;
    
        %===============================================
    %reference trajectory
    p_xr=xr;
    p_yr=yr;
    p_deltar=deltar;

    dis=0.001*sin(t);

    

    %===============================================
    %obtain error_w,error_v
     p_error_w=error_w;
    
    error_w=wd-w;
    error_v=vd-v;


    error_xv=error_xv+error_v*ts;
    d_error_w=(error_w-p_error_w)/ts;
          %===============================================
    %obtain sliding function   
%     sliding(1)=c1*error_delta+error_w;

    sliding1=c1*error_w+d_error_w;
    sliding2=c2*error_xv+error_v;
    sliding3=c3*theta+d_theta;   
    
    sliding(1)=sliding1;
    sliding(2)=c4*sliding2+sliding3;

    %================================


   %adative laws=======================

% for i=1:2
%     delta_weight(i,:)=-0.01*sliding(i).*receptive(i,:);
% 
%    delta_mean(i,:)=-(0.01*sliding(i))*[(weight(i,:))].*[receptive(i,:)].*(2*(inputmatrix(i)-mean(i,:))./(var(i,:).^2));
%    delta_var(i,:)=-(0.01*sliding(i))*[(weight(i,:))].*[receptive(i,:)].*(2*(inputmatrix(i)-mean(i,:)).^2./(var(i,:).^3));
% 
% end

for i=1:2
    delta_weight(i,:)=-0.03*sliding(i).*receptive;

       delta_rw(i,:)=-(0.5*sliding(i))*[(weight(i,:))].*[receptive].*(2*(inputmatrix(i)-mean(i,:))./(var(i,:).^2)).*p_s(i,:);
   
    
   delta_mean(i,:)=-(5*sliding(i))*[(weight(i,:))].*[receptive].*(2*(inputmatrix(i)-mean(i,:))./(var(i,:).^2));
   delta_var(i,:)=-(5*sliding(i))*[(weight(i,:))].*[receptive].*(2*(inputmatrix(i)-mean(i,:)).^2./(var(i,:).^3));

   
   
end
    %update weight mean variance====================

    rw=rw+delta_rw;
    weight=weight+delta_weight;

    mean=mean+delta_mean;
    var=var+delta_var;

if t==10
    rw=1*ones(2,10)*6;
end
if t==10.5
    rw=1*ones(2,10)*5;
end
    %===========================================================================
    %controller
    %CMAC****************************************************************************************

    inputmatrix(1,:)=ones(1,value)*sliding(1)+rw(1,:).*p_s(1,:);
    inputmatrix(2,:)=ones(1,value)*sliding(2)+rw(2,:).*p_s(2,:);

    %sensory_cortex layer===========
    for j=1:2
    for i=1:value
    s(j,i)=exp(-((inputmatrix(j,i)-mean(j,i))^2)*(var(j,i)^-2));

    end
    end
    %==============
    %Receptive field
        %receptive=s;
    receptive=s(1,:).*s(2,:);
        uc(1)=receptive*weight(1,:)';
        uc(2)=receptive*weight(2,:)';

        %uc(1)=receptive(1,:)*weight(1,:)';
       % uc(2)=receptive(2,:)*weight(2,:)';

    %=====================================================================
    %===============================================
    %organize input of WIP system
    tu_w=uc(1);
    tu_v=uc(2);
    %===============================================
    %WIP system model

f_1=-(m*l^2*sin(theta*d_theta*d_delta))/(alpha+m*l^2*sin(theta)^2);
f_2=(1/(2*betta))*(-(m^2)*g*l^2*sin(2*theta)-m^2*l^3*sin(2*theta)*cos(theta*(d_delta)^2)+2*P*m*l*sin(theta*(d_theta^2)));
f_3=(1/(2*betta))*(-(m^2)*l^2*sin(2*theta*d_theta^2)+Q*m*l^2*sin(2*theta*d_delta^2)+2*Q*m*g*l*sin(theta));

g1=D/r*(alpha+m*l^2*sin(theta)^2);
g2=(P+m*l*r*cos(theta))/betta*r;
g3=-(Q*r+m*l*cos(theta))/(betta*r);
    
    

    d_w=f_1-g1*tu_w;
    d_v=f_2-g2*tu_v;
    dd_theta=f_3-g3*tu_v+1*dis;
    


    if t==10
    xx=0.4;
else
    xx=0;
end
    %================================
   w=w+d_w*ts;
   v=v+d_v*ts;
   d_theta=d_theta+dd_theta*ts;
   theta=theta+d_theta*ts+xx;
       %===============================================
    %trajectory planer module  (sensor) 



    %================================
    
    

    %===============================================
    %tracking error dynamics
    d_x=cos(delta)*v;
    d_y=sin(delta)*v;  
    d_delta=w;
    
    x=x+d_x*ts;
    y=y+d_y*ts;
    delta=delta+d_delta*ts;
    %================================
    
                   p_s(1,:)=s(1,:);
    p_s(2,:)=s(2,:);
    
   
   W=[W w];
   V=[V v];
   Theta=[Theta theta];
   X=[X x];
   Y=[Y y];
   XD=[XD xr];
   YD=[YD yr];

S1=[S1 sliding(1)];
S2=[S2 sliding(2)];
S3=[S3 sliding2];
S4=[S4 sliding3];

TW=[TW tu_w];
TV=[TV tu_v];

TT=[TT t];

EV=[EV error_v];
EW=[EW error_w];

ET=[ET theta];

end


figure(3)
plot(TT,S1);
legend('sliding surface1');

figure(4)
plot(TT,S2);
legend('sliding surface2');

figure(5)
plot(TT,EW);
legend('error_w');

figure(6)
plot(TT,EV);
legend('error_v');


figure(8)
plot(TT,ET);
legend('theta');


figure(9)
plot(TT,S3);
legend('sliding function2');

figure(10)
plot(TT,S4);
legend('sliding function3');

figure(11)
plot(TT,TW);
legend('w control signal');

figure(12)
plot(TT,TV);
legend('v control signal');