
close all;clear all;clc;


p_s=zeros(2,10);
rw=zeros(2,10);

%===============================================
%initial setup
ts=0.01;

xv=0;
v=0;
w=0;
% 
% wc=0.05;
wc=pi/45;
d_w=0;
d_theta=0;  
theta=0.1;
% 
% x=1.3;
% y=-0.3;
% 
x=5.3;
y=-1.3;


% %x=4;
% y=0.5;

delta=0.6*pi;

xr=1;
yr=0;
    

d_delta=0;
deltar=pi/2;
wd=0;
vd=0.5;
xvd=0;


        error_w=0;
        error_v=0; 

        error_xv=0;

        p_vd=0;
        p_wd=0;

%===============================================

%===============================================
%WIP set
g=9.8;
Mw=0.03;
m=2;
r=0.04;
l=0.1;
j_w=3.17*(10^-5);
j_theta=0.003;
j_delta=0.002;
D=0.17;
P=m*l^2+j_theta;
Q=2*Mw+2*j_w/(r^2)+m;



%===================================
%sliding function paraneter set
% c1=2;
% c2=0.4;
% c3=4;
% c4=2;

sliding=[0;0];

c1=5;
c2=0.4;
c3=4;
c4=0.01;


k1=5;
k2=12;
k3=10;
k4=15;
v1=0;
v2=0;



% c4=-0.2;

%     sliding1=c1*error_w+d_error_w;
%     sliding2=c2*error_xv+error_v;
%     sliding3=c3*theta+d_theta;   
%     
%     sliding(1)=sliding1;
%     sliding(2)=c4*sliding2+sliding3;
%===================================


%===================================
%position controller parameter set

lamda1=1;
lamda2=1;
lamda3=20;
lamda4=5;


%===================================



%===================================
%===================================
%BELC parameter set
mean=[-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8;-1 -0.8 -0.6 -0.4 -0.2 0 0.2 0.4 0.6 0.8];
var=ones(2,10)*1;
receptive=zeros(1,10);
inputmatrix=zeros(2,10);
s=zeros(2,10);

    weight_a=zeros(2,10);
    weight_p=zeros(2,10);
% weight=rand(3,10);
value=10;
ub=[0;0];

weight=zeros(2,10)*0.001*0;
%===================================

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
EXV=[];

TW=[];
TV=[];

       alpha=(D^2)*(2*Mw+j_w/(r^2))+j_delta;
       betta=P*Q-m^2*l^2*cos(theta)^2;


f_1=-(m*l^2*sin(2*theta*d_theta*d_delta))/(alpha+m*l^2*sin(theta)^2);
f_2=(1/(2*betta))*(-(m^2)*g*l^2*sin(2*theta)-m^2*l^3*sin(2*theta)*cos(theta*(d_delta)^2)+2*P*m*l*sin(theta*(d_theta^2)));
f_3=(1/(2*betta))*(-(m^2)*l^2*sin(2*theta*d_theta^2)+Q*m*l^2*sin(2*theta*d_delta^2)+2*Q*m*g*l*sin(theta));

g1=D/(r*(alpha+m*l^2*sin(theta)^2));
g2=(P+m*l*r*cos(theta))/(betta*r);
g3=-(Q*r+m*l*cos(theta))/(betta*r);


    wr=0.5;
    vr=0.5;
    p_delta=0;
    
    z=0;
    
    xr=5;
    yr=0;
    
        p_xr=xr;
    p_yr=yr;
    
    error_x=xr-x;
    error_y=yr-y;
    error_delta=deltar-delta;
    
    sliding1=0;
    sliding2=0;
    sliding3=0;
        
for t=0:ts:60

    tu_d1=0.01*sin(0.5*t);
    tu_d2=0.01*cos(0.5*t);
    tu_d3=0.01*sin(0.5*t);

    
    
       alpha=(D^2)*(2*Mw+j_w/(r^2))+j_delta;
       betta=P*Q-m^2*l^2*cos(theta)^2;
       
    c4=0.2;
        %===============================================
    %reference trajectory

xr=5*cos(3*wc*t)*cos(wc*t);
yr=5*cos(3*wc*t)*sin(wc*t);



% 
    vr=abs(((xr-p_xr)^2+(yr-p_yr)^2)^0.5)/ts;
    wr=pi/45;
%     
    p_xr=xr;
    p_yr=yr;


        %=================================
    
    %obtain error_x,error_y,error_delta

    error_x=(xr-x)*cos(delta)+(yr-y)*sin(delta);
    error_y=(xr-x)*-sin(delta)+(yr-y)*cos(delta);
    
%     deltar=mod(deltar-(atan((xr-x)/(yr-y))),2*pi);
    deltar=mod(deltar+(wd)*ts,2*pi);
    error_delta=deltar-delta;
%        error_delta=-(atan((xr-x)/(yr-y)));
    %=================================
    

    %=================================
    %obtain desired trajectory

        wd=wr+2*lamda3*vr*error_y*cos(error_delta/2)+lamda4*sin(error_delta/2);
    vd=vr*cos(error_delta)+lamda1*tanh(w)*w*error_x-lamda1*tanh(w)*(vr*sin(error_delta)+lamda2*error_y)+lamda2*error_x-lamda1*(1-tanh(w)^2)*d_w*error_y;
         xvd=(error_x^2+error_y^2)^0.5;

    
%     wd=0;
%     vd=0;
%         xvd=0;
%         deltar=0;


    
    
    
    d_wd=(wd-p_wd)/ts;
    d_vd=(vd-p_vd)/ts;
    
    
    p_vd=vd;
    p_wd=wd;
    
     %===============================================
    %obtain error_w,error_v


    error_v=-(v-vd);
    error_xv=-(xv-xvd);
    

    error_w=-(w-wd);
          %===============================================
    %obtain sliding function   
    


    
    

    
    thick1=5;
    thick2=1;
    
    fy1=sliding3/thick1;
    fy2=sliding2/thick2;
    
    p_z=z;
    
    z=sat(fy2)*0.9;
    
    d_z=(z-p_z)/ts;

    sliding1=c1*error_delta+error_w;
    sliding2=c2*error_xv+error_v;
    sliding3=c3*(theta-z)+d_theta;   

    p_sliding(1)=sliding(1);
    
    sliding(1)=sliding1;
    sliding(2)=sliding3;

    
    
    %input=[error_w;error_v];
       input=[sliding(1);sliding(2)];
   %adative laws=======================

for i=1:2
 %   delta_weight_a(i,:)=-0.0001*input(i).*s(i,:);%(error converge much quickly as learning rate minuslearning)
%     delta_weight_p(i,:)=0.001*input(i).*s(i,:);
    
    delta_weight(i,:)=-0.00001*input(i).*receptive;

       delta_rw(i,:)=-(0.1*input(i))*[(weight(i,:))].*[receptive].*(2*(inputmatrix(i)-mean(i,:))./(var(i,:).^2)).*p_s(i,:);
   
   delta_mean(i,:)=-(0.1*100*input(i))*[(weight(i,:))].*[receptive].*(2*(inputmatrix(i)-mean(i,:))./(var(i,:).^2));
   delta_var(i,:)=-(0.1*100*input(i))*[(weight(i,:))].*[receptive].*(2*(inputmatrix(i)-mean(i,:)).^2./(var(i,:).^3));

end
    %update weight mean variance====================

    
%     weight_a=weight_a+delta_weight_a;
% %     weight_p=weight_p+delta_weight_p;

    rw=rw+delta_rw;
    weight=weight+delta_weight;

    mean=mean+delta_mean;
    var=var+delta_var;


    %===========================================================================
    %controller
    %CMAC****************************************************************************************

    
    inputmatrix(1,:)=ones(1,value)*input(1)+rw(1,:).*p_s(1,:);
    inputmatrix(2,:)=ones(1,value)*input(2)+rw(2,:).*p_s(2,:);
    
%     inputmatrix(1,:)=ones(1,value)*input(1);
%     inputmatrix(2,:)=ones(1,value)*input(2);

    %sensory_cortex layer===========
    for j=1:2
    for i=1:value
    %s(j,i)=exp(-((input(j)-mean(j,i))^2)*(var(j,i)^-2));

    s(j,i)=exp(-((inputmatrix(j,i)-mean(j,i))^2)*(var(j,i)^-2));

    end
    end
    %==============
    %Receptive field
    %receptive=s;
    receptive=s(1,:).*s(2,:);
    
            ub(1)=receptive*weight(1,:)';
        ub(2)=receptive*weight(2,:)';
%     ub(1)=s(1,:)*weight_a(1,:)'-0*s(1,:)*weight_p(1,:)';
%         ub(2)=s(2,:)*weight_a(2,:)'-0*s(2,:)*weight_p(2,:)';
    %===============================================
    %organize input of WIP system




% %     
%  tu_w=(-c1*error_w-d_wd+f_1-10*sat(sliding(1))-0*1*ub(1))/(-g1);
% tu_v=(-c3*d_theta+c3*d_z-f_3-50*sat(sliding(2))-0*1*ub(2))/(g3);

 tu_w=(-c1*error_w-d_wd+f_1-5*sat(sliding(1)))/(-g1)-1*(0.005*ub(1));
tu_v=(-c3*d_theta+c3*d_z-f_3-21*sat(sliding(2)))/(g3)-1*(0.005*ub(2));
%
% tu_v=c4*(c2*error_v+d_vd-f_2)+c3*-d_theta-f_3+50*sign(sliding(2))/(-(-g3-c4*g2));
    %===============================================
    %WIP system model
f_1=-(m*l^2*sin(2*theta*d_theta*d_delta))/(alpha+m*l^2*sin(theta)^2);
f_2=(1/(2*betta))*(-(m^2)*g*l^2*sin(2*theta)-m^2*l^3*sin(2*theta)*cos(theta*(d_delta)^2)+2*P*m*l*sin(theta*(d_theta^2)));
f_3=(1/(2*betta))*(-(m^2)*l^2*sin(2*theta*d_theta^2)+Q*m*l^2*sin(2*theta*d_delta^2)+2*Q*m*g*l*sin(theta));

g1=D/(r*(alpha+m*l^2*sin(theta)^2));
g2=(P+m*l*r*cos(theta))/(betta*r);
g3=-(Q*r+m*l*cos(theta))/(betta*r);
    
    

    d_w=f_1+g1*tu_w+1*tu_d1;
    d_v=f_2+g2*tu_v+1*tu_d2;
    dd_theta=f_3+g3*tu_v+1*tu_d3;
    
    %================================
    
    
    if t==30
        dis=1*750;
    else
        dis=0;
    end
    
   w=w+d_w*ts;
   v=v+d_v*ts+1*dis*ts;
   d_theta=d_theta+dd_theta*ts;
   theta=theta+d_theta*ts;
   
   delta=delta+ts*w;    
   delta=mod(delta,2*pi);
   xv=v*ts;
       %===============================================

    
    
    %===============================================
    %tracking error dynamics
    d_x=cos(delta)*v;
    d_y=sin(delta)*v;  
    d_delta=w;

   
    
    
    x=x+d_x*ts;
    y=y+d_y*ts;
%     delta=delta+d_delta*ts;

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

%EV=[EV error_v];
EV=[EV ub(1)];
%EW=[EW error_w];
EW=[EW ub(2)];
% EXV=[EXV error_xv];
 EXV=[EXV 10*sat(sliding(1))];

% ET=[ET theta];
 ET=[ET 50*sat(sliding(2))];

% 
% plot(x,y)
% hold on
% plot(xr,yr)

end

figure(1)
% plot(1.3,-0.3,'b*');
hold on
plot(X,Y,'r');
% hold off;
% 
% hold on

% figure(2)
plot(XD,YD)

hold off
xlabel('x')
ylabel('x')
legend('real','ref')



figure(5)
plot(TT,EW);
legend('error_w');

figure(6)
plot(TT,EV);
legend('error_v');

figure(7)
plot(TT,EXV);
legend('error_xv');


figure(8)
plot(TT,ET);
legend('theta');

figure(3)
plot(TT,S1);
legend('sliding surface1');

figure(4)
plot(TT,S2);
legend('sliding surface2');

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