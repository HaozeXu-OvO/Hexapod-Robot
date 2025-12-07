clear all, close all
l1 = 34; l2 = 43.51; l3 = 93.07;
% 右中腿
RM(1)=Revolute('d', 0, 'a', 50.8, 'alpha', 0);
RM(2)=Revolute('d', 0, 'a', l1, 'alpha', pi/2);
RM(3)=Revolute('d', 0, 'a', l2, 'alpha', 0);
RM(4)=Revolute('d', 0, 'a', l3, 'alpha', 0);
RMAngleOffset=[-pi/2 0 0 -pi/2];

rmT=SerialLink(RM,'name','6DOF Manipulator Arm','offset',RMAngleOffset);

rmangs = [0 0 0 0];

% 左中腿
LM(1)=Revolute('d', 0, 'a', 50.8, 'alpha', 0);
LM(2)=Revolute('d', 0, 'a', l1, 'alpha', pi/2);
LM(3)=Revolute('d', 0, 'a', l2, 'alpha', 0);
LM(4)=Revolute('d', 0, 'a', l3, 'alpha', 0);
LMAngleOffset=[pi/2 0 0 -pi/2];

lmT=SerialLink(LM,'name','6DOF Manipulator Arm','offset',LMAngleOffset);

lmangs = [0 0 0 0];

% 右前腿
RF(1)=Revolute('d', 0, 'a', 94.93715, 'alpha', 0);
RF(2)=Revolute('d', 0, 'a', l1, 'alpha', pi/2);
RF(3)=Revolute('d', 0, 'a', l2, 'alpha', 0);
RF(4)=Revolute('d', 0, 'a', l3, 'alpha', 0);
RFAngleOffset=[-0.44372705 0 0 -pi/2];

rfT=SerialLink(RF,'name','6DOF Manipulator Arm','offset',RFAngleOffset);

rfangs = [0 0 0 0];

% 左前腿
LF(1)=Revolute('d', 0, 'a', 94.93715, 'alpha', 0);
LF(2)=Revolute('d', 0, 'a', l1, 'alpha', pi/2);
LF(3)=Revolute('d', 0, 'a', l2, 'alpha', 0);
LF(4)=Revolute('d', 0, 'a', l3, 'alpha', 0);
LFAngleOffset=[0.44372705 0 0 -pi/2];

lfT=SerialLink(LF,'name','LF leg','offset',LFAngleOffset);

lfangs = [0 0 0 0];

% 左后腿
LB(1)=Revolute('d', 0, 'a', 94.93715, 'alpha', 0);
LB(2)=Revolute('d', 0, 'a', l1, 'alpha', pi/2);
LB(3)=Revolute('d', 0, 'a', l2, 'alpha', 0);
LB(4)=Revolute('d', 0, 'a', l3, 'alpha', 0);
LBAngleOffset=[2.69786555 0 0 -pi/2];

lbT=SerialLink(LB,'name','LB leg','offset',LBAngleOffset);

lbangs = [0 0 0 0];

% 右后腿
RB(1)=Revolute('d', 0, 'a', 94.93715, 'alpha', 0);
RB(2)=Revolute('d', 0, 'a', l1, 'alpha', pi/2);
RB(3)=Revolute('d', 0, 'a', l2, 'alpha', 0);
RB(4)=Revolute('d', 0, 'a', l3, 'alpha', 0);
RBAngleOffset=[3.58531965 0 0 -pi/2];

rbT=SerialLink(RB,'name','RB leg','offset',RBAngleOffset);

rbangs = [0 0 0 0];

% Body
platform=SerialLink([0 0 0 0],'name','platform');%腰部关节
platform.base=[1 0 0 0;
               0 1 0 0;
               0 0 1 0 ;
               0 0 0 1;]; %基座高度


rms=SerialLink([platform,rmT],'name','rm');
lms=SerialLink([platform,lmT],'name','lm');
rfs=SerialLink([platform,rfT],'name','rf');
lfs=SerialLink([platform,lfT],'name','lf');
lbs=SerialLink([platform,lbT],'name','lb');
rbs=SerialLink([platform,rbT],'name','rb');

% 设置图像显示范围参数
view(3)
axis([-250, 250, -200, 200, -150, 150])
hold on

% 画图
rms.plot([0 0 0 0 0])
hold on

lms.plot([0 0 0 0 0])
hold on

rfs.plot([0 0 0 0 0])
hold on

lfs.plot([0 0 0 0 0])
hold on

lbs.plot([0 0 0 0 0])
hold on

rbs.plot([0 0 0 0 0])
hold on
