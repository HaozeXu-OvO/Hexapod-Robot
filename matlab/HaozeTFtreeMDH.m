
pi4 = 0.78539816325;
ll = 94.93715; lm = 50.8;l1 = 34; l2 = 43.51; l3 = 93.07;
% ??????
RM1=RevoluteMDH('a',0,'d',0,'alpha',0,'offset',-pi/2);
RM2=RevoluteMDH('a',lm,'d',0,'alpha',0,'offset',0);
RM3=RevoluteMDH('a',l1,'d',0,'alpha',pi/2,'offset',0);
RM4=RevoluteMDH('a',l2,'d',0,'alpha',0,'offset',-pi/2);
RM5=RevoluteMDH('a',l3,'d',0,'alpha',0,'offset',0);
% ?????
RF1=RevoluteMDH('a',0,'d',0,'alpha',0,'offset',-0.44372705);
RF2=RevoluteMDH('a',ll,'d',0,'alpha',0,'offset',-0.78539816325+0.44372705);
RF3=RevoluteMDH('a',l1,'d',0,'alpha',pi/2,'offset',0);
RF4=RevoluteMDH('a',l2,'d',0,'alpha',0,'offset',-pi/2);
RF5=RevoluteMDH('a',l3,'d',0,'alpha',0,'offset',0);

% ?????
LF1=RevoluteMDH('a',0,'d',0,'alpha',0,'offset',0.44372705);
LF2=RevoluteMDH('a',ll,'d',0,'alpha',0,'offset',0.78539816325-0.44372705);
LF3=RevoluteMDH('a',l1,'d',0,'alpha',pi/2,'offset',0);
LF4=RevoluteMDH('a',l2,'d',0,'alpha',0,'offset',-pi/2);
LF5=RevoluteMDH('a',l3,'d',0,'alpha',0,'offset',0);

% ?????
LM1=RevoluteMDH('a',0,'d',0,'alpha',0,'offset',pi/2);
LM2=RevoluteMDH('a',lm,'d',0,'alpha',0,'offset',0);
LM3=RevoluteMDH('a',l1,'d',0,'alpha',pi/2,'offset',0);
LM4=RevoluteMDH('a',l2,'d',0,'alpha',0,'offset',-pi/2);
LM5=RevoluteMDH('a',l3,'d',0,'alpha',0,'offset',0);

% ?????
LB1=RevoluteMDH('a',0,'d',0,'alpha',0,'offset',2.69786555);
LB2=RevoluteMDH('a',ll,'d',0,'alpha',0,'offset',-0.78539816325+0.44372705);
LB3=RevoluteMDH('a',l1,'d',0,'alpha',pi/2,'offset',0);
LB4=RevoluteMDH('a',l2,'d',0,'alpha',0,'offset',-pi/2);
LB5=RevoluteMDH('a',l3,'d',0,'alpha',0,'offset',0);

% ?????
RB1=RevoluteMDH('a',0,'d',0,'alpha',0,'offset',3.58531965);
RB2=RevoluteMDH('a',ll,'d',0,'alpha',0,'offset',0.78539816325-0.44372705);
RB3=RevoluteMDH('a',l1,'d',0,'alpha',pi/2,'offset',0);
RB4=RevoluteMDH('a',l2,'d',0,'alpha',0,'offset',-pi/2);
RB5=RevoluteMDH('a',l3,'d',0,'alpha',0,'offset',0);

% bot = SerialLink([L1 L2 L3 L4] , 'name', 'robot');
botRM = SerialLink([RM1 RM2 RM3 RM4 RM5] , 'name', 'robotRM');
botRF = SerialLink([RF1 RF2 RF3 RF4 RF5] , 'name', 'robotRF');
botLF = SerialLink([LF1 LF2 LF3 LF4 LF5] , 'name', 'robotLF');
botLM = SerialLink([LM1 LM2 LM3 LM4 LM5] , 'name', 'robotLM');
botLB = SerialLink([LB1 LB2 LB3 LB4 LB5] , 'name', 'robotLB');
botRB = SerialLink([RB1 RB2 RB3 RB4 RB5] , 'name', 'robotRB');
%????
platform=SerialLink([0 0 0 0],'name','platform','modified');%???????
platform.base=[1 0 0 0;
               0 1 0 0;
               0 0 1 0 ;
               0 0 0 1;]; %???????

% hold on;
% botRM.plot([0 0 0 0 0]);
% hold on;
% botRF.plot([0 0 0 0 0]);
% figure('name','robot','NumberTitle','off');

% rms.teach();
RMleg=SerialLink([platform,botRM],'name','Rm'); % ??????????????????
RFleg=SerialLink([platform,botRF],'name','Rf');
LFleg=SerialLink([platform,botLF],'name','Lf');
LMleg=SerialLink([platform,botLM],'name','Lm');
LBleg=SerialLink([platform,botLB],'name','Lb');
RBleg=SerialLink([platform,botRB],'name','Rb');

RMleg.plot([0 0 0 0 0 0])
hold on

RFleg.plot([0 0 0 0 0 0])
hold on

LFleg.plot([0 0 0 0 0 0])
hold on

LMleg.plot([0 0 0 0 0 0])
hold on

LBleg.plot([0 0 0 0 0 0])
hold on

RBleg.plot([0 0 0 0 0 0])
hold on
