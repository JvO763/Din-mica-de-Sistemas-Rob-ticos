%% Aula de exerc�cios - Cinem�tica Direta
% SEM0590

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Puma 560 - simb�lico

clc; 
clear; 
close all;

%Descri��o das vari�veis simb�licas

syms d2 a2 a3 d4 d6
syms q1 q2 q3 q4 q5 q6

% Par�metros de DH: theta d a alpha

DH = [0  0    0    -pi/2 ;
      0  d2   a2    0    ;
      0  0   -a3    pi/2 ;
      0  d4   0    -pi/2 ;
      0  0    0     pi/2 ;
      0  d6   0     0    ];    

% Cria��o dos links do rob�
% Formato da cria��o theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica

L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 0 0],'standard');
L(3) = Link([DH(3,:) 0 0],'standard'); 
L(4) = Link([DH(4,:) 0 0],'standard');
L(5) = Link([DH(5,:) 0 0],'standard');
L(6) = Link([DH(6,:) 0 0],'standard');

% Cria��o do modelo do rob�

q0 = [q1 q2 q3 q4 q5 q6];
p560 = SerialLink(L,'name','Puma560');

%C�lculo da cinem�tica direta

T = p560.fkine(q0);
A_01 = L(1).A(q1);
A_12 = L(2).A(q2);
A_23 = L(3).A(q3);
A_34 = L(4).A(q4);
A_45 = L(5).A(q5);
A_56 = L(6).A(q6);

A_06 = A_01*A_12*A_23*A_34*A_45*A_56;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Puma 560 - num�rico

% Par�metros de DH: theta d a alpha

DH = [0  0       0       -pi/2 ;
      0  0.15    0.4318  0     ;
      0  0      -0.0203  pi/2  ;
      0  0.4318  0       -pi/2 ;
      0  0       0       pi/2  ;
      0  0.056   0       0     ];    

% Cria��o dos links do rob�
% Formato da cria��o theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - prism�tica

L(1) = Link([DH(1,:) 0 pi/2],'standard'); 
L(2) = Link([DH(2,:) 0 0],'standard');
L(3) = Link([DH(3,:) 0 pi/2],'standard'); 
L(4) = Link([DH(4,:) 0 0],'standard');
L(5) = Link([DH(5,:) 0 0],'standard');
L(6) = Link([DH(6,:) 0 0],'standard');

% Definindo os limites das juntas

L(1).qlim = [-160 160]*pi/180;
L(2).qlim = [-225 45]*pi/180;
L(3).qlim = [-45 225]*pi/180;
L(4).qlim = [-110 170]*pi/180;
L(5).qlim = [-100 100]*pi/180;
L(6).qlim = [-266 266]*pi/180;

% Cria��o do modelo do rob�

q0 = [0 0 0 0 0 0];
p560 = SerialLink(L,'name','Puma560');

%Plot do rob�

p560.teach(q0)