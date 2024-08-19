%% Aula de exercícios - Cinemática Direta
% SEM0590

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Puma 560 - simbólico

clc; 
clear; 
close all;

%Descrição das variáveis simbólicas

syms d2 a2 a3 d4 d6
syms q1 q2 q3 q4 q5 q6

% Parâmetros de DH: theta d a alpha

DH = [0  0    0    -pi/2 ;
      0  d2   a2    0    ;
      0  0   -a3    pi/2 ;
      0  d4   0    -pi/2 ;
      0  0    0     pi/2 ;
      0  d6   0     0    ];    

% Criação dos links do robô
% Formato da criação theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica

L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 0 0],'standard');
L(3) = Link([DH(3,:) 0 0],'standard'); 
L(4) = Link([DH(4,:) 0 0],'standard');
L(5) = Link([DH(5,:) 0 0],'standard');
L(6) = Link([DH(6,:) 0 0],'standard');

% Criação do modelo do robô

q0 = [q1 q2 q3 q4 q5 q6];
p560 = SerialLink(L,'name','Puma560');

%Cálculo da cinemática direta

T = p560.fkine(q0);
A_01 = L(1).A(q1);
A_12 = L(2).A(q2);
A_23 = L(3).A(q3);
A_34 = L(4).A(q4);
A_45 = L(5).A(q5);
A_56 = L(6).A(q6);

A_06 = A_01*A_12*A_23*A_34*A_45*A_56;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Puma 560 - numérico

% Parâmetros de DH: theta d a alpha

DH = [0  0       0       -pi/2 ;
      0  0.15    0.4318  0     ;
      0  0      -0.0203  pi/2  ;
      0  0.4318  0       -pi/2 ;
      0  0       0       pi/2  ;
      0  0.056   0       0     ];    

% Criação dos links do robô
% Formato da criação theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - prismática

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

% Criação do modelo do robô

q0 = [0 0 0 0 0 0];
p560 = SerialLink(L,'name','Puma560');

%Plot do robô

p560.teach(q0)