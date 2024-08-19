%% Aula de exercícios - Cinemática Direta
% SEM0590

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoCalibraTruck - simbólico

clc; 
clear; 
close all;

%Descrição das variáveis simbólicas

syms d2 t1
syms q1 q2 

% Parâmetros de DH: theta d a alpha

DH = [ (t1 - pi/2)     0      0     -pi/2;
       pi/2          d2     0     0 ];
   
% Criação dos links do robô
% Formato da criação theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica

L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 1 0],'standard'); % prismática

% Criação do modelo do robô

q0 = [q1 q2];
acalibra = SerialLink(L,'name','AutoCalibraTruck');

%Cálculo da cinemática direta

T = acalibra.fkine(q0);
A_01 = L(1).A(q1);
A_12 = L(2).A(q2);

A_02 = A_01*A_12;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoCalibraTruck - numérico

% Parâmetros de DH: theta d a alpha

DH = [ t1 - pi/2     0      0     -pi/2;
       pi/2          0.2     0     0 ];
   
% Criação dos links do robô
% Formato da criação theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - prismática

L(1) = Link([DH(1,:) 0 -pi/2],'standard'); 
L(2) = Link([DH(2,:) 1 0],'standard'); % prismática

% Definindo os limites das juntas

L(1).qlim = [-170 170]*pi/180;
L(2).qlim = [0 0.7];

% Criação do modelo do robô

q = [pi/4 0.1];
acalibra = SerialLink(L,'name','AutoCalibraTruck');

%Plot do robô

acalibra.teach(q)