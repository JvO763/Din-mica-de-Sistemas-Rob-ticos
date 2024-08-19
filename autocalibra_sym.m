%% Aula de exerc�cios - Cinem�tica Direta
% SEM0590

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoCalibraTruck - simb�lico

clc; 
clear; 
close all;

%Descri��o das vari�veis simb�licas

syms d2 t1
syms q1 q2 

% Par�metros de DH: theta d a alpha

DH = [ (t1 - pi/2)     0      0     -pi/2;
       pi/2          d2     0     0 ];
   
% Cria��o dos links do rob�
% Formato da cria��o theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica

L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 1 0],'standard'); % prism�tica

% Cria��o do modelo do rob�

q0 = [q1 q2];
acalibra = SerialLink(L,'name','AutoCalibraTruck');

%C�lculo da cinem�tica direta

T = acalibra.fkine(q0);
A_01 = L(1).A(q1);
A_12 = L(2).A(q2);

A_02 = A_01*A_12;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% AutoCalibraTruck - num�rico

% Par�metros de DH: theta d a alpha

DH = [ t1 - pi/2     0      0     -pi/2;
       pi/2          0.2     0     0 ];
   
% Cria��o dos links do rob�
% Formato da cria��o theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - prism�tica

L(1) = Link([DH(1,:) 0 -pi/2],'standard'); 
L(2) = Link([DH(2,:) 1 0],'standard'); % prism�tica

% Definindo os limites das juntas

L(1).qlim = [-170 170]*pi/180;
L(2).qlim = [0 0.7];

% Cria��o do modelo do rob�

q = [pi/4 0.1];
acalibra = SerialLink(L,'name','AutoCalibraTruck');

%Plot do rob�

acalibra.teach(q)