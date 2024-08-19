%% Aula de exerc�cios - Cinem�tica Direta
% SEM0590

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Stanford - simb�lico

clc; 
clear; 
close all;

%Descri��o das vari�veis simb�licas

syms t1 d2
syms q1 q2

% Par�metros de DH: theta d a alpha

DH = [ t1      0   0      -pi/2 ;
       pi/2         d2    0   0];
   
% Cria��o dos links do rob�
% Formato da cria��o theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica

L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 1 0],'standard'); % prism�tica

% Cria��o do modelo do rob�

q0 = [q1 q2];
autocalibra = SerialLink(L,'name','AutoCalibra');

%C�lculo da cinem�tica direta

disp('T (lib) ='); T = autocalibra.fkine(q0);   % direto da biblioteca
disp('A_01 ='); A_01 = L(1).A(q1)
disp('A_12 ='); A_12 = L(2).A(q2)
disp('A_02  ='); A_02 = A_01*A_12   % a partir das matrizes de cada elo

% inversa da matriz de transformação
disp('A_10 (inv) ='); A01_inv = simplify(inv(A_01))
[R,t] = tr2rt(A_01);
A01_inv = [R' -R'*t; 0 0 0 1]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Stanford - num�rico

% Par�metros de DH: theta d a alpha

DH = [-pi/2     0        0   -pi/2 ;
       pi/2     200       0    0   ];
   
% Cria��o dos links do rob�
% Formato da cria��o theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - prism�tica

L(1) = Link([DH(1,:) 0 pi/2],'standard');
L(2) = Link([DH(2,:) 1 0],'standard'); % prism�tica

% Definindo os limites das juntas

L(1).qlim = [-170 170]*pi/180;
L(2).qlim = [40 200];

% Cria��o do modelo do rob�

q = [pi/4 45];
autocalibra = SerialLink(L,'name','AutoCalibraTruck');

%Plot do rob�

autocalibra.teach(q)

% direta
q = [pi/4 45]
A_02 = autocalibra.fkine(q)
A_01 = L(1).A(q(1));
A_12 = L(2).A(q(2));
figure(); autocalibra.teach(q);

% inversa (feita manualmente)
%noat
q1_ik = atan2(-A_02.o(1),A_02.o(2))
q2_ik = A_02.t(1)/(sin(q1_ik))
q_ik = [q1_ik q2_ik]

% https://www.petercorke.com/RTB/r9/html/SerialLink.html
q_ikv2 = autocalibra.ikine(A_02, [0 0], 'mask', [0 1 1 0 0 0])

syms q1;
A_02 = [0, -sin(q1), cos(q1), q2*cos(q1);
 0, cos(q1),  sin(q1),  -q2*sin(q1);
-1,        0,        0,           0;
 0,        0,        0,           1]

