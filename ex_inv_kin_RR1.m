%% Aula de exerc�cios - Cinem�tica Inversa
% SEM0590

clc; clear; close all;

%% Cinemática (simbólico)

syms q1 q2 l1 l2 real

%      th    d    a    alpha
DH = [ 0     l1   0    pi/2 ;
       0     0    l2    0 ];
   
% Links
% theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica
L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 0 0],'standard');

% robot
q0 = [q1 q2];
rr_manip = SerialLink(L,'name','RR');

% matriz de transformação
disp('A_02 (lib) ='); A_02 = rr_manip.fkine(q0)   % direto da biblioteca
disp('A_01 ='); A_01 = L(1).A(q1)
disp('A_12 ='); A_12 = L(2).A(q2)

disp('A_02  ='); A_02 = A_01*A_12   % a partir das matrizes de cada elo

% inversa da matriz de transformação
disp('A_10 (inv) ='); A01_inv = simplify(inv(A_01))
[R,t] = tr2rt(A_01);
A01_inv = [R' -R'*t; 0 0 0 1]

%% Cinemática (numérica)

l1 = 1.0;
l2 = 2.0;

%      th    d    a    alpha
DH = [ 0     l1   0    pi/2 ;
       0     0    l2    0 ];
   
% Links
% theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica
L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 0 0],'standard');

q0 = [0 0]
rr_manip = SerialLink(L,'name','RR');
rr_manip.teach(q0)

% direta
q = [10 25]*pi/180
A_02 = rr_manip.fkine(q)
A_01 = L(1).A(q(1));
A_12 = L(2).A(q(2));
figure(); rr_manip.teach(q);

% inversa (feita manualmente)
q1_ik = atan2(A_02.t(2),A_02.t(1))
q2_ik = atan2(A_02.t(3)-l1,A_02.t(1)*cos(q1_ik)+A_02.t(2)*sin(q1_ik))
q_ik = [q1_ik q2_ik]

% https://www.petercorke.com/RTB/r9/html/SerialLink.html
q_ikv2 = rr_manip.ikine(A_02, [0 0], 'mask', [0 1 1 0 0 0])
