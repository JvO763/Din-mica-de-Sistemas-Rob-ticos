%% Aula de exercícios - Cinemática Inversa
% SEM0590

clc; clear; close all;

%% Cinemática (simbólico)

clc; clear; close all;

syms q1 q2 q3 t1 d2

%      th    d    a    alpha
DH = [ 0            100     0     pi/2 ;
       t1-pi/2      0       0     -pi/2 ;
       pi/2         d2      0     0];
   
% Links
% theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica
L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 0 0],'standard');
L(3) = Link([DH(3,:) 1 0],'standard'); % prismática

% Criação do modelo do robô
q0 = [q1 q2 q3];
acalibra = SerialLink(L,'name','AutoCalibraTruck');

% matriz de transformação
disp('A_03 (lib) ='); A_03 = acalibra.fkine(q0)   % direto da biblioteca
disp('A_01 ='); A_01 = L(1).A(q1)
disp('A_12 ='); A_12 = L(2).A(q2)
disp('A_23 ='); A_23 = L(3).A(q3)
disp('A_03  ='); A_03 = (A_01*A_12*A_23)   % a partir das matrizes de cada elo

syms nx ny nz sx sy sz ax ay az px py pz
A03_lit = [nx sx ax px;
           ny sy ay py;
           nz sz az pz;
           0  0  0  1];

% Inversa
disp('A_10 (inv) ='); A01_inv = simplify(inv(A_01))  % usando inv

[R,t] = tr2rt(A_01);
A01_inv = [R' -R'*t; 0 0 0 1]    % manualmente
simplify(A01_inv*A03_lit)
simplify(A_12*A_23)

%% CinemÃ¡tica (numÃ©rica)

%      th    d    a    alpha
DH = [ 0        100        0   pi/2 ;
       -pi/2    200        0   -pi/2 ;
       pi/2     30         0   0   ];
   
% Links
% theta d a alpha tipo offset
% tipo: 0 - rotativa, 1 - primatica
L(1) = Link([DH(1,:) 0 0],'standard'); 
L(2) = Link([DH(2,:) 0 0],'standard');
L(3) = Link([DH(3,:) 1 0],'standard'); % prismática


% Definindo os limites das juntas

L(1).qlim = [0 0]*pi/180;
L(2).qlim = [-170 170]*pi/180;
L(3).qlim = [40 200];

q0 = [0 pi/2 50];
acalibra = SerialLink(L,'name','AutoCalibraTruck');
figure(); acalibra.teach(q0);

% direta
q = [0 pi/2 50];
A_03 = acalibra.fkine(q)

[R,t] = tr2rt(A_03);
disp('t ='); disp(t')

% inversa (feita manualmente)
q1_ik = atan2(-A_03.o(1),A_03.o(2))
q2_ik = A_03.t(1)/(sin(q1_ik))
q_ik  = [q1_ik q2_ik]
% figure(); acalibra.plot(q_ik_v1*pi/180);

% Cinemática inversa numérica
q_ik_v3 = acalibra.ikine(A_03,[0 0 0],'mask',[0 180/pi 160 0 0 0])
