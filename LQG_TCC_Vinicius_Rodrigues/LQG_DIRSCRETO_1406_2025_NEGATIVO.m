%%=========================================================================
%                     LQG - Linear-Quadrático-Gaussiano
% =========================================================================
% SCRIPT: LQG_Discreto_NEGATIVO.m
% AUTOR: Vinícius de Oliveira Bento Rodrigues
% DATA: 14/06/2025
%
% Este script executa o fluxo completo de modelagem e controle para os
% motores DC do robô, seguindo três etapas principais:
% Este script realiza a identificação de sistemas para os motores do robô
% a partir de dados experimentais e projeta os ganhos para um controlador
% LQG (Linear-Quadrático-Gaussiano) para três regiões de operação distintas
% =========================================================================
%% OBTER MODELOS E CONTROLADORES
clear
close all;  
clc;

% Carregar Dados
dados = load('Druffarel_Campo-negativo.txt');

amostras = 500;

for i = 1 : amostras   
    % Vetor de tempo
    t(i) = dados(i,2); 
    
    % Dados roda direita
    rpm_rd_30(i) = dados(i,5);
    rpm_rd_40(i) = dados(i+500,5);
    rpm_rd_50(i) = dados(i+1000,5);
    rpm_rd_60(i) = dados(i+1500,5);
    rpm_rd_70(i) = dados(i+2000,5);
    rpm_rd_80(i) = dados(i+2500,5);
    rpm_rd_90(i) = dados(i+3000,5);
    rpm_rd_100(i) = dados(i+3500,5);
    
    % Dados roda esquerda
    rpm_re_30(i) = dados(i,6);
    rpm_re_40(i) = dados(i+500,6);
    rpm_re_50(i) = dados(i+1000,6);
    rpm_re_60(i) = dados(i+1500,6);
    rpm_re_70(i) = dados(i+2000,6);
    rpm_re_80(i) = dados(i+2500,6);
    rpm_re_90(i) = dados(i+3000,6);
    rpm_re_100(i) = dados(i+3500,6);  
end

%% ========================================================================
% Obtem os modelos matemáticos (função de transferência) do motor direito.
% Converte os modelos para o formato de espaço de estados.
% =========================================================================
%                             MODELO RODA DIREITA
% =========================================================================

Ts = 0.02;
%============================== REGIAO 1 ==================================
tfz_rd_30 = EncontraTFz(abs(rpm_rd_30),30,t,Ts);
[numz_30,denz_30] = tfdata(tfz_rd_30,'v');

tfz_rd_40 = EncontraTFz(abs(rpm_rd_40),40,t,Ts);
[numz_40,denz_40] = tfdata(tfz_rd_40,'v');

tfz_rd_50 = EncontraTFz(abs(rpm_rd_50),50,t,Ts);
[numz_50,denz_50] = tfdata(tfz_rd_50,'v');

media_num_rd1 = [numz_30(1,2) numz_40(1,2) numz_50(1,2)];
media_den_rd1 = [denz_30(1,2) denz_40(1,2) denz_50(1,2)];

[A_rd1,B_rd1,C_rd1,D_rd1] = tf2ss(mean(media_num_rd1),[1 mean(media_den_rd1)]);


%============================== REGIAO 2 ==================================
tfz_rd_60 = EncontraTFz(abs(rpm_rd_60),60,t,Ts);
[numz_60,denz_60] = tfdata(tfz_rd_60,'v');

tfz_rd_70 = EncontraTFz(abs(rpm_rd_70),70,t,Ts);
[numz_70,denz_70] = tfdata(tfz_rd_70,'v');

tfz_rd_80 = EncontraTFz(abs(rpm_rd_80),80,t,Ts);
[numz_80,denz_80] = tfdata(tfz_rd_80,'v');

media_num_rd2 = [numz_60(1,2) numz_70(1,2) numz_80(1,2)];
media_den_rd2 = [denz_60(1,2) denz_70(1,2) denz_80(1,2)];

[A_rd2,B_rd2,C_rd2,D_rd2] = tf2ss(mean(media_num_rd2),[1 mean(media_den_rd2)]);


%============================== REGIAO 3 ==================================
tfz_rd_90 = EncontraTFz(abs(rpm_rd_90),90,t,Ts);
[numz_90,denz_90] = tfdata(tfz_rd_90,'v');

tfz_rd_100 = EncontraTFz(abs(rpm_rd_100),100,t,Ts);
[numz_100,denz_100] = tfdata(tfz_rd_100,'v');

media_num_rd3 = [numz_90(1,2) numz_100(1,2)];
media_den_rd3 = [denz_90(1,2) denz_100(1,2)];

[A_rd3,B_rd3,C_rd3,D_rd3] = tf2ss(mean(media_num_rd3),[1 mean(media_den_rd3)]);


%% ========================================================================
% Obtem os modelos matemáticos (função de transferência) do motor esquerdo.
% Converte os modelos para o formato de espaço de estados.
% =========================================================================
%                             MODELO RODA ESQUERDA
% =========================================================================

Ts = 0.02;
%============================== REGIAO 1 ==================================
tfz_re_30 = EncontraTFz(abs(rpm_re_30),30,t,Ts);
[numz_30,denz_30] = tfdata(tfz_re_30,'v');

tfz_re_40 = EncontraTFz(abs(rpm_re_40),40,t,Ts);
[numz_40,denz_40] = tfdata(tfz_re_40,'v');

tfz_re_50 = EncontraTFz(abs(rpm_re_50),50,t,Ts);
[numz_50,denz_50] = tfdata(tfz_re_50,'v');

media_num_re1 = [numz_30(1,2) numz_40(1,2) numz_50(1,2)];
media_den_re1 = [denz_30(1,2) denz_40(1,2) denz_50(1,2)];

[A_re1,B_re1,C_re1,D_re1] = tf2ss(mean(media_num_re1),[1 mean(media_den_re1)]);


%============================== REGIAO 2 ==================================
tfz_re_60 = EncontraTFz(abs(rpm_re_60),60,t,Ts);
[numz_60,denz_60] = tfdata(tfz_re_60,'v');

tfz_re_70 = EncontraTFz(abs(rpm_re_70),70,t,Ts);
[numz_70,denz_70] = tfdata(tfz_re_70,'v');

tfz_re_80 = EncontraTFz(abs(rpm_re_80),80,t,Ts);
[numz_80,denz_80] = tfdata(tfz_re_80,'v');

media_num_re2 = [numz_60(1,2) numz_70(1,2) numz_80(1,2)];
media_den_re2 = [denz_60(1,2) denz_70(1,2) denz_80(1,2)];

[A_re2,B_re2,C_re2,D_re2] = tf2ss(mean(media_num_re2),[1 mean(media_den_re2)]);


%============================== REGIAO 3 ==================================
tfz_re_90 = EncontraTFz(abs(rpm_re_90),90,t,Ts);
[numz_90,denz_90] = tfdata(tfz_re_90,'v');

tfz_re_100 = EncontraTFz(abs(rpm_re_100),100,t,Ts);
[numz_100,denz_100] = tfdata(tfz_re_100,'v');

media_num_re3 = [numz_90(1,2) numz_100(1,2)];
media_den_re3 = [denz_90(1,2) denz_100(1,2)];

[A_re3,B_re3,C_re3,D_re3] = tf2ss(mean(media_num_re3),[1 mean(media_den_re3)]);


%% ========================================================================
% Projeta e calcula os ganhos do controlador LQG (K e L) do motor direito.
% =========================================================================
%                             GANHOS RODA DIREITA
% =========================================================================

%%  Calcula ganhos K1 e Ki da PRIMEIRA região 30-50
% Expansão do modelo em espaço de estados por integrador (1/s)
% Ach = [A      0]      Bch = [B]      Cch = [0  C]
%       [-Ts*C  1]            [0]
Aex_rd1 = [A_rd1 0 ; -(Ts*C_rd1) 1];
Bex_rd1 = [B_rd1 ; 0];

% Matrizes de ponderação Q e R
% Q(1,1) penaliza o erro do estado original (velocidade).
% Q(2,2) penaliza o erro integral acumulado.
% R penaliza o esforço de controle u(k).
par1 = 0.5;% Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
par2 = 20; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
Q1 = [par1 0;0 par2];
R1 = 2; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

% A função 'idare' resolve a Equação Algébrica de Riccati para tempo discreto
% e retorna o ganho ótimo k = [K1, -Ki] para o sistema aumentado.
[xiz1,Kchz1] = idare(Aex_rd1, Bex_rd1, Q1, R1); 

K1_rd1 = Kchz1(1,1) % Ganho proporcional ao estado
Ki_rd1 = -Kchz1(1,2)% Ganho proporcional ao erro integral

%==========================================================================
%%  Calcula ganhos K1 e Ki da SEGUNDA região 60-80
% Expansão do modelo em espaço de estados por integrador (1/s)
% Ach = [A      0]      Bch = [B]      Cch = [0  C]
%       [-Ts*C  1]            [0]
Aex_rd2 = [A_rd2 0 ; -(Ts*C_rd2) 1];
Bex_rd2 = [B_rd2 ; 0];

% Matrizes de ponderação Q e R
par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
par2 = 12; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
Q2 = [par1 0;0 par2];
R2 = 2.5; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

%Teste final 1
%par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
%par2 = 15; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
%Q2 = [par1 0;0 par2];
%R2 = 2.5; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

%Teste final 2
%par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
%par2 = 12; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
%Q2 = [par1 0;0 par2];
%R2 = 2.5; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

[xiz2,Kchz2,Lchz2] = idare(Aex_rd2, Bex_rd2, Q2, R2); 

K1_rd2 = Kchz2(1,1) % Ganho proporcional ao estado
Ki_rd2 = -Kchz2(1,2)% Ganho proporcional ao erro integral

%==========================================================================
%%  Calcula ganhos K1 e Ki da TERCEIRA região 90 e 100
% Expansão do modelo em espaço de estados por integrador (1/s)
% Ach = [A      0]      Bch = [B]      Cch = [0  C]
%       [-Ts*C  1]            [0]
Aex_rd3 = [A_rd3 0 ; -(Ts*C_rd3) 1];
Bex_rd3 = [B_rd3 ; 0];

% Matrizes de ponderação Q e R
par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
par2 = 20; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
Q3 = [par1 0;0 par2];
R3 = 2.5; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

[xiz3,Kchz3,Lchz] = idare(Aex_rd3, Bex_rd3, Q3, R3); 

K1_rd3 = Kchz3(1,1) % Ganho proporcional ao estado
Ki_rd3 = -Kchz3(1,2) % Ganho proporcional ao erro integral

%% ========================================================================
% Projeta e calcula os ganhos do controlador LQG (K e L) do motor esquerdo.
% =========================================================================
%                             GANHOS RODA ESQUERDA
% =========================================================================
%%  Calcula ganhos K1 e Ki da PRIMEIRA região
% Expansão do modelo em espaço de estados por integrador (1/s)
% Ach = [A      0]      Bch = [B]      Cch = [0  C]
%       [-Ts*C  1]            [0]
Aex_re1 = [A_re1 0 ; -(Ts*C_re1) 1];
Bex_re1 = [B_re1 ; 0];

% Matrizes de ponderação Q e R
par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
par2 = 20; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
Q1 = [par1 0;0 par2];
R1 = 2; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

[xiz1,Kchz1,Lchz1] = idare(Aex_re1, Bex_re1, Q1, R1); 

K1_re1 = Kchz1(1,1) % Ganho proporcional ao estado
Ki_re1 = -Kchz1(1,2)% Ganho proporcional ao erro integral

%==========================================================================
%%  Calcula ganhos K1 e Ki da SEGUNDA região
% Expansão do modelo em espaço de estados por integrador (1/s)
% Ach = [A      0]      Bch = [B]      Cch = [0  C]
%       [-Ts*C  1]            [0]
Aex_re2 = [A_re2 0 ; -(Ts*C_re2) 1];
Bex_re2 = [B_re2 ; 0];

% Matrizes de ponderação Q e R
par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
par2 = 10; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
Q2 = [par1 0;0 par2];
R2 = 2.5; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

%Teste final 1
%par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
%par2 = 10; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
%Q2 = [par1 0;0 par2];
%R2 = 2; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

%Teste final 2
%par1 = 0.5; % Atua na retroação da velocidade, quanto mais alto o parâmetro maior o tempo de acomodação.
%par2 = 10; % Atua na integração, quanto maior o paramêtro mais rápido erro nulo em regime permanente.
%Q2 = [par1 0;0 par2];
%R2 = 2.5; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

[xiz2,Kchz2,Lchz2] = idare(Aex_re2, Bex_re2, Q2, R2); 

K1_re2 = Kchz2(1,1) % Ganho proporcional ao estado
Ki_re2 = -Kchz2(1,2)% Ganho proporcional ao erro integral

%==========================================================================
%%  Calcula ganhos K1 e Ki da TERCEIRA região
% Expansão do modelo em espaço de estados por integrador (1/s)
% Ach = [A      0]      Bch = [B]      Cch = [0  C]
%       [-Ts*C  1]            [0]
Aex_re3 = [A_re3 0 ; -(Ts*C_re3) 1];
Bex_re3 = [B_re3 ; 0];

% Matrizes de ponderação Q e R
par1 = 0.5; % parâmetro que mexe com retroação da velocidade, quanto maior mais lento o sistema fica
par2 = 13; % parâmetro que mexe com a integração, quanto maior mais rápido erro nulo em regime permanente
Q3 = [par1 0;0 par2];
R3 = 2; % Atua no esforço de controle, quanto maior mais lenta e amortecida a dinâmica do sistema.

[xiz3,Kchz3,Lchz3] = idare(Aex_re3, Bex_re3, Q3, R2); 

K1_re3 = Kchz3(1,1) % Ganho proporcional ao estado
Ki_re3 = -Kchz3(1,2)% Ganho proporcional ao erro integral


%==========================================================================
%% EXIBIÇÃO DOS GANHOS
k1rd = [K1_rd1 K1_rd2 K1_rd3]
kird = [Ki_rd1 Ki_rd2 Ki_rd3]

k1re = [K1_re1 K1_re2 K1_re3]
kire = [Ki_re1 Ki_re2 Ki_re3]

fprintf('float Ki_Dir_neg[3] = {%2.4f, %2.4f, %2.4f}; \n',abs(Ki_rd1),abs(Ki_rd2),abs(Ki_rd3));
fprintf('float K1_Dir_neg[3] = {%2.4f, %2.4f, %2.4f}; \n',abs(K1_rd1),abs(K1_rd2),abs(K1_rd3));

fprintf('\nfloat Ki_Esq_neg[3] = {%2.4f, %2.4f, %2.4f}; \n',abs(Ki_re1),abs(Ki_re2),abs(Ki_re3));
fprintf('float K1_Esq_neg[3] = {%2.4f, %2.4f, %2.4f}; \n',abs(K1_re1),abs(K1_re2),abs(K1_re3));
%% ========================================================================
% =========================================================================
%                          OBSERVADOR DE ESTADOS
% =========================================================================

% Estimador de Estados via Riccati
W = 10 * eye(1); % Covariância do ruído do processo (incertezas no modelo, distúrbios)
                 % Um W alto significa que confiamos menos no nosso modelo.
V = 1; % Covariância do ruído de medição (ruído do encoder)
       % Um V alto significa que confiamos menos na medição do sensor.

% RODA DIREITA
[yiz1,Ke_rd1] = idare(A_rd1',C_rd1, W, V); 

[yiz2,Ke_rd2] = idare(A_rd2', C_rd2, W, V); 

[yiz3,Ke_rd3] = idare(A_rd3', C_rd3, W, V); 

% RODA ESQUERDA
[yiz1,Ke_re1] = idare(A_re1', C_re1, W, V); 

[yiz2,Ke_re2] = idare(A_re2', C_re2, W, V); 

[yiz3,Ke_re3] = idare(A_re3', C_re3, W, V); 
