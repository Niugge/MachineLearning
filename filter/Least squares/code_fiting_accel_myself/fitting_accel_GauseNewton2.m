%
%   @    Accel raw data fitting
%
%   @Algorithm  :   Least Squares   .(LM)
%
%   @use        Gause-Newton  
%   @use        Levenberg-Marquardt
%   @Author     Niu Hongfang
%   @Date       2019.05.24
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   measure:    axm, aym, azm           
%               up,down,left,right,front,back    6 faces
%   estimate:   ax,  ay,  az
%
%   relation between measure and estimation:
%                   |ax|    |a   b   c|   |ax + ox|
%                   |ay| =  |d   e   f| * |ay + oy|
%                   |az|    |g   h   i|   |az + oz|
%
%   accel satisfy:
%                   r^2 = ax^2 + ay^2 + az^2;       r=g or 1                
%   
%   measure redisual function:
%                   redisual = r - sqrt(ax^2 + ay^2 + az^2);
%   
%   define redisual matrix:
%                   Fi = [redisual0 redisua1 ... redisua5]';
%
%   define target funtion, has 6 measures (axm,aym,azm):
%                   F(a,b,c,d,e,f,ox,oy,oz) = redisual0^2 + redisual1^2 +
%                                             ... + redisua5^2;
%                   
%                   x  = [a,b,c,d,e,f,g,h,i,ox,oy,oz]';
%
%   Initialization  x0 = [1,0,0,0,1,0,0,0,1,0,0,0];
%
%   esimate parameter:
%                   a b c d e f g h i ox oy oz,   12   unknow  variable
%
%   Pi[1][9] is  a first order partial derivatives of redisual(a,b,c,d,e,f,g,h,i,ox,oy,oz).
%
%   J[6][9] is  Jacobian of Fi(a,b,c,d,e,f,g,h,i,ox,oy,oz).
%   
%   Levenberg-Marquardt: 
%                   lamda    :damping factor
%   Jacobian:       
%                                | P0[1][12] |
%                   J[6][12] =   | P1[1][12] |
%                                | ...      |
%                                | P5[1][12] |        |
%
%   define:
%                   JTJ[12][12]      =   J[6][12]'*J[6][12];
%                   JTJ_LM[12][12]   =   JTJ[12][12] + lamda*I[12][12];
%
%                   JTFi[12][1]   =   J[6][12]'*Fi[6][1];   
%
%   Desitination:
%                   beta[12][1]  = JTJ_LM[12][12]/ JTFi[12][1];
%                   x[12][1]     = x0[12][1] - beta[12][1];
%
%   Infinite iterations, make its approximate optimal solution.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clc;
clear all;
%close all;

ONE_G = 1.0;

PARAM_NUMBER = 12; 

JTJ = zeros(PARAM_NUMBER,PARAM_NUMBER);
JTFI = zeros(PARAM_NUMBER,1);
jacob = zeros(PARAM_NUMBER,1);

Accel_Data_Raw =  [-0.011308  -0.006936 -1.014725;...   %down
                    0.001049  -0.010036  1.009912;...   %up
                   -1.011354  -0.108833 -0.112941;...   %back
                    0.972251   0.046388  0.031283;...   %front
                   -0.027451   0.984027 -0.004315;...   %left
                   -0.058122   -1.009629 0.004295];     %right

[row,line] = size(Accel_Data_Raw);
fprintf("row = %d,line = %d\r\n", row, line);

M = @(x)    [x(1)   x(2)   x(3); ...
             x(4)   x(5)   x(6); ...
             x(7)   x(8)   x(9)];
         
param_offset = zeros(3,1);
param_M = [1,0,0,0,1,0,0,0,1]';

param = [param_M; param_offset];

fitness = 0;
fitness_f = 0;
max_iterations = 50;
num_iterations = 0;
eps = 1e-10;
lamda = 3;
lma_damping = 10;
E = eye(PARAM_NUMBER);

min_fitness = calc_mean_squared_residuals(Accel_Data_Raw, M(param_M), param_offset);
fprintf("Init fitness is %f \n", min_fitness);

while min_fitness > eps
    
    JTJ = zeros(PARAM_NUMBER,PARAM_NUMBER);
    JTFI = zeros(PARAM_NUMBER,1);
    
    for i=1:row
     
        sample = Accel_Data_Raw(i,:)';
        jacob = calc_jacob(sample, M(param_M), param_offset)';
    
        %fprintf("i = %d\n", i);
        JTJ = JTJ + jacob*jacob';
        JTFI = JTFI + jacob*calc_residual(sample,  M(param_M), param_offset);
    end
    
    JTJ = JTJ + lamda*E;
    
    detal = JTJ\JTFI;
    
    param_offset    = param_offset - detal(1:3);
    param_M         = param_M - detal(4:12);
    
    fitness = calc_mean_squared_residuals(Accel_Data_Raw,  M(param_M), param_offset);
    fprintf("Calcu fitness is %12f \n", fitness);
    
    if fitness >= min_fitness
        
        lamda = lamda * lma_damping;
        
    else
    
        lamda = lamda / lma_damping;
        min_fitness = fitness;
        param = [param_M; param_offset];
        
    end
    
    num_iterations = num_iterations + 1;
    fprintf("iteration conut: %d\n", num_iterations);
    if max_iterations <=  num_iterations 
        fprintf("error: iterations count more!\n");
        break;
    end
end
fprintf("\nparam: \n");

fprintf("\nThat is: = \n");
fprintf("\t\t| x |\t| %10f %10f %10f |\t| xm + %f |\n", param(1), param(2), param(3), param(10));
fprintf("\t\t| y | = | %10f %10f %10f | *  | xm + %f |\n", param(4), param(5), param(6), param(11));
fprintf("\t\t| z |\t| %10f %10f %10f |\t| xm + %f |\n", param(7), param(8), param(9), param(12));


contrast_curve_accle2(param);



 function fitness = calc_mean_squared_residuals(data, M, offset)
   
    sum = 0.0;
    row = size(data, 1);
    
    for i=1:row
        resid = calc_residual(data(i,:)',M, offset);
        sum = sum + power(resid,2);
    end
    fitness = sum / row;
 end
 
 function residual = calc_residual(sample, M, offset)

   residual = 1.0 - norm(M*(sample+offset)); 
   
 end
 
 function jacob = calc_jacob(sample, M, offset)
 
    estimate = M * (sample+offset);
    length = norm(estimate);
    
    A = estimate(1);
    B = estimate(2);
    C = estimate(3);
    
    x = sample(1);
    y = sample(2);
    z = sample(3);
    
    xo = offset(1);
    yo = offset(2);
    zo = offset(3);
    
    %1-3 offset
    jacob(1) = -1.0 * (((M(1,1) * A) + (M(1,2) * B) + (M(1,3) * C))/length);
    jacob(2) = -1.0 * (((M(2,1) * A) + (M(2,2) * B) + (M(2,3) * C))/length);
    jacob(3) = -1.0 * (((M(3,1) * A) + (M(3,2) * B) + (M(3,3) * C))/length);
     
    %4-12: M
    jacob(4) = -1.0 * ((x + xo) * A)/length;    %a
    jacob(5) = -1.0 * ((y + yo) * A)/length;    %b
    jacob(6) = -1.0 * ((z + zo) * A)/length;    %c
    jacob(7) = -1.0 * ((x + xo) * B)/length;    %d
    jacob(8) = -1.0 * ((y + yo) * B)/length;    %e
    jacob(9) = -1.0 * ((z + zo) * B)/length;    %f
    jacob(10)= -1.0 * ((x + xo) * C)/length;    %g
    jacob(11)= -1.0 * ((y + yo) * C)/length;    %h
    jacob(12)= -1.0 * ((z + zo) * C)/length;    %i
    
 end