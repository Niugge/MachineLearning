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
%                   |ax|    |a   d   e|   |ax + ox|
%                   |ay| =  |d   b   f| * |ay + oy|
%                   |az|    |e   f   c|   |az + oz|
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
%                   x  = [a,b,c,d,e,f,ox,oy,oz]';
%
%   Initialization  x0 = [1,1,1,0,0,0,0,0,0];
%
%   esimate parameter:
%                   a b c d e f ox oy oz,   9   unknow  variable
%
%   Pi[1][9] is  a first order partial derivatives of redisual(a,b,c,d,e,f,ox,oy,oz).
%
%   J[6][9] is  Jacobian of Fi(a,b,c,d,e,f,ox,oy,oz).
%   
%   Levenberg-Marquardt: 
%                   lamda    :  damping factor
%   Jacobian:       
%                               | P0[1][9] |
%                   J[6][9] =   | P1[1][9] |
%                               | ...      |
%                               | P5[1][9] |        |
%
%   define:
%                   JTJ[9][9]       =   J[6][9]' * J[6][9];
%                   JTJ_LM[9][9]    =   JTJ[9][9] + lamda * I[9][9];
%
%                   JTFi[9][1]   =   J[6][9]' * Fi[6][1];   
%
%   Desitination:
%                   beta[9][1]  = JTJ_LM[9][9] / JTFi[9][1];
%                   x[9][1]     = x0[9][1] - beta[9][1];
%
%   Infinite iterations, make its approximate optimal solution.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
clc;
clear all;
%close all;

ONE_G = 1.0;

JTJ = zeros(9,9);
JTFI = zeros(9,1);
jacob = zeros(9,1);

Accel_Data_Raw =  [-0.011308  -0.006936 -1.014725;...   %down
                    0.001049  -0.010036  1.009912;...   %up
                   -1.011354  -0.108833 -0.112941;...   %back
                    0.972251   0.046388  0.031283;...   %front
                   -0.027451   0.984027 -0.004315;...   %left
                   -0.058122   -1.009629 0.004295];     %right

[row,line] = size(Accel_Data_Raw);
fprintf("row = %d,line = %d\r\n", row, line);
%  M = [1,0,0;0,1,0;0,0,1];
%  offset = [0;0;0];
diag = ones(3,1);
offdiag = zeros(3,1);
offset = zeros(3,1);
 
param = [offset, diag, offdiag];

fitness = 0;
fitness_f = 0;
max_iterations = 50;
num_iterations = 0;
eps = 1e-10;
lamda = 3;
lma_damping = 10;
E = eye(9);

min_fitness = calc_mean_squared_residuals(Accel_Data_Raw, diag, offdiag, offset);
fprintf("Init fitness is %f \n", min_fitness);

while min_fitness > eps
    
    JTJ = zeros(9,9);
    JTFI = zeros(9,1);
    
    for i=1:row
     
        sample = Accel_Data_Raw(i,:)';
        jacob = calc_jacob(sample, diag, offdiag, offset)';
    
        %fprintf("i = %d\n", i);
        JTJ = JTJ + jacob*jacob';
        JTFI = JTFI + jacob*calc_residual(sample, diag, offdiag, offset);
    end
    
    JTJ = JTJ + lamda*E;
    
    detal = JTJ\JTFI;
    
    offset  = offset    - detal(1:3);
    diag    = diag      - detal(4:6);
    offdiag = offdiag   - detal(7:9);
    
    fitness = calc_mean_squared_residuals(Accel_Data_Raw, diag, offdiag, offset);
    fprintf("Calcu fitness is %12f \n", fitness);
    
    if fitness >= min_fitness
        
        lamda = lamda * lma_damping;
        
    else
    
        lamda = lamda / lma_damping;
        min_fitness = fitness;
        param = [offset', diag', offdiag'];
        
    end
    
    num_iterations = num_iterations + 1;
    fprintf("iteration conut: %d\n", num_iterations);
    if max_iterations <=  num_iterations 
        fprintf("error: iterations count more!\n");
        break;
    end
end
fprintf("param: \n");
fprintf("\t\toffset:  %10f %10f %10f\n", offset(1), offset(2), offset(3));
fprintf("\t\tdiag:    %10f %10f %10f\n", diag(1), diag(2), diag(3));
fprintf("\t\toffdiag: %10f %10f %10f\n", offdiag(1), offdiag(2), offdiag(3));
fprintf("\nThat is: = \n");
fprintf("\t\t| x |\t| %10f %10f %10f |\t| xm + %f |\n", diag(1), offdiag(1), offdiag(2), offset(1));
fprintf("\t\t| y | = | %10f %10f %10f | *  | xm + %f |\n", offdiag(1), diag(2), offdiag(3), offset(2));
fprintf("\t\t| z |\t| %10f %10f %10f |\t| xm + %f |\n", offdiag(2), offdiag(3), diag(3), offset(3));


contrast_curve_accle(param);



 function fitness = calc_mean_squared_residuals(data, diag, offdiag, offset)
   
    sum = 0.0;
    row = size(data, 1);
    
    for i=1:row
        resid = calc_residual(data(i,:)',diag, offdiag, offset);
        sum = sum + power(resid,2);
    end
    fitness = sum / row;
 end
 
 function residual = calc_residual(sample,diag, offdiag, offset)
 
    M = [diag(1)    offdiag(1)  offdiag(2);...
         offdiag(1) diag(2)     offdiag(3);...
         offdiag(2) offdiag(3)  diag(3)];
 
   residual = 1.0 - norm(M*(sample+offset)); 
 end
 
 function jacob = calc_jacob(sample,diag, offdiag, offset)
 
    M = [diag(1)    offdiag(1)  offdiag(2);...
         offdiag(1) diag(2)     offdiag(3);...
         offdiag(2) offdiag(3)  diag(3)];
     
    estimate = M * (sample+offset);
    length = norm(estimate);
    
    A = estimate(1);
    B = estimate(2);
    C = estimate(3);
    
    %0-2 offset
    jacob(1) = -1.0 * (((diag(1)    * A) + (offdiag(1) * B) + (offdiag(2) * C))/length);
    jacob(2) = -1.0 * (((offdiag(1) * A) + (diag(2)    * B) + (offdiag(3) * C))/length);
    jacob(3) = -1.0 * (((offdiag(2) * A) + (offdiag(3) * B) + (diag(3)    * C))/length);
     
    %3-5: diag
    jacob(4) = -1.0 * ((sample(1) + offset(1)) * A)/length;
    jacob(5) = -1.0 * ((sample(2) + offset(2)) * B)/length;
    jacob(6) = -1.0 * ((sample(3) + offset(3)) * C)/length;
    
    %6-8: off-diagonals
    jacob(7) = -1.0 * (((sample(2) + offset(2)) * A) + ((sample(1) + offset(1)) * B))/length;
    jacob(8) = -1.0 * (((sample(3) + offset(3)) * A) + ((sample(1) + offset(1)) * C))/length;
    jacob(9) = -1.0 * (((sample(3) + offset(3)) * B) + ((sample(2) + offset(2)) * C))/length;
 
 end