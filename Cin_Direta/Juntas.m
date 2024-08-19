function [theta,d,a,alpha,tipo] = Juntas(Nmr_de_Juntas)
    for i=1:Nmr_de_Juntas
        theta(i) = input('Theta:');
        d(i) = input('D:');
        a(i) = input('A:');
        alpha(i) = input('Alpha:');
        tipo(i) = input('r ou p:', 's');
        
    end
