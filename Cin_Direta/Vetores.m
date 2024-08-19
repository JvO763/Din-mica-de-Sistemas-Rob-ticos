function [normal, slide, approach, position] = Vetores(matriz)
    normal = matriz(1:3,1);
    slide = matriz(1:3,2);
    approach = matriz(1:3,3);
    position = matriz(1:3,4);