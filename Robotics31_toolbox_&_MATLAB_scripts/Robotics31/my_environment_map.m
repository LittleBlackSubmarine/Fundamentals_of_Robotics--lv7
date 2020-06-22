clear;
close all;
clc;

% environment map

map.dim = 2;

map.X = [
    260, 0;    %1
    260, 165;  %2
    0, 165;    %3
    0, 0;      %4
    5, 160;    %5
    255, 160;  %6
    255, 5;    %7
    5, 5;      %8
    60, 160;   %9
    65, 160;   %10
    60, 145;   %11
    65, 145;   %12
    60, 120;   %13
    70, 120;   %14
    60, 115;   %15
    65, 115;   %16
    70, 100;   %17
    65, 105;   %18
    45, 40;    %19
    45, 35;    %20
    75, 60;    %21
    75, 55;    %22
    175, 160;  %23
    180, 160;  %24
    175, 115;  %25
    180, 115;  %26
    175, 80;   %27
    180, 80;   %28
    180, 55;   %29
    175, 60;   %30
    210, 80;   %31
    210, 75;   %32
    125, 55;   %33
    130, 55;   %34
    5, 105;    %35
    5, 100;    %36
    5, 40;     %37
    5, 35;     %38
    125, 5;    %39
    130, 5;    %40
    255, 80;   %41
    255, 75;   %42
    70, 115;   %43
    70, 105;   %44
    180, 60;   %45
    ]'*0.05;   % 45 kuteva radne okoline (neki zapravo nisu kutevi u 
               % stvarnosti, ali su bili potrebni radi podjele odredenih 
               % prepreka na trokute

map.S = [
    1, 2, 6;
    1, 6, 5;
    2, 3, 7;
    2, 7, 6;
    3, 4, 7;
    4, 8, 7;
    4, 1, 8;
    1, 5, 8;
    9, 10, 12;
    9, 12, 11;
    13, 14, 43;
    13, 43, 15;
    16, 43, 18;
    43, 44, 18;
    35, 44, 17;
    35, 17, 36;
    37, 19, 20;
    37, 20, 38;
    23, 26, 25;
    23, 24, 26;
    31, 41, 42;
    31, 42, 32;
    27, 28, 45;
    27, 45, 30;
    21, 45, 22;
    45, 29, 22;
    33, 40, 39;
    33, 34, 40
    ]';

map = edges2(map);

map.nL = 0;

% robot

robot = createmobrob();

x0 = [235*0.05 25*0.05 2*pi/3]';  % Pocetni polozaj i kut mobilnog robota

robot.mem = [1; x0];  %mobrobctrlalg -> [iPt we]

% controller

ctrlparam.T = 0.05;
ctrlparam.vmax = 0.5;
ctrlparam.rho0 = 1;
ctrlparam.eta = 1e-4;
ctrlparam.path = [
    225, 40;  %B
    180, 92.5;%C
    165, 92.5;%D
    80, 75;   %E
    70, 75;   %F
    50, 30;   %G
    20, 20    %H
    ]'*0.05;     % Tocke putanje mobilnog robota


sim('mobrob_sim.mdl');

