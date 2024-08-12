close all    % Close all open figures
clear        % Reset variables
clc          % Clear the command window

%LTEV2Vsim('help');
% Configuration file
% configFile = 'BenchmarkPoisson.cfg'; 
%configFile = 'BolognaA.cfg';
configFile = 'Highway3GPP.cfg';

% Simulation time (s)
T = 1500;
bit = 2000;
% Beacon size (bytes)
B = bit/8;
 
%% LTE Autonomous (3GPP Mode 4) - on a subframe basis
% Autonomous allocation algorithm defined in 3GPP standard

LTEV2Vsim(configFile,'simulationTime',T,'BRAlgorithm',18,'Raw',150,...
    'beaconSizeBytes',B); 

