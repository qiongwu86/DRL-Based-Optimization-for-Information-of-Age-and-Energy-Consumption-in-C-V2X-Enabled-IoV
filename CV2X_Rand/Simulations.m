close all    % Close all open figures
clear        % Reset variables
clc          % Clear the command window

%LTEV2Vsim('help');
% Configuration file
% configFile = 'BenchmarkPoisson.cfg'; 
%configFile = 'BolognaA.cfg';
configFile = 'Highway3GPP.cfg';

% Simulation time (s)
T = 10;

b = 5984;  %bit         不同信道数对应bit数：968 2216 3368 4584 5992 -8
% Beacon size (bytes)   为什么初始190
B = b/8;
 
%% LTE Autonomous (3GPP Mode 4) - on a subframe basis
% Autonomous allocation algorithm defined in 3GPP standard

LTEV2Vsim(configFile,'simulationTime',T,'BRAlgorithm',18,'Raw',150,...
    'beaconSizeBytes',B); 

