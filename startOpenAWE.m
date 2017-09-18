% run this script to setup OpenAWE

rootDir  = fileparts(which('startOpenAWE'));
addpath(genpath(fullfile(rootDir,'Modeling')));
addpath(genpath(fullfile(rootDir,'Optimization')));

% setup casadi
try
  casadi.SX.sym('x');
  disp('Casadi installation in the path found.');
catch e
  error('Casadi installation not found. Please setup casadi 3.2');
end

addpath(fullfile(rootDir,'libs'));
addpath(fullfile(rootDir,'libs','OpenOCL'));
StartupOC(fullfile(rootDir,'..'))