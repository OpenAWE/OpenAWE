% run this script to setup OpenAWE

rootDir  = fileparts(which('startOpenAWE'));
addpath(genpath(fullfile(rootDir,'Modeling')));
addpath(genpath(fullfile(rootDir,'Optimization')));


% check if casadi is working
try
  casadi.SX.sym('x');
catch e
  if ~strcmp(e.identifier,'MATLAB:undefinedVarOrClass')
    error('Casadi installation in the path found but does not work properly. Try restarting Matlab.');
  else
    error('Casadi installation not found. Please setup casadi 3.3');
  end
end

% make sure OpenOCL is in the path and StartupOCL is executed
openocl_test = getenv('OPENOCL_TEST')
if isempty(openocl_test)
  error('Please setup OpenOCL first and run StartupOCL.m');
end
