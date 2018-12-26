% run this script to setup OpenAWE

rootDir  = fileparts(which('startOpenAWE'));
addpath(genpath(fullfile(rootDir,'Modeling')));
addpath(genpath(fullfile(rootDir,'Optimization')));


% make sure OpenOCL is in the path and StartupOCL is executed
openocl_test = getenv('OPENOCL_TEST')
if isempty(openocl_test)
  error('Please setup OpenOCL first and run StartupOCL.m');
end
