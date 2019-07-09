awe_path = fileparts(which('awe'));
addpath(awe_path);

if exist(fullfile(awe_path, 'lib'), 'dir')
  disp('OpenOCL is already setup in the ./lib folder. If it does not work delete the lib folder and try again.')
else
  websave(fullfile(awe_path, 'openocl.zip'), 'https://github.com/OpenOCL/OpenOCL/archive/a997739267e80bbc028adaee0fbce49cfef3b342.zip');
  unzip(fullfile(awe_path, 'openocl.zip'), fullfile(awe_path, 'lib'));
end

addpath(genpath(fullfile(awe_path, 'lib')))

if exist(fullfile(awe_path, 'openocl.zip'), 'file')
  delete(fullfile(awe_path, 'openocl.zip'));
end