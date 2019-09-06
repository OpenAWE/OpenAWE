
ocl_version = '98c3116917e18fc49e0286a81d911fb4dd161075';
ocl_url = ['https://github.com/OpenOCL/OpenOCL/archive/', ocl_version, '.zip'];

awe_path = fileparts(which('awe'));
addpath(awe_path);

ocl_installed = false;

if exist(fullfile(awe_path,'lib','OCL_INSTALLED'), 'file')
  fid = fopen(fullfile(awe_path,'lib','OCL_INSTALLED'), 'r' );
  installed_ocl_version = fscanf(fid, '%s');
  fclose(fid);
  
  if ~strcmp(ocl_version, installed_ocl_version)
    addpath(genpath(fullfile(awe_path, 'lib')));
    rmpath(genpath(fullfile(awe_path, 'lib')));
    s = rmdir(fullfile(awe_path, 'lib'), 's');
    if ~s
      error(['Could not remove OpenOCL. Restart Matlab and try ', ...
        'again or remove the lib directory manually.']);
    end
  else
    ocl_installed = true;
  end
end

if ocl_installed
  disp('OpenOCL is already setup in the ./lib folder. If it does not work delete the lib folder and try again.')
else
  disp('Downloading OpenOCL...');
  websave(fullfile(awe_path, 'openocl.zip'), ocl_url);
  unzip(fullfile(awe_path, 'openocl.zip'), fullfile(awe_path, 'lib'));
  
  fid = fopen(fullfile(awe_path,'lib','OCL_INSTALLED'), 'wt' );
  fprintf(fid,'%s\n', ocl_version);
  fclose(fid);

end

addpath(genpath(fullfile(awe_path, 'lib')))

if exist(fullfile(awe_path, 'openocl.zip'), 'file')
  delete(fullfile(awe_path, 'openocl.zip'));
end
disp('finished')