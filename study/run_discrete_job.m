try
 cd(fileparts(mfilename('fullpath')));run_discrete_comparison;
catch e
 disp(getReport(e,'extended'));bdclose('all');exit(1);
end
bdclose('all');exit(0);
