if exist('FORCESNLPsolver', 'dir')
    rmdir FORCESNLPsolver s
end

if exist('@FORCESproWS', 'dir')
    rmdir @FORCESproWS s
end
delete *.c
delete *.o
delete *.py
delete *.mexa64 *.csv *.forces FORCESNLPsolver.m FORCESNLPsolver_mex_compiler_warnings.txt stdout_temp
