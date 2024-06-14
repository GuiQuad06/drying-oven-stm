:: Create symlinks for git hooks
FOR %%i in (%~dp0git_hooks\*) DO mklink %~dp0..\.git\hooks\%%~ni %%i
