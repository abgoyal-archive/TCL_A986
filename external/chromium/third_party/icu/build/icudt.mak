BUILD :
	@nmake /nologo /f $(ProjectDir)\..\source\data\makedata.mak ICUMAKE=$(ProjectDir)\..\source\data CFG=$(ConfigurationName) datalib
	@xcopy /R /C /Y /d $(ProjectDir)\..\bin\icudt42.dll $(ProjectDir)\..\..\..\$(ConfigurationName)\

REBUILD :
	@nmake /nologo /f $(ProjectDir)\..\source\data\makedata.mak ICUMAKE=$(ProjectDir)\..\source\data CFG=$(ConfigurationName) clean datalib
	@xcopy /R /C /Y /d $(ProjectDir)\..\bin\icudt42.dll $(ProjectDir)\..\..\..\$(ConfigurationName)\

CLEAN :
	@nmake /nologo /f $(ProjectDir)\..\source\data\makedata.mak ICUMAKE=$(ProjectDir)\..\source\data CFG=$(ConfigurationName) clean
