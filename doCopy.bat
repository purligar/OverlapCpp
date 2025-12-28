@echo off
setlocal enableextensions enabledelayedexpansion

rem Assume files to be dll files
set source_dll=%1
set target_dll=%2

rem Create filenames with pdb extension
set source_pdb=%~d1%~p1%~n1.pdb
set target_pdb=%~d2%~p2%~n2.pdb

rem Check if at least one of the two files exists
if not exist %source_dll% (
	if not exist %source_pdb% (
		echo Neither !source_dll! nor !source_pdb! exist !
		echo Check your Post Build Script
		exit 1
	)
)

rem Copy dll file
if exist %source_dll% (
	set cmd_dll=copy /B /Y !source_dll! !target_dll!
	echo Copying DLL file: !cmd_dll!
	!cmd_dll!

	rem Check if copy returned an error
	if %ERRORLEVEL% GEQ 1 (
		echo copy returned %ERRORLEVEL%, copying DLL file !target_dll! failed !
		exit %ERRORLEVEL%
	)

	rem Check if the file exists (just to make sure)
	if not exist !target_dll! (
		echo DLL file !target_dll! does not exist, copying failed !
		exit 1
	)
)

rem Copy pdb file if it exists
if exist %source_pdb% (
	set cmd_pdb=copy /B /Y !source_pdb! !target_pdb!
	echo Copying PDB file: !cmd_pdb!
	!cmd_pdb!

	rem Check if copy returned an error
	if %ERRORLEVEL% GEQ 1 (
		echo copy returned %ERRORLEVEL%, copying PDB file !target_pdb! failed !
		exit %ERRORLEVEL%
	)

	rem Check if the file exists (just to make sure)
	if not exist !target_pdb! (
		echo PDB file !target_pdb! does not exist, copying failed !
		exit 1
	)
)
