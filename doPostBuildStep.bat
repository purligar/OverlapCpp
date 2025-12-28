@echo off%OpenCvVersionShort%%ConfigChar%

set TargetDir=%1%
set OutDir=%2%
set PlatformToolset=%3%
set PlatformTarget=%4%
set Platform=%PlatformToolset%-%PlatformTarget%
set Configuration=%5%
if "%Configuration%" == "Debug" (
	set ConfigChar=d
)

echo ----- Post Build Step for Configuration %Configuration% -----
if defined CML (
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_core-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_core450%ConfigChar%.dll
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_imgproc-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_imgproc450%ConfigChar%.dll
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_highgui-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_highgui450%ConfigChar%.dll
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_videoio-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_videoio450%ConfigChar%.dll
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_imgcodecs-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_imgcodecs450%ConfigChar%.dll
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_calib3d-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_calib3d450%ConfigChar%.dll
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_features2d-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_features2d450%ConfigChar%.dll
	call doCopy.bat %CML%\OpenCV\OpenCV-4.5.0\opencv_flann-V141-%PlatformTarget%-4_5_0-md%ConfigChar%.dll %TargetDir%opencv_flann450%ConfigChar%.dll
	if "%Configuration%" == "Debug" (
		call doCopy.bat %CML%\TBB\TBB-2020.3\V141-%PlatformTarget%\tbb_debug.dll %TargetDir%tbb_debug.dll
	) else (
		call doCopy.bat %CML%\TBB\TBB-2020.3\V141-%PlatformTarget%\tbb.dll %TargetDir%tbb.dll
	)

	if defined BASLER (
		call doCopy.bat %CML%\Basler\Basler-8.1-x64\*.dll %TargetDir%
	)
) else (
	echo CML environment variable not defined. Skipping OpenCV and Basler DLL copy.
)