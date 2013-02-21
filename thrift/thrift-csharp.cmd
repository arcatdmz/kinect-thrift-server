thrift -gen csharp KinectService.thrift
move /Y .\gen-csharp\Jp\Digitalmuseum\Kinect\* ..\csharp\ConsoleKinectServer\
rmdir /Q /S gen-csharp
