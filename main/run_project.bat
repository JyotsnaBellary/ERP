cd build
cmake --build . --verbose

@REM Check if the build was successful
if %ERRORLEVEL% NEQ 0 (
    echo Build failed with error code %ERRORLEVEL%
    exit /b %ERRORLEVEL%
)
echo Running: main\Debug\graph_reader.exe
@REM dir main\Debug\graph_reader.exe
Debug\graph_reader.exe