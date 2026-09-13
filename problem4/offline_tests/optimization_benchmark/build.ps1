$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & g++ -std=c++14 -O2 -Wall -Wextra -pedantic -static bridge.cpp -o bridge.exe -lwinhttp
    if ($LASTEXITCODE -ne 0) { throw 'Benchmark compilation failed' }
} finally {
    Pop-Location
}
