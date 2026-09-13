$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    & g++ -std=c++14 -O2 -Wall -Wextra -pedantic -static main.cpp -o problem4.exe -lwinhttp
    if ($LASTEXITCODE -ne 0) { throw 'C++14 compilation failed' }
    Write-Host 'Built problem4.exe'
} finally {
    Pop-Location
}
