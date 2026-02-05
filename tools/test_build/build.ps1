# Build PlatformIO examples with prefix filter
# e.g.
# 1. Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
# 2. .\tools\test_build\build.ps1 -prefix "BHI"

param(
    [string]$prefix = "*"
)

# Set-Location ..\..\
Write-Host "Starting build process..." -ForegroundColor Cyan

# Get and filter examples
$examples = Get-ChildItem -Path "examples" -Directory -Name
if ($prefix -ne "*") {
    $examples = $examples | Where-Object { $_ -like "${prefix}*" }
}

if ($examples.Count -eq 0) {
    Write-Host "ERROR: No examples found" -ForegroundColor Red
    exit 1
}

$envs = @("nrf52840_arduino")
$total = $examples.Count * $envs.Count
$current = 0
$success = 0

# Clean
platformio run -t clean > $null 2>&1

foreach ($env in $envs) {
    foreach ($example in $examples) {
        $current++
        Write-Progress -Activity "Building examples" -Status "$example ($env)" -PercentComplete (($current / $total) * 100)
        
        # Skip if .skip file exists
        if (Test-Path "examples/$example/.skip.$env") {
            continue
        }
        
        # Build
        $env:PLATFORMIO_SRC_DIR = "examples/$example"
        $output = platformio run -e $env 2>&1
        
        if ($LASTEXITCODE -ne 0) {
            Write-Host "`nERROR: Build failed for $example ($env)" -ForegroundColor Red
            Write-Host $output -ForegroundColor Red
            exit 1
        } else {
            $success++
        }
    }
}

Write-Progress -Activity "Building examples" -Completed
Write-Host "`nBuild completed: $success/$total successful" -ForegroundColor Green