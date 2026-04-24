# Build PlatformIO examples with prefix filter and skip prefixes
# e.g.
# 1. Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope Process
# 2. .\tools\test_build\build.ps1 -prefix "BHI" -skipPrefix "ESP_IDF*"

param(
    [string]$prefix = "*",
    [string]$skipPrefix = ""
)

$logFile = "build_log.txt"

$null = New-Item -Path $logFile -ItemType File -Force
"=== Build started at $(Get-Date) ===" | Out-File -FilePath $logFile -Encoding utf8

Write-Host "Starting build process..." -ForegroundColor Cyan

$examplesRoot = (Resolve-Path "examples").Path
$allExamples = Get-ChildItem -Path "examples" -Directory -Recurse | Where-Object {
    $_.Name -ne "main" -and (
        @($_.GetFiles("*.ino")).Count -gt 0 -or
        @($_.GetFiles("*.cpp")).Count -gt 0 -or
        @($_.GetFiles("*.c")).Count -gt 0
    )
} | ForEach-Object { $_.FullName.Replace($examplesRoot + "\", "") }

if ($prefix -ne "*") {
    $examples = $allExamples | Where-Object { (Split-Path $_ -Leaf) -like "${prefix}*" }
} else {
    $examples = $allExamples
}

if ($skipPrefix) {
    $skipPatterns = $skipPrefix -split ',' | ForEach-Object { $_.Trim() }
    foreach ($pattern in $skipPatterns) {
        $examples = $examples | Where-Object { (Split-Path $_ -Leaf) -notlike $pattern }
    }
}

if ($examples.Count -eq 0) {
    Write-Host "ERROR: No examples found after filtering" -ForegroundColor Red
    exit 1
}

$envs = @("esp32dev_arduino")
$total = $examples.Count * $envs.Count
$current = 0
$success = 0
$failed = 0
$skipped = 0

platformio run -t clean > $null 2>&1

foreach ($env in $envs) {
    foreach ($example in $examples) {
        $current++
        Write-Progress -Activity "Building examples" -Status "$example ($env)" -PercentComplete (($current / $total) * 100)

        if (Test-Path "examples/$example/.skip.$env") {
            $skipped++
            $skipMsg = "SKIP: $example ($env) - .skip.$env exists"
            Write-Host $skipMsg -ForegroundColor Yellow
            $skipMsg | Out-File -FilePath $logFile -Append -Encoding utf8
            continue
        }

        $buildHeader = "`n=== Building $example ($env) ==="
        $buildHeader | Out-File -FilePath $logFile -Append -Encoding utf8

        $env:PLATFORMIO_SRC_DIR = "examples/$example"
        $output = platformio run -e $env 2>&1

        $output | Out-File -FilePath $logFile -Append -Encoding utf8

        if ($LASTEXITCODE -ne 0) {
            $failed++
            $errorMsg = "ERROR: Build failed for $example ($env)"
            Write-Host $errorMsg -ForegroundColor Red
            # $output | Select-Object -First 5 | Write-Host -ForegroundColor Red
        } else {
            $success++
        }
    }
}

Write-Progress -Activity "Building examples" -Completed

$built = $total - $skipped
$summary = @"
`n=== Build Summary ===
Total combinations : $total
Skipped            : $skipped
Built              : $built
Successful         : $success
Failed             : $failed
"@

Write-Host $summary -ForegroundColor Green
$summary | Out-File -FilePath $logFile -Append -Encoding utf8
"=== Build finished at $(Get-Date) ===" | Out-File -FilePath $logFile -Append -Encoding utf8

if ($failed -gt 0) {
    exit 1
} else {
    exit 0
}