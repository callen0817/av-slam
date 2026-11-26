#Requires -Version 5.1
<#
.SYNOPSIS
    Builds the native Windows AV SLAM Desktop application (AVSlamDesktop.exe)
#>
$ErrorActionPreference = "Stop"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$BuildDir = Join-Path $RepoRoot "build\desktop_win64"

cmake -S $RepoRoot -B $BuildDir -G "Ninja" `
    -DCMAKE_BUILD_TYPE=Release `
    -DAVSLAM_BUILD_DESKTOP_APP=ON `
    -DAVSLAM_BUILD_IOS_APP=OFF `
    -DAVSLAM_BUILD_TESTS=OFF `
    -DAVSLAM_BUILD_AB_BENCHMARK=OFF

cmake --build $BuildDir --target AVSlamDesktop
Write-Host "Built Windows AV SLAM Desktop: $BuildDir\AVSlamDesktop.exe" -ForegroundColor Green
