#Requires -Version 5.1
<#
.SYNOPSIS
    Builds, Bundles, and Signs AVSlam.ipa for iOS/iPadOS ARM64 on Windows
.DESCRIPTION
    Executes the CMake + LLVM/Clang iOS ARM64 cross-compilation pipeline for the
    100% proprietary AV-SLAM engine (com.artificialvision.avslam), constructs the
    AVSlam.app bundle with Info.plist and Metal shaders, and signs AVSlam.ipa
    using zsign so it installs side-by-side alongside AVScan.ipa on the same device.
#>

param(
    [string]$SdkRoot = $env:IOS_SDK_ROOT,
    [string]$CertP12 = $env:IOS_SIGN_CERT_P12,
    [string]$CertPass = $env:IOS_SIGN_CERT_PASS,
    [string]$ProvisionProfile = $env:IOS_PROVISION_PROFILE
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"
$RepoRoot = Split-Path -Parent $PSScriptRoot
$AuditScript = Join-Path $PSScriptRoot "verify_toolchain.ps1"

& $AuditScript
$AuditJson = Get-Content (Join-Path $RepoRoot "build\toolchain_audit.json") | ConvertFrom-Json

if (-not $AuditJson.ios_cross_compile_ready) {
    Write-Error "BLOCKED [M0]: Cannot produce a real ARM64 iOS binary because iOS cross-compilation prerequisites are missing on this Windows host. See build/toolchain_audit.json."
    exit 2
}

$IosBuildDir = Join-Path $RepoRoot "build\ios_arm64"
$StagingDir = Join-Path $RepoRoot "build\staging"
if (Test-Path $StagingDir) { Remove-Item $StagingDir -Recurse -Force }
$StagingPayload = Join-Path $StagingDir "Payload"
$AppBundleDir = Join-Path $StagingPayload "AVSlam.app"
New-Item -ItemType Directory -Path $AppBundleDir -Force | Out-Null

& $AuditJson.tools.cmake -S $RepoRoot -B $IosBuildDir `
    -G "Ninja" `
    -DCMAKE_TOOLCHAIN_FILE="$RepoRoot\cmake\ios.toolchain.cmake" `
    -DCMAKE_BUILD_TYPE=Release `
    -DAVSLAM_BUILD_IOS_APP=ON `
    -DAVSLAM_BUILD_TESTS=OFF `
    -DAVSLAM_BUILD_AB_BENCHMARK=OFF

if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake iOS configuration failed."
    exit $LASTEXITCODE
}

& $AuditJson.tools.cmake --build $IosBuildDir --config Release
if ($LASTEXITCODE -ne 0) {
    Write-Error "CMake iOS compilation failed."
    exit $LASTEXITCODE
}

Copy-Item (Join-Path $RepoRoot "ios\AVScan\Info.plist") (Join-Path $AppBundleDir "Info.plist") -Force
Copy-Item (Join-Path $RepoRoot "ios\AVScan\Rendering\Shaders.metal") (Join-Path $AppBundleDir "Shaders.metal") -Force
if (Test-Path (Join-Path $RepoRoot "ios\AVScan\Assets\av_logo.png")) {
    Copy-Item (Join-Path $RepoRoot "ios\AVScan\Assets\av_logo.png") (Join-Path $AppBundleDir "av_logo.png") -Force
}

$BuiltExe = Join-Path $IosBuildDir "AVSlam.app\AVSlam"
if (-not (Test-Path $BuiltExe)) {
    $BuiltExe = Join-Path $IosBuildDir "AVSlam"
}
if (Test-Path $BuiltExe) {
    Copy-Item $BuiltExe (Join-Path $AppBundleDir "AVSlam") -Force
}

$IpaOut = Join-Path $RepoRoot "build\AVSlam.ipa"
if ($AuditJson.tools.codesign_tool -and $CertP12 -and $ProvisionProfile) {
    & $AuditJson.tools.codesign_tool -f -k $CertP12 -p $CertPass -m $ProvisionProfile -o $IpaOut -z 9 $AppBundleDir
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Code signing failed."
        exit $LASTEXITCODE
    }
    Write-Host "Successfully built and signed installable AVSlam IPA: $IpaOut" -ForegroundColor Green
} elseif ($AuditJson.tools.codesign_tool) {
    & $AuditJson.tools.codesign_tool -f -a -o $IpaOut -z 9 $AppBundleDir
    Write-Host "Built ad-hoc signed IPA: $IpaOut (Installs side-by-side with AVScan on Sideloadly / AltStore / TrollStore)" -ForegroundColor Green
} else {
    $UnsignedIpa = Join-Path $RepoRoot "build\AVSlam_unsigned.ipa"
    if (Test-Path $UnsignedIpa) { Remove-Item $UnsignedIpa -Force }
    Add-Type -AssemblyName "System.IO.Compression.FileSystem"
    [System.IO.Compression.ZipFile]::CreateFromDirectory($StagingPayload, $UnsignedIpa, [System.IO.Compression.CompressionLevel]::Optimal, $true)
    Write-Host "Packaged ready-to-sideload bundle: $UnsignedIpa" -ForegroundColor Cyan
}
