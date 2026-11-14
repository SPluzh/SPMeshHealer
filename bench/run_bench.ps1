param(
    [string]$InputFile = "C:\Users\user\Desktop\cpp\sculptsp-native\models\SPMeshHealer_in.goz",
    [string]$ExePath = "$PSScriptRoot\..\dist\SPMeshHealer.exe",
    [int]$Runs = 3,
    [int]$Hull = 1
)

$ErrorActionPreference = "Stop"
if (!(Test-Path $InputFile)) { throw "Input file not found: $InputFile" }
if (!(Test-Path $ExePath)) { throw "Executable not found: $ExePath" }

$outFile = Join-Path $env:TEMP "spmh_bench_out$([System.IO.Path]::GetExtension($InputFile))"
$verifyFile = Join-Path $env:TEMP "spmh_bench_verify$([System.IO.Path]::GetExtension($InputFile))"

$loadTimes = @()
$healTimes = @()
$totalTimes = @()
$lastVerts = ""
$lastFaces = ""
$lastHoles = ""

Write-Host "=== SPMeshHealer Benchmark ==="
Write-Host "Input: $InputFile"
Write-Host "Exe:   $ExePath"
Write-Host "Hull:  $Hull | Runs: $Runs"
Write-Host ""

for ($i = 1; $i -le $Runs; $i++) {
    if (Test-Path $outFile) { Remove-Item $outFile -Force }
    $output = & $ExePath --in $InputFile --out $outFile --hull $Hull 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) {
        Write-Host $output
        throw "Run $i failed with exit code $LASTEXITCODE"
    }

    if ($output -match "Loaded in ([0-9.]+) ms") { $loadTimes += [double]$Matches[1] }
    if ($output -match "Healed in ([0-9.]+) ms") { $healTimes += [double]$Matches[1] }
    if ($output -match "Done in ([0-9.]+) ms \| Verts: ([^|]+) \| Faces: ([^|]+) \| Holes filled: (\S+)") {
        $totalTimes += [double]$Matches[1]
        $lastVerts = $Matches[2].Trim()
        $lastFaces = $Matches[3].Trim()
        $lastHoles = $Matches[4].Trim()
    }
    Write-Host ("Run {0}: Load={1:F1} ms | Heal={2:F1} ms | Total={3:F1} ms" -f $i, $loadTimes[-1], $healTimes[-1], $totalTimes[-1])
}

# Verify output topology (check remaining holes & manifoldness)
$verifyOut = & $ExePath --in $outFile --out $verifyFile --hull $Hull 2>&1 | Out-String
$nonManifold = $verifyOut -match "Non-manifold"
$remHoles = if ($verifyOut -match "After stitch_borders: (\d+) boundary hole") { [int]$Matches[1] } else { -1 }

if (Test-Path $outFile) { Remove-Item $outFile -Force }
if (Test-Path $verifyFile) { Remove-Item $verifyFile -Force }

$avgLoad  = ($loadTimes  | Measure-Object -Average).Average
$avgHeal  = ($healTimes  | Measure-Object -Average).Average
$avgTotal = ($totalTimes | Measure-Object -Average).Average
$minTotal = ($totalTimes | Measure-Object -Minimum).Minimum

Write-Host ""
Write-Host "=== Summary ($Runs runs) ==="
Write-Host ("Avg Load:  {0:F2} ms" -f $avgLoad)
Write-Host ("Avg Heal:  {0:F2} ms" -f $avgHeal)
Write-Host ("Avg Total: {0:F2} ms (min: {1:F2} ms)" -f $avgTotal, $minTotal)
Write-Host "Verts:     $lastVerts"
Write-Host "Faces:     $lastFaces"
Write-Host "Holes:     $lastHoles"
Write-Host "Verify:    remainingHoles=$remHoles | nonManifold=$nonManifold"
