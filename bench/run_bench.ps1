param(
    [string[]]$InputFiles = @(
        "C:\Users\user\Desktop\cpp\sculptsp-native\models\SPMeshHealer_in.goz",
        "C:\Users\user\Desktop\cpp\sculptsp-native\models\SPMeshHealer_in.obj",
        "C:\Users\user\Desktop\cpp\sculptsp-native\models\SPMeshHealer_in.stl"
    ),
    [string]$ExePath = "$PSScriptRoot\..\dist\SPMeshHealer.exe",
    [int]$Runs = 2,
    [int]$Hull = 1
)

$ErrorActionPreference = "Stop"
if (!(Test-Path $ExePath)) { throw "Executable not found: $ExePath" }

Write-Host "=== SPMeshHealer Benchmark Suite ==="
Write-Host "Exe:  $ExePath"
Write-Host "Hull: $Hull | Runs per file: $Runs"
Write-Host ""

foreach ($InputFile in $InputFiles) {
    if (!(Test-Path $InputFile)) {
        Write-Warning "Skipping missing file: $InputFile"
        continue
    }

    $ext = [System.IO.Path]::GetExtension($InputFile)
    $name = [System.IO.Path]::GetFileName($InputFile)
    $outFile = Join-Path $env:TEMP "spmh_bench_out$ext"
    $verifyFile = Join-Path $env:TEMP "spmh_bench_verify$ext"

    $loadTimes = @()
    $healTimes = @()
    $totalTimes = @()
    $lastVerts = ""
    $lastFaces = ""
    $lastHoles = ""

    Write-Host "--- $name ---"
    for ($i = 1; $i -le $Runs; $i++) {
        if (Test-Path $outFile) { Remove-Item $outFile -Force }
        $output = & $ExePath --in $InputFile --out $outFile --hull $Hull 2>&1 | Out-String
        if ($LASTEXITCODE -ne 0) {
            Write-Host $output
            throw "Run $i failed on $name with exit code $LASTEXITCODE"
        }

        if ($output -match "Loaded in ([0-9.]+) ms") { $loadTimes += [double]$Matches[1] }
        if ($output -match "Healed in ([0-9.]+) ms") { $healTimes += [double]$Matches[1] }
        if ($output -match "Done in ([0-9.]+) ms \| Verts: ([^|]+) \| Faces: ([^|]+) \| Holes filled: (\S+)") {
            $totalTimes += [double]$Matches[1]
            $lastVerts = $Matches[2].Trim()
            $lastFaces = $Matches[3].Trim()
            $lastHoles = $Matches[4].Trim()
        }
        $writeMs = $totalTimes[-1] - $loadTimes[-1] - $healTimes[-1]
        Write-Host ("  Run {0}: Load={1:F1} ms | Heal={2:F1} ms | Write={3:F1} ms | Total={4:F1} ms" -f $i, $loadTimes[-1], $healTimes[-1], $writeMs, $totalTimes[-1])
    }

    $verifyOut = & $ExePath --in $outFile --out $verifyFile --hull $Hull 2>&1 | Out-String
    $nonManifold = $verifyOut -match "Non-manifold"
    $remHoles = if ($verifyOut -match "After stitch_borders: (\d+) boundary hole") { [int]$Matches[1] } else { -1 }

    if (Test-Path $outFile) { Remove-Item $outFile -Force }
    if (Test-Path $verifyFile) { Remove-Item $verifyFile -Force }

    $avgLoad  = ($loadTimes  | Measure-Object -Average).Average
    $avgHeal  = ($healTimes  | Measure-Object -Average).Average
    $avgTotal = ($totalTimes | Measure-Object -Average).Average
    $avgWrite = $avgTotal - $avgLoad - $avgHeal

    Write-Host ("  AVG: Load={0:F1} ms | Heal={1:F1} ms | Write={2:F1} ms | Total={3:F1} ms" -f $avgLoad, $avgHeal, $avgWrite, $avgTotal)
    Write-Host ("  Stats: Verts: {0} | Faces: {1} | Holes: {2} | Verify: remHoles={3}, nonManifold={4}" -f $lastVerts, $lastFaces, $lastHoles, $remHoles, $nonManifold)
    Write-Host ""
}
