param(
  [Parameter(Mandatory = $true)]
  [string]$OutputPrefix,
  [int]$TimeoutSec = 20
)

$OutputPrefix = $OutputPrefix.Trim()
if ([string]::IsNullOrWhiteSpace($OutputPrefix)) {
  throw "OutputPrefix must not be empty"
}
if ($OutputPrefix -match "['`r`n]") {
  throw "OutputPrefix must not contain a quote or newline"
}
if ($OutputPrefix.EndsWith('.posegraph', [System.StringComparison]::OrdinalIgnoreCase)) {
  $OutputPrefix = $OutputPrefix.Substring(0, $OutputPrefix.Length - 10)
} elseif ($OutputPrefix.EndsWith('.data', [System.StringComparison]::OrdinalIgnoreCase)) {
  $OutputPrefix = $OutputPrefix.Substring(0, $OutputPrefix.Length - 5)
}
if ($TimeoutSec -lt 1) { throw "TimeoutSec must be positive" }
$parent = Split-Path -Parent $OutputPrefix
if ($parent -and -not (Test-Path -LiteralPath $parent)) {
  New-Item -ItemType Directory -Path $parent -Force | Out-Null
}

$available = $false
for ($i = 0; $i -lt $TimeoutSec; $i++) {
  & ros2 service type /slam_toolbox/serialize_map *> $null
  if ($LASTEXITCODE -eq 0) { $available = $true; break }
  Start-Sleep -Seconds 1
}
if (-not $available) {
  throw "slam_toolbox/serialize_map was not available within $TimeoutSec seconds"
}

$request = "{filename: '$OutputPrefix'}"
$response = & ros2 service call /slam_toolbox/serialize_map `
  slam_toolbox/srv/SerializePoseGraph $request 2>&1
if ($LASTEXITCODE -ne 0 -or (($response -join "`n") -notmatch 'result(?:=|:)\s*0\b')) {
  throw "serialize_map failed: $($response -join "`n")"
}
$posegraph = Get-Item -LiteralPath "$OutputPrefix.posegraph" -ErrorAction SilentlyContinue
$data = Get-Item -LiteralPath "$OutputPrefix.data" -ErrorAction SilentlyContinue
if ($null -eq $posegraph -or $posegraph.Length -le 0 -or
    $null -eq $data -or $data.Length -le 0) {
  throw "serialize_map returned success but did not create non-empty .posegraph and .data"
}
Write-Host "Saved posegraph: $OutputPrefix.posegraph and $OutputPrefix.data"
