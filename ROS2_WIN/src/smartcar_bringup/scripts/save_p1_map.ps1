param(
  [Parameter(Mandatory = $true)]
  [string]$OutputPrefix
)

$OutputPrefix = $OutputPrefix.Trim()
if ([string]::IsNullOrWhiteSpace($OutputPrefix)) {
  throw "OutputPrefix must not be empty"
}
if ($OutputPrefix -match "['`r`n]") {
  throw "OutputPrefix must not contain a quote or newline"
}
if ($OutputPrefix.EndsWith('.yaml', [System.StringComparison]::OrdinalIgnoreCase)) {
  $OutputPrefix = $OutputPrefix.Substring(0, $OutputPrefix.Length - 5)
}
$parent = Split-Path -Parent $OutputPrefix
if ($parent -and -not (Test-Path -LiteralPath $parent)) {
  New-Item -ItemType Directory -Path $parent -Force | Out-Null
}

ros2 run nav2_map_server map_saver_cli -t /map -f $OutputPrefix --fmt pgm --mode trinary
if ($LASTEXITCODE -ne 0) {
  throw "map_saver_cli failed with exit code $LASTEXITCODE"
}
$yaml = Get-Item -LiteralPath "$OutputPrefix.yaml" -ErrorAction SilentlyContinue
$pgm = Get-Item -LiteralPath "$OutputPrefix.pgm" -ErrorAction SilentlyContinue
if ($null -eq $yaml -or $yaml.Length -le 0 -or
    $null -eq $pgm -or $pgm.Length -le 0) {
  throw "map_saver_cli returned success but did not create non-empty .yaml and .pgm"
}
Write-Host "Saved map: $OutputPrefix.yaml and $OutputPrefix.pgm"
