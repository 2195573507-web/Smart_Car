param(
  [Parameter(Mandatory = $true)]
  [string]$Prefix,
  [ValidateSet('continue', 'localize')]
  [string]$Mode = 'continue',
  [string]$StartPose = '0.0,0.0,0.0',
  [switch]$StartAtDock,
  [int]$TimeoutSec = 20
)

$Prefix = $Prefix.Trim()
if ([string]::IsNullOrWhiteSpace($Prefix)) { throw "Prefix must not be empty" }
if ($Prefix -match "['`r`n]") { throw "Prefix must not contain a quote or newline" }
if ($Prefix.EndsWith('.posegraph', [System.StringComparison]::OrdinalIgnoreCase)) {
  $Prefix = $Prefix.Substring(0, $Prefix.Length - 10)
} elseif ($Prefix.EndsWith('.data', [System.StringComparison]::OrdinalIgnoreCase)) {
  $Prefix = $Prefix.Substring(0, $Prefix.Length - 5)
}
$posegraph = Get-Item -LiteralPath "$Prefix.posegraph" -ErrorAction SilentlyContinue
$data = Get-Item -LiteralPath "$Prefix.data" -ErrorAction SilentlyContinue
if ($null -eq $posegraph -or $posegraph.Length -le 0 -or
    $null -eq $data -or $data.Length -le 0) {
  throw "Both $Prefix.posegraph and $Prefix.data must be non-empty"
}
if ($Mode -eq 'localize' -and $StartAtDock) {
  throw "StartAtDock is valid only for continue mode"
}
$parts = $StartPose -split ','
if ($parts.Count -ne 3) { throw "StartPose must be x,y,theta" }
foreach ($part in $parts) {
  $number = 0.0
  if (-not [double]::TryParse($part.Trim(), [Globalization.NumberStyles]::Float,
      [Globalization.CultureInfo]::InvariantCulture, [ref]$number)) {
    throw "StartPose contains a non-numeric value: $part"
  }
}
if ($TimeoutSec -lt 1) { throw "TimeoutSec must be positive" }
$matchType = if ($Mode -eq 'localize') { 3 } elseif ($StartAtDock) { 1 } else { 2 }
$request = "{filename: '$Prefix', match_type: $matchType, initial_pose: {x: $($parts[0].Trim()), y: $($parts[1].Trim()), theta: $($parts[2].Trim())}}"

$available = $false
for ($i = 0; $i -lt $TimeoutSec; $i++) {
  & ros2 service type /slam_toolbox/deserialize_map *> $null
  if ($LASTEXITCODE -eq 0) { $available = $true; break }
  Start-Sleep -Seconds 1
}
if (-not $available) {
  throw "slam_toolbox/deserialize_map was not available within $TimeoutSec seconds"
}
$response = & ros2 service call /slam_toolbox/deserialize_map `
  slam_toolbox/srv/DeserializePoseGraph $request 2>&1
# Humble's DeserializePoseGraph.srv has an empty response section, so the
# service CLI communicates success through its exit code rather than result: 0.
if ($LASTEXITCODE -ne 0) {
  throw "deserialize_map failed: $($response -join "`n")"
}
Write-Host "Loaded posegraph prefix $Prefix in $Mode mode"
