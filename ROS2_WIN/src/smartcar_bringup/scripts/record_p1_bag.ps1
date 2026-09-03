param(
  [Parameter(Mandatory = $true)]
  [string]$Output,
  [switch]$UseSimTime
)

$Output = $Output.Trim()
if ([string]::IsNullOrWhiteSpace($Output)) { throw "Output must not be empty" }
$parent = Split-Path -Parent $Output
if ($parent -and -not (Test-Path -LiteralPath $parent)) {
  New-Item -ItemType Directory -Path $parent -Force | Out-Null
}
$topics = @('/scan', '/odom', '/tf', '/tf_static', '/diagnostics', '/map')
$rosArgs = @('bag', 'record', '-o', $Output) + $topics
if ($UseSimTime) { $rosArgs += '--use-sim-time' }
& ros2 @rosArgs
exit $LASTEXITCODE
