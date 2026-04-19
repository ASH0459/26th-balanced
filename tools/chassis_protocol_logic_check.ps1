$ErrorActionPreference = 'Stop'

$taskHeaderPath = 'Users/Application/APP_Chassis/App_Chassis_Task.h'
$taskHeader = Get-Content $taskHeaderPath -Raw
$paramsHeaderPath = 'Users/Application/APP_Chassis/App_Chassis_Params.h'
$paramsHeader = Get-Content $paramsHeaderPath -Raw

$dtMsMatch = [regex]::Match($taskHeader, '(?m)^\s*#define\s+CHASSIS_CONTROL_TIME_MS\s+([0-9]+)\b')
$dtSecMatch = [regex]::Match($taskHeader, '(?m)^\s*#define\s+CHASSIS_CONTROL_TIME\s+([0-9]*\.?[0-9]+)f?\b')
$freqMatch = [regex]::Match($taskHeader, '(?m)^\s*#define\s+CHASSIS_CONTROL_FREQUENCE\s+([0-9]*\.?[0-9]+)f?\b')

if (-not $dtMsMatch.Success -or -not $dtSecMatch.Success -or -not $freqMatch.Success) {
  throw "Failed to parse control-time macros from $taskHeaderPath"
}

$dtMs = [int]$dtMsMatch.Groups[1].Value
$dt = [double]$dtSecMatch.Groups[1].Value
$freq = [double]$freqMatch.Groups[1].Value

$dtFromMs = $dtMs / 1000.0
$freqFromDt = 1.0 / $dt

if ([Math]::Abs($dt - $dtFromMs) -gt 1e-9) {
  throw "Control time mismatch: CHASSIS_CONTROL_TIME_MS=$dtMs ms but CHASSIS_CONTROL_TIME=$dt s"
}

if ([Math]::Abs($freq - $freqFromDt) -gt 1e-3) {
  throw "Frequency mismatch: CHASSIS_CONTROL_FREQUENCE=$freq Hz but 1/CHASSIS_CONTROL_TIME=$freqFromDt Hz"
}

$gyroRampUpMatch = [regex]::Match($paramsHeader, '(?m)^\s*#define\s+CHASSIS_SMALL_GYRO_RAMP_UP_RATE\s+([0-9]*\.?[0-9]+)f?\b')
$gyroRampDownMatch = [regex]::Match($paramsHeader, '(?m)^\s*#define\s+CHASSIS_SMALL_GYRO_RAMP_DOWN_RATE\s+([0-9]*\.?[0-9]+)f?\b')

if (-not $gyroRampUpMatch.Success -or -not $gyroRampDownMatch.Success) {
  throw "Failed to parse small-gyro ramp macros from $paramsHeaderPath"
}

# 1) Gyro ramp up/down smoothness (same constants as behaviour file)
# dt comes from CHASSIS_CONTROL_TIME in App_Chassis_Task.h.
$up = [double]$gyroRampUpMatch.Groups[1].Value
$down = [double]$gyroRampDownMatch.Groups[1].Value
$target = 12.0
$spin = 0.0
$trace = @()

$upStep = $up * $dt
$downStep = $down * $dt
$upTicks = [int][Math]::Ceiling($target / $upStep) + 10
$downTicks = [int][Math]::Ceiling($target / $downStep) + 10
$upEndIndex = $upTicks - 1

for ($i = 0; $i -lt $upTicks; $i++) {
  $step = $upStep
  if ($spin -lt $target) { $spin = [Math]::Min($spin + $step, $target) }
  $trace += $spin
}

for ($i = 0; $i -lt $downTicks; $i++) {
  $step = $downStep
  if ($spin -gt 0.0) { $spin = [Math]::Max($spin - $step, 0.0) }
  $trace += $spin
}

$monoUp = $true
for ($i = 1; $i -lt $upTicks; $i++) {
  if ($trace[$i] + 1e-9 -lt $trace[$i-1]) { $monoUp = $false; break }
}

$monoDown = $true
for ($i = $upTicks; $i -lt $trace.Count; $i++) {
  if ($trace[$i] -gt $trace[$i-1] + 1e-9) { $monoDown = $false; break }
}

$gyroPass = $monoUp -and $monoDown -and ([Math]::Abs($trace[$upEndIndex] - $target) -lt 1e-6) -and ([Math]::Abs($trace[-1] - 0.0) -lt 1e-6)

# 2) Static source checks for mode=6 no-action and VT timeout safe-stop
$beh = Get-Content 'Users/Application/APP_Chassis/App_Chassis_Behaviour.cpp' -Raw
$canh = Get-Content 'Users/Bsp/Bsp_Device/Dev_Can_Receive/Dev_Can_Receive.h' -Raw
$task = Get-Content 'Users/Application/APP_Chassis/App_Chassis_Task.cpp' -Raw

$mode6Reserved = $canh -match 'CHASSIS_MODE_RESERVED_2\s*=\s*6'
$normalGate = $beh -match 'requested_mode\s*==\s*CHASSIS_MODE_NORMAL\s*&&\s*protocol_valid'
$nonNormalHold = $beh -match 'chassis_action_hold_control\('
$mode6Pass = $mode6Reserved -and $normalGate -and $nonNormalHold

$timeoutBranch = $task -match 'toe_is_error\(VT_TOE\)'
$jointZero = $task -match 'chassis_move\.chassis_joint\[i\]\.joint_T\s*=\s*0\.0f'
$wheelZeroL = $task -match 'chassis_move\.chassis_wheel\[0\]\.wheel_T\s*=\s*0\.0f'
$wheelZeroR = $task -match 'chassis_move\.chassis_wheel\[1\]\.wheel_T\s*=\s*0\.0f'
$timeoutPass = $timeoutBranch -and $jointZero -and $wheelZeroL -and $wheelZeroR

$summary = [PSCustomObject]@{
  ControlTimeConsistencyPass = $true
  GyroRampSmoothPass = $gyroPass
  Mode6NoActionPass = $mode6Pass
  TimeoutSafeStopPass = $timeoutPass
  GyroRampUpFinal = [Math]::Round($trace[$upEndIndex], 3)
  GyroRampDownFinal = [Math]::Round($trace[-1], 3)
}

"=== SUMMARY ==="
$summary | Format-List
