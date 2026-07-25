param(
    [Parameter(Mandatory = $true)]
    [string]$Port,

    [ValidateRange(1, 120)]
    [int]$DurationSeconds = 15,

    [ValidateRange(0, 2000)]
    [int]$Kp = 200,

    [ValidateRange(0, 2000)]
    [int]$Ki = 0,

    [ValidateRange(0, 100)]
    [int]$Kd = 0,

    [ValidateRange(0, 100)]
    [int]$Target = 40,

    [switch]$AutoStep,
    [switch]$Arm,
    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $OutputPath = Join-Path $PSScriptRoot "pid-capture-$stamp.csv"
}

$serial = [System.IO.Ports.SerialPort]::new(
    $Port,
    115200,
    [System.IO.Ports.Parity]::None,
    8,
    [System.IO.Ports.StopBits]::One)
$serial.NewLine = "`n"
$serial.ReadTimeout = 100
$serial.WriteTimeout = 500
$serial.DtrEnable = $false
$serial.RtsEnable = $false

$writer = $null
$armedByScript = $false

try {
    $serial.Open()
    Start-Sleep -Milliseconds 300
    $serial.DiscardInBuffer()
    $serial.DiscardOutBuffer()

    $writer = [System.IO.StreamWriter]::new($OutputPath, $false)
    $writer.WriteLine('ms,run,auto,target,ramp_target_pps,left_pps,right_pps,left_error,right_error,left_output_permille,right_output_permille,kp_x1000,ki_x1000,kd_x1000')

    $serial.WriteLine('GET')
    if ($Arm) {
        $serial.WriteLine("SET,KP,$Kp")
        $serial.WriteLine("SET,KI,$Ki")
        $serial.WriteLine("SET,KD,$Kd")
        $serial.WriteLine('RESET')
        if ($AutoStep) {
            $serial.WriteLine('AUTO,1')
        } else {
            $serial.WriteLine("SET,TARGET,$Target")
            $serial.WriteLine('RUN')
        }
        $armedByScript = $true
    }

    $timer = [System.Diagnostics.Stopwatch]::StartNew()
    $lastHeartbeatMs = -1000L

    while ($timer.Elapsed.TotalSeconds -lt $DurationSeconds) {
        if ($Arm -and (($timer.ElapsedMilliseconds - $lastHeartbeatMs) -ge 500L)) {
            $serial.WriteLine('PING')
            $lastHeartbeatMs = $timer.ElapsedMilliseconds
        }

        try {
            $line = $serial.ReadLine().Trim()
        } catch [System.TimeoutException] {
            continue
        }

        if ($line.StartsWith('DATA,')) {
            $fields = $line.Split(',')
            if ($fields.Count -eq 15) {
                $writer.WriteLine(($fields[1..14] -join ','))
                $writer.Flush()
                Write-Host ("t={0,6} target={1,5} L={2,5} R={3,5} eL={4,5} eR={5,5} out={6,3}/{7,3}" -f
                    $fields[1], $fields[5], $fields[6], $fields[7],
                    $fields[8], $fields[9], $fields[10], $fields[11])
            }
        } else {
            Write-Host $line
        }
    }
} finally {
    if (($serial -ne $null) -and $serial.IsOpen) {
        if ($armedByScript) {
            try { $serial.WriteLine('STOP') } catch {}
        }
        $serial.Close()
    }
    if ($writer -ne $null) {
        $writer.Dispose()
    }
}

Write-Host "Capture saved to $OutputPath"
