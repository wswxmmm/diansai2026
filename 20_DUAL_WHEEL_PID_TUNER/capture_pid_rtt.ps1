param(
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

    [ValidateRange(1, 65535)]
    [int]$RttTelnetPort = 19021,

    [switch]$AutoStep,
    [switch]$Arm,
    [string]$OutputPath
)

$ErrorActionPreference = 'Stop'

if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
    $OutputPath = Join-Path $PSScriptRoot "pid-rtt-capture-$stamp.csv"
}

$tcpClient = [System.Net.Sockets.TcpClient]::new()
$stream = $null
$writer = $null
$armedByScript = $false
$ascii = [System.Text.Encoding]::ASCII

function Send-RttLine {
    param([Parameter(Mandatory = $true)][string]$Text)

    if (($stream -ne $null) -and $stream.CanWrite) {
        $bytes = $ascii.GetBytes("$Text`r`n")
        $stream.Write($bytes, 0, $bytes.Length)
        $stream.Flush()
    }
}

try {
    $connectTask = $tcpClient.ConnectAsync('127.0.0.1', $RttTelnetPort)
    if (-not $connectTask.Wait(5000)) {
        throw "Timed out connecting to the CCS RTT server on port $RttTelnetPort."
    }
    [void]$connectTask.GetAwaiter().GetResult()

    $stream = $tcpClient.GetStream()
    $stream.WriteTimeout = 500
    $readBytes = [byte[]]::new(4096)

    # Discard buffered telemetry so the capture starts at the current controller state.
    $drainTimer = [System.Diagnostics.Stopwatch]::StartNew()
    while ($drainTimer.ElapsedMilliseconds -lt 750) {
        while ($stream.DataAvailable) {
            $count = $stream.Read($readBytes, 0, $readBytes.Length)
            if ($count -eq 0) {
                throw 'The CCS RTT server closed the connection while draining data.'
            }
        }
        Start-Sleep -Milliseconds 10
    }

    $writer = [System.IO.StreamWriter]::new($OutputPath, $false)
    $writer.WriteLine('ms,run,auto,target,ramp_target_pps,left_pps,right_pps,left_error,right_error,left_output_permille,right_output_permille,kp_x1000,ki_x1000,kd_x1000')

    Send-RttLine 'GET'
    Send-RttLine 'PING'
    $connectTimer = [System.Diagnostics.Stopwatch]::StartNew()
    $captureTimer = $null
    $firmwareSeen = $false
    $commandsSent = $false
    $lastHeartbeatMs = -1000L
    $pendingText = ''

    while ($true) {
        if (-not $tcpClient.Connected) {
            throw 'The CCS RTT server disconnected.'
        }
        if (-not $firmwareSeen -and $connectTimer.Elapsed.TotalSeconds -ge 10) {
            throw 'No PID firmware RTT data was found within 10 seconds.'
        }
        if ($firmwareSeen -and
            ($captureTimer.Elapsed.TotalSeconds -ge $DurationSeconds)) {
            break
        }

        if ($armedByScript -and
            (($captureTimer.ElapsedMilliseconds - $lastHeartbeatMs) -ge 250L)) {
            Send-RttLine 'PING'
            $lastHeartbeatMs = $captureTimer.ElapsedMilliseconds
        }

        if (-not $stream.DataAvailable) {
            Start-Sleep -Milliseconds 10
            continue
        }

        $count = $stream.Read($readBytes, 0, $readBytes.Length)
        if ($count -eq 0) {
            throw 'The CCS RTT server closed the connection.'
        }
        $pendingText += $ascii.GetString($readBytes, 0, $count)
        $lines = $pendingText -split "`n"
        $pendingText = $lines[-1]
        if ($lines.Count -le 1) {
            continue
        }

        foreach ($rawLine in $lines[0..($lines.Count - 2)]) {
            $line = $rawLine.Trim()
            $isFirmwareLine =
                $line.StartsWith('READY,') -or
                $line.StartsWith('STATUS,') -or
                $line.StartsWith('DATA,') -or
                $line.StartsWith('OK,') -or
                $line.StartsWith('ERR,') -or
                $line.StartsWith('PONG') -or
                $line.StartsWith('FAULT,')

            if (-not $isFirmwareLine) {
                continue
            }

            if (-not $firmwareSeen) {
                $firmwareSeen = $true
                $captureTimer = [System.Diagnostics.Stopwatch]::StartNew()
                Write-Host 'Connected to PID firmware over J-Link RTT.'
            }

            if ($Arm -and (-not $commandsSent)) {
                Send-RttLine "SET,KP,$Kp"
                Start-Sleep -Milliseconds 50
                Send-RttLine "SET,KI,$Ki"
                Start-Sleep -Milliseconds 50
                Send-RttLine "SET,KD,$Kd"
                Start-Sleep -Milliseconds 50
                Send-RttLine 'RESET'
                Start-Sleep -Milliseconds 50
                if ($AutoStep) {
                    Send-RttLine 'AUTO,1'
                } else {
                    Send-RttLine "SET,TARGET,$Target"
                    Start-Sleep -Milliseconds 50
                    Send-RttLine 'RUN'
                }
                Start-Sleep -Milliseconds 50
                Send-RttLine 'PING'
                $commandsSent = $true
                $armedByScript = $true
                $lastHeartbeatMs = $captureTimer.ElapsedMilliseconds
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
    }
} finally {
    if (($stream -ne $null) -and $stream.CanWrite -and $armedByScript) {
        try {
            Send-RttLine 'STOP'
            Start-Sleep -Milliseconds 100
        } catch {
        }
    }
    if ($writer -ne $null) {
        $writer.Dispose()
    }
    if ($stream -ne $null) {
        $stream.Dispose()
    }
    $tcpClient.Dispose()
}

Write-Host "Capture saved to $OutputPath"
