param(
    [ValidateRange(1, 65535)]
    [int]$Port = 2331
)

$ErrorActionPreference = 'Stop'
$ascii = [System.Text.Encoding]::ASCII
$client = [System.Net.Sockets.TcpClient]::new('127.0.0.1', $Port)
$stream = $client.GetStream()
$stream.ReadTimeout = 250

function Send-GdbPacket {
    param([Parameter(Mandatory = $true)][string]$Payload)

    $checksum = 0
    foreach ($value in $ascii.GetBytes($Payload)) {
        $checksum = ($checksum + $value) -band 0xFF
    }
    $message = '$' + $Payload + '#' + $checksum.ToString('x2')
    $bytes = $ascii.GetBytes($message)
    $stream.Write($bytes, 0, $bytes.Length)
    $stream.Flush()
}

function Read-GdbPacket {
    $payload = [System.Text.StringBuilder]::new()
    $inPacket = $false

    while ($true) {
        try {
            $value = $stream.ReadByte()
        } catch [System.IO.IOException] {
            return $null
        }
        if ($value -lt 0) {
            throw 'J-Link GDB Server closed the connection.'
        }
        $character = [char]$value
        if (-not $inPacket) {
            if ($character -eq '$') {
                $inPacket = $true
                [void]$payload.Clear()
            }
            continue
        }
        if ($character -eq '#') {
            [void]$stream.ReadByte()
            [void]$stream.ReadByte()
            $ack = [byte[]](43)
            $stream.Write($ack, 0, 1)
            return $payload.ToString()
        }
        [void]$payload.Append($character)
    }
}

try {
    Send-GdbPacket 'qSupported'
    while ($null -eq (Read-GdbPacket)) {
    }
    Send-GdbPacket '?'
    while ($null -eq (Read-GdbPacket)) {
    }

    # Clear hardware breakpoints left behind by a previous CCS session.
    foreach ($address in 'e0002008', 'e000200c', 'e0002010', 'e0002014') {
        Send-GdbPacket "M$address,4:00000000"
        while ($null -eq (Read-GdbPacket)) {
        }
    }
    Send-GdbPacket 'c'

    while ($true) {
        $packet = Read-GdbPacket
        if (($packet -ne $null) -and
            ($packet.StartsWith('S') -or $packet.StartsWith('T'))) {
            Start-Sleep -Milliseconds 50
            Send-GdbPacket 'c'
        }
    }
} finally {
    $stream.Dispose()
    $client.Dispose()
}
