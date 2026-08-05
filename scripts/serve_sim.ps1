# Serve ViDAR browser sim at http://127.0.0.1:8765
$Port = 8765
$Root = Join-Path $PSScriptRoot "..\sim" | Resolve-Path
$Captures = Join-Path $PSScriptRoot "..\captures" | Resolve-Path -ErrorAction SilentlyContinue
if (-not $Captures) {
    $Captures = Join-Path $PSScriptRoot "..\captures"
    New-Item -ItemType Directory -Force -Path $Captures | Out-Null
    $Captures = (Resolve-Path $Captures).Path
}

$listener = New-Object System.Net.HttpListener
$listener.Prefixes.Add("http://127.0.0.1:$Port/")
$listener.Start()

Write-Host "ViDAR sim -> http://127.0.0.1:$Port"
Write-Host "Serving: $Root"
Write-Host "Captures: $Captures"
Write-Host "Press Ctrl+C to stop"

$mime = @{
    ".html" = "text/html; charset=utf-8"
    ".css"  = "text/css; charset=utf-8"
    ".js"   = "text/javascript; charset=utf-8"
    ".json" = "application/json; charset=utf-8"
    ".png"  = "image/png"
    ".ico"  = "image/x-icon"
}

function Write-BytesResponse($response, $statusCode, $contentType, $bytes) {
    $response.StatusCode = $statusCode
    if ($contentType) { $response.ContentType = $contentType }
    if ($bytes) {
        $response.ContentLength64 = $bytes.Length
        $response.OutputStream.Write($bytes, 0, $bytes.Length)
    }
    $response.Close()
}

function Decode-DataUrl($dataUrl) {
    if (-not $dataUrl) { return $null }
    $comma = $dataUrl.IndexOf(",")
    if ($comma -lt 0) { return $null }
    return [Convert]::FromBase64String($dataUrl.Substring($comma + 1))
}

try {
    while ($listener.IsListening) {
        $context = $listener.GetContext()
        $request = $context.Request
        $response = $context.Response
        $path = $request.Url.LocalPath

        if ($request.HttpMethod -eq "POST" -and $path -eq "/api/capture") {
            try {
                $reader = New-Object System.IO.StreamReader($request.InputStream, $request.ContentEncoding)
                $body = $reader.ReadToEnd()
                $reader.Close()
                $payload = $body | ConvertFrom-Json
                $id = if ($payload.timestamp) { $payload.timestamp } else { Get-Date -Format "yyyy-MM-ddTHH-mm-ss" }

                $frameBytes = Decode-DataUrl $payload.frame
                if (-not $frameBytes) { throw "Missing frame image" }

                [IO.File]::WriteAllBytes((Join-Path $Captures "$id.png"), $frameBytes)
                [IO.File]::WriteAllBytes((Join-Path $Captures "latest.png"), $frameBytes)

                if ($payload.process) {
                    $pb = Decode-DataUrl $payload.process
                    if ($pb) {
                        [IO.File]::WriteAllBytes((Join-Path $Captures "$id-process.png"), $pb)
                        [IO.File]::WriteAllBytes((Join-Path $Captures "latest-process.png"), $pb)
                    }
                }

                if ($payload.mask) {
                    $mb = Decode-DataUrl $payload.mask
                    if ($mb) {
                        [IO.File]::WriteAllBytes((Join-Path $Captures "$id-mask.png"), $mb)
                        [IO.File]::WriteAllBytes((Join-Path $Captures "latest-mask.png"), $mb)
                    }
                }

                if ($payload.meta) {
                    $metaJson = $payload.meta | ConvertTo-Json -Depth 8
                    [IO.File]::WriteAllText((Join-Path $Captures "$id.json"), $metaJson)
                    [IO.File]::WriteAllText((Join-Path $Captures "latest.json"), $metaJson)
                }

                $ok = [Text.Encoding]::UTF8.GetBytes(('{"ok":true,"id":"' + $id + '"}'))
                Write-BytesResponse $response 200 "application/json; charset=utf-8" $ok
            } catch {
                $err = [Text.Encoding]::UTF8.GetBytes('{"ok":false,"error":"' + ($_.Exception.Message -replace '"', "'") + '"}')
                Write-BytesResponse $response 500 "application/json; charset=utf-8" $err
            }
            continue
        }

        $rel = [Uri]::UnescapeDataString($path.TrimStart("/"))
        if ([string]::IsNullOrWhiteSpace($rel)) { $rel = "index.html" }

        $file = Join-Path $Root $rel
        $file = [System.IO.Path]::GetFullPath($file)

        if (-not $file.StartsWith($Root.Path, [StringComparison]::OrdinalIgnoreCase)) {
            Write-BytesResponse $response 403 $null $null
            continue
        }

        if (Test-Path $file -PathType Leaf) {
            $ext = [System.IO.Path]::GetExtension($file).ToLowerInvariant()
            $bytes = [System.IO.File]::ReadAllBytes($file)
            $ctype = $null
            if ($mime.ContainsKey($ext)) { $ctype = $mime[$ext] }
            Write-BytesResponse $response 200 $ctype $bytes
        } else {
            Write-BytesResponse $response 404 $null $null
        }
    }
} finally {
    $listener.Stop()
}
