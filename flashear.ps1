<#
    flashear.ps1 - Compila y graba el firmware de una placa del ASPID.

    Comprueba el Device ID de la placa conectada ANTES de escribir, para no
    grabar el firmware del core (H7) en una G4 ni al contrario.

    Uso:
        .\flashear.ps1 core
        .\flashear.ps1 power -Preset Release
        .\flashear.ps1 rf -SoloCompilar
        .\flashear.ps1 avionica -SinGrabar
        .\flashear.ps1 -Listar
#>

[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [string] $Placa,

    [ValidateSet('Debug', 'Release')]
    [string] $Preset = 'Debug',

    # Compila pero no graba.
    [switch] $SoloCompilar,

    # Graba lo ya compilado, sin recompilar.
    [switch] $SinCompilar,

    # Compila y comprueba el Device ID, pero no graba (lo usa la sesión de
    # debug de VS Code, que graba con el ST-LINK_gdbserver).
    [switch] $SinGrabar,

    # Graba aunque el Device ID no sea el esperado.
    [switch] $Forzar,

    # Lista las placas y sale.
    [switch] $Listar
)

$ErrorActionPreference = 'Stop'
$raiz = $PSScriptRoot

# Device ID: 0x483 = STM32H723 (core), 0x469 = STM32G473 (las tres G4).
$placas = [ordered]@{
    'core'     = @{ Carpeta = 'core_h7_v0';              Elf = 'core_h7_v0.elf';              DeviceId = '0x483'; Que = 'Ordenador de vuelo (STM32H723)' }
    'power'    = @{ Carpeta = 'power_g4_v0';             Elf = 'Power_MK2.elf';               DeviceId = '0x469'; Que = 'Placa de potencia (STM32G473)' }
    'avionica' = @{ Carpeta = 'sensores_avionica_g4_v0'; Elf = 'sensores_avionica_g4_v0.elf'; DeviceId = '0x469'; Que = 'Aviónica / sensores (STM32G473)' }
    'rf'       = @{ Carpeta = 'rf_g4_v0';                Elf = 'rf_g4_v0.elf';                DeviceId = '0x469'; Que = 'Placa de radio (STM32G473 + SX1262)' }
}

# Windows PowerShell 5.1 convierte el stderr de un .exe en error terminante
# cuando $ErrorActionPreference es 'Stop' (cmake escribe avisos normales por
# stderr). Por eso las llamadas a .exe van siempre por aquí.
function Invoke-Nativo {
    param(
        [Parameter(Mandatory)] [string]   $Exe,
        [string[]] $Argumentos = @(),
        [switch]   $Capturar
    )
    $previo = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        if ($Capturar) { return (& $Exe @Argumentos 2>&1 | Out-String) }
        # 2>&1 trae el stderr al pipeline como ErrorRecord; lo pasamos a texto
        # para que los avisos normales de cmake no parezcan errores de PowerShell.
        & $Exe @Argumentos 2>&1 | ForEach-Object {
            if ($_ -is [System.Management.Automation.ErrorRecord]) { $_.ToString() } else { $_ }
        } | Out-Host
        return $null
    }
    finally { $ErrorActionPreference = $previo }
}

function Mostrar-Placas {
    Write-Host ''
    Write-Host '  Placas:' -ForegroundColor Cyan
    foreach ($k in $placas.Keys) {
        '    {0,-9} {1,-40} Device ID {2}' -f $k, $placas[$k].Que, $placas[$k].DeviceId | Write-Host
    }
    Write-Host ''
}

if ($Listar -or -not $Placa) {
    Mostrar-Placas
    if (-not $Placa) { Write-Host '  Falta el nombre de la placa.' -ForegroundColor Yellow }
    exit 0
}

$clave = $Placa.ToLowerInvariant()
if (-not $placas.Contains($clave)) {
    Write-Host ("  '{0}' no es una placa conocida." -f $Placa) -ForegroundColor Red
    Mostrar-Placas
    exit 1
}

$cfg      = $placas[$clave]
$carpeta  = Join-Path $raiz $cfg.Carpeta
$buildDir = Join-Path $carpeta "build\$Preset"
$elf      = Join-Path $buildDir $cfg.Elf

Write-Host ''
Write-Host ("  === {0} [{1}] ===" -f $cfg.Que, $Preset) -ForegroundColor Cyan

# ---------------------------------------------------------------- compilar
if (-not $SinCompilar) {
    Write-Host '  Compilando...' -ForegroundColor Gray
    Push-Location $carpeta
    try {
        Invoke-Nativo 'cmake' @('--preset', $Preset)
        if ($LASTEXITCODE -ne 0) { throw "cmake --preset $Preset ha fallado." }
        Invoke-Nativo 'cmake' @('--build', "build/$Preset")
        if ($LASTEXITCODE -ne 0) { throw 'La compilación ha fallado.' }
    }
    finally { Pop-Location }
}

if (-not (Test-Path $elf)) { throw "No existe el .elf: $elf" }
Write-Host ("  .elf: {0}" -f $elf) -ForegroundColor Gray

if ($SoloCompilar) {
    Write-Host '  Compilado. No se graba (-SoloCompilar).' -ForegroundColor Green
    Write-Host ''
    exit 0
}

# -------------------------------------------------------------- programador
$prog = (Get-Command 'STM32_Programmer_CLI' -ErrorAction SilentlyContinue).Source
if (-not $prog) {
    $cand = Get-ChildItem 'C:\ST\STM32CubeCLT_*\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue |
            Sort-Object FullName -Descending | Select-Object -First 1
    if ($cand) { $prog = $cand.FullName }
}
if (-not $prog) { throw 'No se encuentra STM32_Programmer_CLI. Instala STM32CubeCLT o añádelo al PATH.' }

# ------------------------------------------------- comprobar el Device ID
Write-Host '  Conectando con la placa...' -ForegroundColor Gray
$info = Invoke-Nativo $prog @('-c', 'port=SWD', 'mode=UR') -Capturar

if ($info -notmatch 'Device ID\s*:\s*(0x[0-9A-Fa-f]+)') {
    Write-Host $info
    throw 'No se ha podido leer el Device ID. ¿Está el ST-Link conectado y la placa alimentada?'
}

$leido    = $Matches[1].ToLowerInvariant()
$esperado = $cfg.DeviceId.ToLowerInvariant()

if ($leido -ne $esperado) {
    Write-Host ''
    Write-Host ("  PLACA EQUIVOCADA: esperaba {0} ({1}) y hay {2} conectada." -f $esperado, $clave, $leido) -ForegroundColor Red
    if (-not $Forzar) {
        Write-Host '  No se graba nada. Usa -Forzar si de verdad quieres escribir.' -ForegroundColor Yellow
        Write-Host ''
        exit 1
    }
    Write-Host '  -Forzar: se graba de todas formas.' -ForegroundColor Yellow
}
else {
    Write-Host ("  Device ID {0}, correcto." -f $leido) -ForegroundColor Green
}

if ($SinGrabar) {
    Write-Host '  No se graba (-SinGrabar).' -ForegroundColor Green
    Write-Host ''
    exit 0
}

# ------------------------------------------------------------------ grabar
Write-Host '  Grabando...' -ForegroundColor Gray
Invoke-Nativo $prog @('-c', 'port=SWD', 'mode=UR', '-w', $elf, '-v', '-rst')
if ($LASTEXITCODE -ne 0) { throw 'La grabación ha fallado.' }

Write-Host ''
Write-Host ("  {0} grabada y reiniciada." -f $cfg.Que) -ForegroundColor Green
Write-Host ''
