param([Parameter(Mandatory=$true)][string]$OutputFile)
$ErrorActionPreference = "Stop"
$recognizer = $null

function Write-Result([string]$Text) {
    $encoding = New-Object System.Text.UTF8Encoding($false)
    [System.IO.File]::WriteAllText($OutputFile, $Text, $encoding)
}

try {
    Add-Type -AssemblyName System.Speech
    $installed = @([System.Speech.Recognition.SpeechRecognitionEngine]::InstalledRecognizers())
    $selected = $installed | Where-Object { $_.Culture.Name -eq "en-US" } | Select-Object -First 1
    if ($null -eq $selected) {
        $selected = $installed | Where-Object { $_.Culture.Name -like "en-*" } | Select-Object -First 1
    }
    if ($null -eq $selected) {
        Write-Result "ERROR: No English Windows speech recognizer is installed. Open Settings > Time & language > Speech and install an English speech language."
        exit 1
    }

    $recognizer = [System.Speech.Recognition.SpeechRecognitionEngine]::new($selected)
    $recognizer.SetInputToDefaultAudioDevice()

    # Exactly the seven requested spoken commands; no extra command vocabulary.
    $phrases = @("take off", "turn left", "turn right", "backward", "hover", "land", "return to home")
    $choices = New-Object System.Speech.Recognition.Choices
    foreach ($phrase in $phrases) { [void]$choices.Add($phrase) }
    $builder = [System.Speech.Recognition.GrammarBuilder]::new()
    $builder.Culture = $selected.Culture
    $builder.Append($choices)
    $grammar = [System.Speech.Recognition.Grammar]::new($builder)
    $grammar.Name = "DroneFlightCommands"
    $recognizer.LoadGrammar($grammar)

    $result = $recognizer.Recognize([TimeSpan]::FromSeconds(10))
    if ($null -ne $result -and -not [string]::IsNullOrWhiteSpace($result.Text)) {
        Write-Result $result.Text
        exit 0
    }
    Write-Result "NO_MATCH: No command recognized in 10 seconds. Click LISTEN TO VOICE again and speak one of the seven commands clearly."
    exit 2
}
catch {
    try { Write-Result ("ERROR: " + $_.Exception.GetBaseException().Message) } catch {}
    exit 1
}
finally {
    if ($null -ne $recognizer) { $recognizer.Dispose() }
}
