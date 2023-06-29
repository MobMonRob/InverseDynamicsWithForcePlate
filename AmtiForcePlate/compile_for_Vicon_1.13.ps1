param(
    [string]$Path = "/home/deralbert/Desktop/BA/Code/InverseDynamicsWithForcePlate/AmtiForcePlate/compile_for_Vicon_1.13.sh"
)

# Check if the script file exists
if (Test-Path $Path -PathType Leaf) {
    # Execute the script
    & bash $Path
}
else {
    Write-Host "Script file not found: $Path"
}
