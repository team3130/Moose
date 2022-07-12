file_names = ["commands/Chassis/DefaultDrive.java", "commands/Chassis/FaceTarget.java", "commands/Chassis/Shift.java", "commands/Chassis/SpinChassisToAbsoluteAngle.java", "commands/Chassis/SpinChassisToAngle.java", "commands/Climber/ExtendArmsIGuess.java", "commands/Climber/spinClimberWinches.java", "commands/Climber/ToggleClimber.java", "commands/Climber/ZeroClimber.java", "commands/Intake/DeployAndSpintake.java", "commands/Intake/DeployAndSpintakeMagazineBack.java", "commands/Intake/DeployIntake.java", "commands/Intake/Spintake.java", "commands/Intake/TimedSpintake.java", "commands/Magazine/Spinzine.java", "commands/Shooter/BenShoot.java", "commands/Shooter/ChooseFlywheelRPM.java", "commands/Shooter/SetFlywheelRPM.java", "commands/Shooter/Shoot.java", "commands/Shooter/SpindevTimed.java", "commands/doNothing.java", "commands/ExampleCommand.java", "controls/JoystickTrigger.java", "controls/PivotTrigger.java", "controls/POVTrigger.java", "controls/TriggerButton.java", "sensors/vision/Algebra.java", "sensors/vision/Limelight.java", "sensors/vision/WheelSpeedCalculations.java", "sensors/Navx.java", "subsystems/Chassis.java", "subsystems/Climber.java", "subsystems/ExampleSubsystem.java", "subsystems/Hood.java", "subsystems/Intake.java", "subsystems/Magazine.java", "subsystems/Shooter.java", "SupportingClasses/AutonCommand.java", "SupportingClasses/Chooser.java", "SupportingClasses/GeneralUtils.java", "utils/Epsilon.java", "utils/LinearInterp.java", "utils/PIDCustom.java", "utils/Utils.java", "Main.java", "Robot.java", "RobotContainer.java", "RobotMap.java"]

for i in range(len(file_names)):
    file = open(file_names[i], "r")
    lines = file.readlines()
    file.close()

    code = ""

    for line in range(len(lines)):
        lines[line] = lines[line].strip()
        for char in range(len(lines[line])):
            try:
                if lines[line][char] == "/":
                    if lines[line][char + 1] == "/":
                        lines[line] = lines[line][0:char]
                elif lines[line][char] == "@":
                    if lines[line][char + 1] == "O":
                        if lines[line][char + 2] == "v":
                            if lines[line][char + 3] == "e":
                                if lines[line][char + 4] == "r":
                                    if lines[line][char + 5] == "r":
                                        if lines[line][char + 6] == "i":
                                            if lines[line][char + 7] == "d":
                                                if lines[line][char + 8] == "e":
                                                    lines[line] += " "
            except IndexError:
                pass
        code += lines[line]

    file = open(file_names[i], "w")
    file.write(code)
    file.close()