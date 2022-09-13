file_names = ["/workspace/Moose/src/main/java/frc/robot/RobotContainer.java", "/workspace/Moose/src/main/java/frc/robot/RobotMap.java", "/workspace/Moose/src/main/java/frc/robot/Main.java", "/workspace/Moose/src/main/java/frc/robot/utils/Epsilon.java", "/workspace/Moose/src/main/java/frc/robot/utils/LinearInterp.java", "/workspace/Moose/src/main/java/frc/robot/utils/PIDCustom.java", "/workspace/Moose/src/main/java/frc/robot/utils/Utils.java", "/workspace/Moose/src/main/java/frc/robot/SupportingClassess/AutonCommand.java", "/workspace/Moose/src/main/java/frc/robot/SupportingClassess/Chooser.java", "/workspace/Moose/src/main/java/frc/robot/SupportingClassess/GeneralUtils.java", "/workspace/Moose/src/main/java/frc/robot/subsystems/Chassis.java", "/workspace/Moose/src/main/java/frc/robot/subsystems/Climber.java", "/workspace/Moose/src/main/java/frc/robot/subsystems/ExampleSubsystem.java", "/workspace/Moose/src/main/java/frc/robot/subsystems/Hood.java", "/workspace/Moose/src/main/java/frc/robot/subsystems/Intake.java", "/workspace/Moose/src/main/java/frc/robot/subsystems/Magazine.java", "/workspace/Moose/src/main/java/frc/robot/subsystems/Shooter.java", "/workspace/Moose/src/main/java/frc/robot/sensors/vision/Algebra.java", "/workspace/Moose/src/main/java/frc/robot/sensors/vision/Limelight.java", "/workspace/Moose/src/main/java/frc/robot/sensors/vision/WheelSpeedCalculations.java", "/workspace/Moose/src/main/java/frc/robot/sensors/Navx.java", "/workspace/Moose/src/main/java/frc/robot/controls/JoystickTrigger.java", "/workspace/Moose/src/main/java/frc/robot/controls/PivotTrigger.java", "/workspace/Moose/src/main/java/frc/robot/controls/POVTrigger.java", "/workspace/Moose/src/main/java/frc/robot/controls/TriggerButton.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/DefaultDrive.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/FaceTarget.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/Shift.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/SpinChassisToAbsoluteAngle.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/SpinChassisToAngle.java", "/workspace/Moose/src/main/java/frc/robot/controls/JoystickTrigger.java", "/workspace/Moose/src/main/java/frc/robot/controls/PivotTrigger.java", "/workspace/Moose/src/main/java/frc/robot/controls/POVTrigger.java", "/workspace/Moose/src/main/java/frc/robot/controls/TriggerButton.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/DefaultDrive.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/FaceTarget.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/Shift.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/SpinChassisToAbsoluteAngle.java", "/workspace/Moose/src/main/java/frc/robot/commands/Chassis/SpinChassisToAngle.java", "/workspace/Moose/src/main/java/frc/robot/commands/Climber/ExtendArmsIGuess.java", "/workspace/Moose/src/main/java/frc/robot/commands/Climber/spinClimberWinches.java", "/workspace/Moose/src/main/java/frc/robot/commands/Climber/ToggleClimber.java", "/workspace/Moose/src/main/java/frc/robot/commands/Climber/ZeroClimber.java", "/workspace/Moose/src/main/java/frc/robot/commands/Intake/DeployAndSpintake.java", "/workspace/Moose/src/main/java/frc/robot/commands/Intake/DeployAndSpintakeMagazineBack.java", "/workspace/Moose/src/main/java/frc/robot/commands/Intake/DeployIntake.java", "/workspace/Moose/src/main/java/frc/robot/commands/Intake/Spintake.java", "/workspace/Moose/src/main/java/frc/robot/commands/Intake/TimedSpintake.java", "/workspace/Moose/src/main/java/frc/robot/commands/Magazine/Spinzine.java", "/workspace/Moose/src/main/java/frc/robot/commands/Shooter/BenShoot.java", "/workspace/Moose/src/main/java/frc/robot/commands/Shooter/ChooseFlywheelRPM.java", "/workspace/Moose/src/main/java/frc/robot/commands/Shooter/SetFlywheelRPM.java", "/workspace/Moose/src/main/java/frc/robot/commands/Shooter/Shoot.java", "/workspace/Moose/src/main/java/frc/robot/commands/Shooter/SpindexTimed.java", "/workspace/Moose/src/main/java/frc/robot/commands/doNothing.java", "/workspace/Moose/src/main/java/frc/robot/commands/ExampleCommand.java"]
#file_names.append(input(": "))

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