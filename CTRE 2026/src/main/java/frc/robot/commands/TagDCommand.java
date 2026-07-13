// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.commands.ChaseAprilTagCommand;


/** An example command that uses an example subsystem. */
public class TagDCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final LimelightSubsystem limelight;
    private Command activeCommand;
       @SuppressWarnings("PMD.UnusedPrivateField")
/**
   * Creates a new TagFinderCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
    public TagDCommand(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
        this.drivetrain = drivetrain;
        this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, limelight);
    }

  // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        activeCommand = null;
        var visionData = limelight.getRawVisionMeasurement();
        if (visionData == null || visionData.tagCount < 1){
            return;
        }

        int seenID = (int) limelight.getID();

        Command chaseCmd;
        switch (seenID) {
            case 10:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    10,
                    2.0, // 2.0 TZ
                    0.0  , // 0.0 TX
                    0.0); //0.0 RY
                break;

            case 11:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    11,
                    1.24,
                    -1.10,
                    edu.wpi.first.math.util.Units.degreesToRadians(45)
                );
                break;
            case 8:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    8,
                    1.64, // TZ
                    .70, //  TX
                    edu.wpi.first.math.util.Units.degreesToRadians(-25) // RY
                );
                break;
            case 12:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    12,
                    1.24,
                    0,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            case 6:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    8,
                    1.24,
                    0,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            case 1:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    1,
                    1.24,
                    0,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            case 17:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    17,
                    1.24,
                    0,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            case 22:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    22,
                    1.24,
                    0,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            case 26:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    26,
                    2.0,
                    0.0,
                    0.0);
                break;
            case 28:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    28,
                    1.24,
                    0,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            case 23:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    23,
                    1.24,
                    0,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            case 24:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    24,
                    1.24,
                    1.10,
                    edu.wpi.first.math.util.Units.degreesToRadians(-41.5)
                );
                break;
            case 27:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    27,
                    1.24,
                    -1.10,
                    edu.wpi.first.math.util.Units.degreesToRadians(41.5)
                );
                break;
            case 16:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    16,
                    0.7303+0.0508+0.3,
                    0.26,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
          case 32:
                chaseCmd = new ChaseAprilTagCommand(
                    drivetrain,
                    limelight,
                    32,
                    0.7303+0.0508+0.3,
                    0.26,
                    edu.wpi.first.math.util.Units.degreesToRadians(0)
                );
                break;
            default:
                chaseCmd = null;
                break;
        }
        if (chaseCmd !=null){
            activeCommand = chaseCmd;
            activeCommand.initialize();
        }

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (activeCommand != null){
            activeCommand.execute();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (activeCommand !=null){
            activeCommand.end(interrupted);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (activeCommand == null){
            return true;
        }
        return activeCommand.isFinished();
  }
}