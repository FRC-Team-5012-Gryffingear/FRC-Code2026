// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ChaseAprilTagCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SwerveSubsystem swerve;
  private final LimelightSubsystem limelight;
  private final int targetTagID;
  private final double forwardOffset;
  private final double horizontalOffset;

  // ProfiledPIDControllers (separate for each axis like in video)
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rotController;
    
    // Constraints for smooth acceleration profiles
    private static final double MAX_VELOCITY_MPS = 2.0;
    private static final double MAX_ACCELERATION_MPS2 = 1.0;
    private static final double MAX_OMEGA_RPS = Math.PI;
    private static final double MAX_ALPHA_RPS2 = Math.PI;
    
    // PID gains (tune these!)
    private static final double KP_LINEAR = 2.0;
    private static final double KD_LINEAR = 0.06;
    private static final double KP_ANGULAR = 2.8;
    private static final double KD_ANGULAR = 0.23;
    
    // Tolerances
    private static final double TOLERANCE_LINEAR = 0.15;  // 5cm
    private static final double TOLERANCE_ANGULAR = edu.wpi.first.math.util.Units.degreesToRadians(4);  // 2°
    private static final double TIMEOUT_SECONDS = 5.0;
    
    private double startTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChaseAprilTagCommand(SwerveSubsystem subsystem, LimelightSubsystem lime, int tagID, double forwardDistance, double horizontalDistance) {
    swerve = subsystem;
    limelight = lime;
    targetTagID = tagID;
    forwardOffset = forwardDistance;
    horizontalOffset = horizontalDistance;

    TrapezoidProfile.Constraints linearConstraints =
            new TrapezoidProfile.Constraints(MAX_VELOCITY_MPS, MAX_ACCELERATION_MPS2);
        TrapezoidProfile.Constraints angularConstraints =
            new TrapezoidProfile.Constraints(MAX_OMEGA_RPS, MAX_ALPHA_RPS2);
        
        xController = new ProfiledPIDController(KP_LINEAR, 0, KD_LINEAR, linearConstraints);
        yController = new ProfiledPIDController(KP_LINEAR, 0, KD_LINEAR, linearConstraints);
        rotController = new ProfiledPIDController(KP_ANGULAR, 0, KD_ANGULAR, angularConstraints);
        
        // Set tolerances (when to consider at goal)
        xController.setTolerance(TOLERANCE_LINEAR);
        yController.setTolerance(TOLERANCE_LINEAR);
        rotController.setTolerance(TOLERANCE_ANGULAR);
        
        // Rotation wraps around (can go -180° to +180°)
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, lime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     var visionData = limelight.getRawVisionMeasurement();
        if (visionData == null || visionData.tagCount < 1) {
            // No tag visible - stop moving
            System.out.println("❌ No tags visible");
            swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
            return;
        }
        // Use ProfiledPIDControllers to calculate velocities (like in video)
        double vx = xController.calculate(limelight.getZ() + forwardOffset,0);
        double vy = yController.calculate(limelight.getX() - horizontalOffset, 0);
        double omega = rotController.calculate(
            limelight.getYaw(), 0
        );
        
        // Clamp to max speeds
        vx = Math.max(-MAX_VELOCITY_MPS, Math.min(MAX_VELOCITY_MPS, vx));
        vy = Math.max(-MAX_VELOCITY_MPS, Math.min(MAX_VELOCITY_MPS, vy));
        omega = Math.max(-MAX_OMEGA_RPS, Math.min(MAX_OMEGA_RPS, omega));
        
        // Send to drivetrain
        swerve.getSwerveDrive().drive(new ChassisSpeeds(vx, -vy, -omega));
    
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
