// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ChaseAprilTagCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final CommandSwerveDrivetrain drivetrain;
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
    private static final double KD_ANGULAR = 0.03;
    
    // Tolerances
    private static final double TOLERANCE_FORWARD = 0.08;  // 5cm
    private static final double TOLERANCE_HORIZONTAL = 0.03;
    private static final double TOLERANCE_ANGULAR = edu.wpi.first.math.util.Units.degreesToRadians(3);  // 2°
    private static final double TIMEOUT_SECONDS = 7.0;
    SwerveRequest m_zeroRequest = new SwerveRequest.RobotCentric()
    .withVelocityX(0)
    .withVelocityY(0)
    .withRotationalRate(0);

    
    private double startTime;
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ChaseAprilTagCommand(CommandSwerveDrivetrain subsystem, LimelightSubsystem lime, int tagID, double forwardDistance, double horizontalDistance) {
    drivetrain = subsystem;
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
        xController.setTolerance(TOLERANCE_FORWARD);
        yController.setTolerance(TOLERANCE_HORIZONTAL);
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
        if ((visionData == null || visionData.tagCount < 1) && LimelightHelpers.getFiducialID("limelight-calvin") == targetTagID) {
            // No tag visible - stop moving
            System.out.println("No tags visible");
            drivetrain.setControl(m_zeroRequest);
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
        SwerveRequest.RobotCentric chaseReq = new SwerveRequest.RobotCentric()
        .withVelocityX(vx)  // Forward
        .withVelocityY(-vy)  // Left positive in WPILib/Limelight
        .withRotationalRate(-omega);  // Tune signs based on testing

        drivetrain.setControl(chaseReq);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(m_zeroRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsed = Timer.getFPGATimestamp() - startTime;
        
        boolean atGoal = xController.atSetpoint() && 
                        yController.atSetpoint() && 
                        rotController.atSetpoint();
        
        boolean timedOut = elapsed > TIMEOUT_SECONDS;
        
        if (atGoal) System.out.println("✅ Reached goal!");
        if (timedOut) System.out.println("⏱️ Timeout!");
        
        return atGoal || timedOut;
  }
}