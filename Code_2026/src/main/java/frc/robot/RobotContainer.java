// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.shooterComm;
import frc.robot.subsystems.shooterSubsys;

import static edu.wpi.first.units.Units.RPM;

import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
  private final shooterSubsys shooter = new shooterSubsys();
  
  // Controllers
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DriverContrlPort);
  
  private final CommandXboxController operatorController = 
      new CommandXboxController(OperatorConstants.OperatorContrlPort);

  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), 
      () -> driverXbox.getLeftY() * -1, 
      () -> driverXbox.getLeftX() * -1)
  .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(1.8)
  .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    // Set shooter default to idle
    shooter.setDefaultCommand(Commands.runOnce(() -> {}, shooter));
  }

  /**
   * Use this method to define your trigger->command mappings.
   */
  private void configureBindings() {
    // Drive default command
    Command drivefieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(drivefieldOrientedAngularVelocity);
    
    // Driver reset odometry
    driverXbox.a().onTrue(Commands.runOnce(() -> {
        Pose2d currentPose = drivebase.getSwerveDrive().getPose();
        Pose2d newPose = new Pose2d(currentPose.getTranslation(), Rotation2d.fromDegrees(0));
        drivebase.getSwerveDrive().resetOdometry(newPose);
      }, drivebase)
    );

    driverXbox.x().onTrue(new DriveDistance(drivebase, 1, 0, 1));

    // Operator shooter controls - fixed syntax
    // operatorController.a().whileTrue(
    //     Commands.parallel(
    //         Commands.run(() -> shooter.setIntake(-0.75), shooter),  // Intake reverse
    //         shooter.setHopperVelocity(-1500)                         // Hopper reverse
    //     ).finallyDo(() -> shooter.setIntake(0))
    // );

    // operatorController.b().whileTrue(
    //     Commands.parallel(
    //         Commands.run(() -> shooter.setIntake(0.75), shooter),    // Intake forward
    //         shooter.setHopperVelocity(1500),                               // Hopper forward
    //         shooter.setShooterVelocity(2000)                               // Shooter forward (index speed)
    //     ).finallyDo(() -> shooter.setIntake(0))
    // );

    operatorController.rightTrigger().whileTrue(new shooterComm(shooter));  // Full shoot
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Commands.sequence(
        new DriveDistance(drivebase, 0.74, 0, 1.0),
        new shooterComm(shooter)  // Shoot after drive
    );
  }
  
  // Public accessors
  public shooterSubsys getShooter() { return shooter; }
  public SwerveSubsystem getDrivebase() { return drivebase; }
}
