// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveDistance;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.DriverContrlPort);
  
  private final CommandXboxController operatorController = 
      new CommandXboxController(OperatorConstants.OperatorContrlPort);

  
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(), () -> driverXbox.getLeftY() * -1, () -> driverXbox.getLeftX() * -1)
  .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
  .deadband(OperatorConstants.DEADBAND)
  .scaleTranslation(1.8)
  .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    shooter.setDefaultCommand(shooter.turnOffIntakeHopperSystemCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command drivefieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(drivefieldOrientedAngularVelocity);
    driverXbox.a().onTrue(
      Commands.runOnce(() -> {
        Pose2d currentPose = drivebase.getSwerveDrive().getPose();
        Pose2d newPose = new Pose2d(
          currentPose.getTranslation(), Rotation2d.fromDegrees(0)
        );
        drivebase.getSwerveDrive().resetOdometry(newPose);
      })
    );

    // driverXbox.x().onTrue(new DriveDistance(drivebase, 1, 0, 1));
    operatorController.a().whileTrue(shooter.intakeFuel(2000/60, 1800/60));

    operatorController.rightBumper().onTrue(shooter.loadShooter());
    operatorController.leftBumper().onTrue(shooter.turnOffShooterCommand());

    driverXbox.x().whileTrue(shooter.shootFuel(2000/60, 2000/60));

    operatorController.b().whileTrue(shooter.outtakeFuel(2000/60, 2000/60));
  
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
      * @return 
      *
      * @return the command to run in autonomous
      */
     public void Command () {
    // An example command will be run in autonomous
    //  return Commands.runOnce(() -> {
    //     intake.toggle();
    //   });
    // return new intakeState2(intake);
  }
    public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //  return Commands.runOnce(() -> {
    //     intake.toggle();
    //   });
    // return new intakeState2(intake);
    return new DriveDistance(drivebase, 0, 0, 0);
  }
}
