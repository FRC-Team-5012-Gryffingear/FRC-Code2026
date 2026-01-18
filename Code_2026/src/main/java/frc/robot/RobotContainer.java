// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ChaseAprilTagCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.ElevatorCom;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ScoringCommand;
import frc.robot.commands.intakeState1;
import frc.robot.commands.intakeState2;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intakePneumatics;
import swervelib.SwerveInputStream;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final intakePneumatics intake = new intakePneumatics();
  private final ElevatorSubsys elev = new ElevatorSubsys(intake);
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
     elev.setDefaultCommand(new ElevatorCom(elev,operatorController, 0));
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

    driverXbox.b().whileTrue(new ChaseAprilTagCommand(drivebase, limelight, 18, 0.864, 0));

    driverXbox.x().onTrue(new DriveDistance(drivebase, 1, 0, 1));

    driverXbox.y().whileTrue(new RunCommand(
      ()-> drivebase.getSwerveDrive().drive(
        new ChassisSpeeds(LimelightHelpers.getTY("limelight-daniel")* -0.1,
        -driverXbox.getLeftX() * 2.0,
        LimelightHelpers.getTX("limelight-daniel") *-0.05)
      ),drivebase
    ));


    operatorController.leftBumper().onTrue(new intakeState1(intake));
    operatorController.rightBumper().onTrue(new intakeState2(intake));
    operatorController.y().whileTrue(new ElevatorCom(elev, operatorController, 15)); //1st level 0.75
    operatorController.a().whileTrue(new ElevatorCom(elev,operatorController, 15)); // 2st level 2.41
    operatorController.b().whileTrue(new ElevatorCom(elev,operatorController, 15)); // 3nd level 5.2 
    operatorController.x().whileTrue(new ElevatorCom(elev, operatorController, 15)); // 4rd level  9.65
    operatorController.leftStick().whileTrue(new ElevatorCom(elev, operatorController, 15)); // Human player station  1.2
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //  return Commands.runOnce(() -> {
    //     intake.toggle();
    //   });
    // return new intakeState2(intake);
    return new SequentialCommandGroup(
        new ChaseAprilTagCommand(drivebase, limelight, 18, 0.864, -0.141),  
        new DriveDistance(drivebase, 0.73, 0, 1.0), 
        new ScoringCommand(elev, intake, 9.43));
  }
}
