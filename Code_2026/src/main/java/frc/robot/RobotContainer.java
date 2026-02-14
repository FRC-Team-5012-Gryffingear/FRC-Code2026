// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ChaseAprilTagCommand;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.VibrateCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeHopsubsys;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.climberSubsys;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final IntakeHopsubsys intake = new IntakeHopsubsys();
  private final LimelightSubsystem lime = new LimelightSubsystem();
  private final SendableChooser<Command> autoChooser;
  private final climberSubsys climber = new climberSubsys();

  // private final SendableChooser<Command> autoChooser;
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
    shooter.setDefaultCommand(shooter.getDefaultCommand()); // does nothing
    // autoChooser = AutoBuilde r.buildAutoChooser();
    intake.setDefaultCommand(intake.turnOffIntakeHopperSystemCommand());
        //Create the NamedCommands that will be used in PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    //Have the autoChooser pull in all PathPlanner autos as options
    autoChooser = AutoBuilder.buildAutoChooser();

    // //Set the default auto (do nothing) 
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("routine", new PathPlannerAuto("Example Auto"));

    //Put the autoChooser on the SmartDashboard
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
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
     driverXbox.y().whileTrue(new RunCommand(
      ()-> drivebase.getSwerveDrive().drive(
        new ChassisSpeeds(LimelightHelpers.getTY("limelight-calvin")* 0.1,
        -driverXbox.getLeftX() * 2.0,
        LimelightHelpers.getTX("limelight-calvin") *-0.075)
      ),drivebase
    ));

    // driverXbox.x().onTrue(new DriveDistance(drivebase, 1, 0, 1));
    operatorController.rightBumper()
    .onTrue(shooter.getShooterToggleCommand());

    operatorController.a().whileTrue(intake.intakeFuel(25, 16.67));

    operatorController.x().whileTrue(intake.shootFuel(15,16.67));

    operatorController.b().whileTrue(intake.outtakeFuel(25, 16.67));
  
    driverXbox.b().whileTrue(new ChaseAprilTagCommand(drivebase, lime, 20, 1.8, 0));

    operatorController.rightTrigger().whileTrue(climber.CliUp());

    operatorController.leftTrigger().whileTrue(climber.CliDown());
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
    return new PathPlannerAuto("auto ian is diddy");
    
    
//     Commands.sequence(
//       shooter.getShooterToggleCommand(),
//       new ChaseAprilTagCommand(drivebase, lime, 20, 1.8, 0),
//       Commands.waitSeconds(.5 ),
//       Commands.parallel(
//         new VibrateCommand(drivebase),
//         intake.shootFuel(25, 16.67))
//       );
}
}