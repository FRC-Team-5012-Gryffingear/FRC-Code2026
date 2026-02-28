// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ChaseAprilTagCommand;
import frc.robot.commands.VibrateCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climbersubsys;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeHopsubsys;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final IntakeHopsubsys intake = new IntakeHopsubsys();
    private final LimelightSubsystem Lime = new LimelightSubsystem();
    private final Climbersubsys climb = new Climbersubsys();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric strafeLeft = new SwerveRequest.RobotCentric()
    .withDeadband(0)
    .withRotationalDeadband(0)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.RobotCentric strafeRight = new SwerveRequest.RobotCentric()
    .withDeadband(0)
    .withRotationalDeadband(0)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        shooter.setDefaultCommand(shooter.getDefaultCommand());
        intake.setDefaultCommand(intake.turnOffIntakeHopperSystemCommand());
        climb.setDefaultCommand(climb.climbStop());
        operatorController.rightBumper()
        .onTrue(shooter.getShooterToggleCommand());
        operatorController.leftTrigger().whileTrue(intake.outtakeFuel(22.5, 16.67)); //intake
        operatorController.rightTrigger().whileTrue(intake.shootFuel(15, 16.67));
        operatorController.x().whileTrue(intake.intakeFuel(25, 16.67)); //outtake
        joystick.povUp().whileTrue(climb.climbUp());
        joystick.povDown().whileTrue(climb.climbDown());
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.y().whileTrue(new VibrateCommand(drivetrain));

        joystick.rightTrigger().and(joystick.povLeft()).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(1).withVelocityY(0))
        );
        joystick.leftTrigger().and(joystick.povLeft()).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-1).withVelocityY(0))
        );
       joystick.leftBumper().and(joystick.povLeft()).whileTrue(drivetrain.applyRequest(()->
            strafeLeft.withVelocityY(1).withVelocityX(0).withRotationalRate(0)
        ));
        joystick.rightBumper().and(joystick.povLeft()).whileTrue(drivetrain.applyRequest(()->
            strafeRight.withVelocityY(-1).withVelocityX(0).withRotationalRate(0)
        ));

        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
            // no idea what it does
        joystick.rightTrigger().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        joystick.leftBumper().whileTrue(drivetrain.applyRequest(()->
            strafeLeft.withVelocityY(0.5).withVelocityX(0).withRotationalRate(0)
        ));
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(()->
            strafeRight.withVelocityY(-0.5).withVelocityX(0).withRotationalRate(0)
        ));
         joystick.b().whileTrue(new ChaseAprilTagCommand(drivetrain, Lime, 20, 2.0, 0));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
    return new PathPlannerAuto("Test");
    // return Commands.sequence(
    //     // 1. Turn on wheels to prepare shooting (point wheels toward hub)
    //     shooter.getShooterToggleCommand()
    //     .withTimeout(1.0),  // adjust duration
    //     // 2. Drive backward 1-2m using PathPlannerAuto (ask Micah/Omar for path name)
    //     new PathPlannerAuto("ShootPrepBackward"),  // replace with actual path name
    //     // 3. Correct position using Limelight/ChaseAprilTag at shooting distance
    //     new ChaseAprilTagCommand(drivetrain, Lime, 20, 2.0, 0),  // adjust params as needed
    //     // 4. Shoot + vibrate in parallel
    //     Commands.parallel(
    //         shooter.getShooterToggleCommand(),  // or shooter shooting command
    //         new VibrateCommand(drivetrain)
    //     ),
    //     // 5. Drive to final position (ask Omar, use PathPlanner path)
    //     new PathPlannerAuto("FinalPosition")  // replace with actual path name
    // );
}

}