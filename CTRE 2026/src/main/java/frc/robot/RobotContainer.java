// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix.platform.can.AutocacheState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ChaseAprilTagCommand;
import frc.robot.commands.TagFinderCommand;
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

            private final SwerveRequest.PointWheelsAt pointX = new SwerveRequest.PointWheelsAt();

    private final SwerveRequest.RobotCentric strafeRight = new SwerveRequest.RobotCentric()
    .withDeadband(0)
    .withRotationalDeadband(0)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final TagFinderCommand tagFinder = new TagFinderCommand(drivetrain, Lime);

    /* Path follower */
    private final SendableChooser<Command> startToShoot;
    private final SendableChooser<Command> shootToEnd;

    public RobotContainer() {
        startToShoot = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Starting to Shooting", startToShoot);
        shootToEnd = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Shooting to Ending", shootToEnd);
        shooter.setDefaultCommand(shooter.getDefaultCommand());
        intake.setDefaultCommand(intake.turnOffIntakeHopperSystemCommand());
        climb.setDefaultCommand(climb.climbStop());
        operatorController.rightBumper()
        .onTrue(shooter.getShooterToggleCommand());
        operatorController.leftTrigger().whileTrue(intake.outtakeFuel(22.5, 16.67)); //intake
        operatorController.rightTrigger().whileTrue(intake.shootFuel(12, 16.67));//shoot orginal hopper 16.67
        operatorController.x().whileTrue(intake.intakeFuel(25, 16.67)); //outtake
        joystick.povUp().whileTrue(climb.climbUp());
        joystick.povDown().whileTrue(climb.climbDown());
        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();

        startToShoot.addOption("Middle to Left", new PathPlannerAuto("StartM to ShootL"));
        startToShoot.addOption("Middle to Middle", new PathPlannerAuto("StartM to ShootM"));
        startToShoot.addOption("Middle to Right", new PathPlannerAuto("StartM to ShootR"));

        startToShoot.addOption("Left to Left", new PathPlannerAuto("StartL to ShootL"));
        startToShoot.addOption("Left to Middle", new PathPlannerAuto("StartL to ShootM"));
        startToShoot.addOption("Left to Right", new PathPlannerAuto("StartL to ShootR"));

        startToShoot.addOption("Right to Left", new PathPlannerAuto("StartR to ShootL"));
        startToShoot.addOption("Right to Middle", new PathPlannerAuto("StartR to ShootM"));
        startToShoot.addOption("Right to Right", new PathPlannerAuto("StartR to ShootR"));

        startToShoot.setDefaultOption("DefaultLeft", new PathPlannerAuto("StartL to ShootL"));
        


        shootToEnd.addOption("Left to Left", new PathPlannerAuto("ShootL to EndL"));
        shootToEnd.addOption("Left to Right", new PathPlannerAuto("ShootL to EndR"));

        shootToEnd.addOption("Middle to Left", new PathPlannerAuto("ShootM to EndL"));
        shootToEnd.addOption("Middle to Right", new PathPlannerAuto("ShootM to EndR"));

        shootToEnd.addOption("Right to Left", new PathPlannerAuto("ShootR to EndL"));
        shootToEnd.addOption("Right to Right", new PathPlannerAuto("ShootR to EndR"));
        shootToEnd.setDefaultOption("DefaultLeftToClimb", new PathPlannerAuto("ShootL to Climb"));
        // autoChooser.addOption("CenterMoveLeft", getShootLeftAuto());
        // autoChooser.addOption("LeftMoveCenter", getShootRightAuto());
        // autoChooser.addOption("RightMoveCenter", RightToMiddle());
        // autoChooser.addOption("CenterMoveRight", MiddleToRight());
        // autoChooser.addOption("glide", angledShot());
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

   
        // joystick.y().whileTrue(new VibrateCommand(drivetrain));
        //  * -0.075)))joystick.y().whileTrue(drivetrain.applyRequest(()-> drive
        // .withVelocityX(LimelightHelpers.getTY("limelight-calvin") * -0.1)
        // .withVelocityY(-joystick.getLeftX()*MaxSpeed)
        // .withRotationalRate(LimelightHelpers.getTX("limelight-calvin");

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

        // joystick.y().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(0.1,-0.1))
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
        //  joystick.b().whileTrue(new ChaseAprilTagCommand(drivetrain, Lime, 20, 2.0, 0, 0)); //ID 10
        //  joystick.b().whileTrue(new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.24, -1.10, Units.degreesToRadians(41.5))); //ID 11
        //  joystick.b().whileTrue(new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.24, 1.10, Units.degreesToRadians(-41.5))); //ID 8
        joystick.b().whileTrue(new TagFinderCommand(drivetrain, Lime));

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
    // return new PathPlannerAuto("Test");
        return Commands.sequence(
        // 1. Point wheels (adjust hub angle)
        drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(0)))
            .withTimeout(0.1),
                    shooter.getShooterToggleCommand().withTimeout(0.2),  // spin up + shoot 1

        // 2. Backup
        startToShoot.getSelected().withTimeout(5.0),
        // 3. Align
        new TagFinderCommand(drivetrain, Lime).withTimeout(1.5),
        // 4a. FIRST shot: shooter spin + intake feed
        
      
        intake.shootFuel(20, 18).withTimeout(5),
        // 4b. Stop shooting (shooter off, intake off)
        
        // 4e. Stop shooting
        shooter.getShooterToggleCommand().withTimeout(0.01),
        intake.turnOffIntakeHopperSystemCommand().withTimeout(0.01),
        // 5. Final position
        shootToEnd.getSelected(),
        new TagFinderCommand(drivetrain, Lime).withTimeout(2.3),
        climb.climbDown().withTimeout(3.75)
    );

    }

}