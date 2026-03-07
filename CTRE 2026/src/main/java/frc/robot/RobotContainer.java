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
import edu.wpi.first.math.util.Units;
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

        autoChooser.addOption("CenterMoveLeft", getShootLeftAuto());
        autoChooser.addOption("LeftMoveCenter", getShootRightAuto());
        autoChooser.addOption("RightMoveCenter", RightToMiddle());
        autoChooser.addOption("CenterMoveRight", MiddleToRight());
        autoChooser.addOption("glide", angledShot());
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
        joystick.y().whileTrue(drivetrain.applyRequest(()-> drive
        .withVelocityX(LimelightHelpers.getTY("limelight-calvin") * -0.1)
        .withVelocityY(-joystick.getLeftX()*MaxSpeed)
        .withRotationalRate(LimelightHelpers.getTX("limelight-calvin") * -0.075)));

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
         joystick.b().whileTrue(new ChaseAprilTagCommand(drivetrain, Lime, 20, 2.0, 0, 0)); //ID 10
        //  joystick.b().whileTrue(new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.24, -1.10, Units.degreesToRadians(41.5))); //ID 11
        //  joystick.b().whileTrue(new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.24, 1.10, Units.degreesToRadians(-41.5))); //ID 11

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
    return autoChooser.getSelected();
}
    public Command getShootLeftAuto(){
        return Commands.sequence(
        // 1. Point wheels (adjust hub angle)
        drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(0)))
            .withTimeout(0.2),
        // 2. Backup
        new PathPlannerAuto("ShootPrepBackward").withTimeout(6.0),
        // 3. Align
        new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.8, 0, 0).withTimeout(4.0),
        // 4a. FIRST shot: shooter spin + intake feed
        
        shooter.getShooterToggleCommand().withTimeout(1.3),  // spin up + shoot 1
        Commands.waitSeconds(1.7),
        intake.shootFuel(20, 18).withTimeout(9.5),
        // 4b. Stop shooting (shooter off, intake off)
        
        // 4e. Stop shooting
        shooter.getShooterToggleCommand().withTimeout(0.5),
        intake.turnOffIntakeHopperSystemCommand().withTimeout(0.5),
        // 5. Final position
        new PathPlannerAuto("FinalPosition").withTimeout(6.0)
    );

    }

    public Command getShootRightAuto(){
        return Commands.sequence(
        // 1. Point wheels (adjust hub angle)
        drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(0)))
            .withTimeout(0.2),
        // 2. Backup
        new PathPlannerAuto("left to shoot").withTimeout(6.0),
        // 3. Align
        // new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.8, 0).withTimeout(4.0),
        // 4a. FIRST shot: shooter spin + intake feed
        
        shooter.getShooterToggleCommand().withTimeout(1.3),  // spin up + shoot 1
        Commands.waitSeconds(1.7),
        intake.shootFuel(20, 18).withTimeout(2.5),
        new VibrateCommand(drivetrain).withTimeout(1),
        intake.shootFuel(20, 18).withTimeout(2.5),
        // 4b. Stop shooting (shooter off, intake off)
        
        // 4e. Stop shooting
        shooter.getShooterToggleCommand().withTimeout(0.5),
        intake.turnOffIntakeHopperSystemCommand().withTimeout(0.5),
        // 5. Final position
        new PathPlannerAuto("shoot to left").withTimeout(6.0)
    );}
     public Command RightToMiddle(){
            return Commands.sequence(
        // 1. Point wheels (adjust hub angle)
        drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(0)))
            .withTimeout(0.2),
        // 2. Backup
        new PathPlannerAuto("right to shoot").withTimeout(6.0),
        // 3. Align
        new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.8, 0, 0).withTimeout(4.0),
        // 4a. FIRST shot: shooter spin + intake feed
        
        shooter.getShooterToggleCommand().withTimeout(1.3),  // spin up + shoot 1
        Commands.waitSeconds(2),
        intake.shootFuel(20, 18).withTimeout(2.5),
        new VibrateCommand(drivetrain).withTimeout(1),
        intake.shootFuel(20, 18).withTimeout(2.5),
        // 4b. Stop shooting (shooter off, intake off)
        
        // 4e. Stop shooting
        shooter.getShooterToggleCommand().withTimeout(0.5),
        intake.turnOffIntakeHopperSystemCommand().withTimeout(0.5),
        // 5. Final position
        new PathPlannerAuto("shoot to right").withTimeout(6.0)
    ); 
     }
     public Command MiddleToRight(){
            return Commands.sequence(
        // 1. Point wheels (adjust hub angle)
        drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(0)))
            .withTimeout(0.2),
        // 2. Backup
        new PathPlannerAuto("ShootPrepBackward").withTimeout(6.0),
        // 3. Align
        new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.8, 0,0).withTimeout(4.0),
        // 4a. FIRST shot: shooter spin + intake feed
        
        shooter.getShooterToggleCommand().withTimeout(1.3),  // spin up + shoot 1
        Commands.waitSeconds(1.7),
        intake.shootFuel(20, 18).withTimeout(5),
        // 4b. Stop shooting (shooter off, intake off)
        
        // 4e. Stop shooting
        shooter.getShooterToggleCommand().withTimeout(0.5),
        intake.turnOffIntakeHopperSystemCommand().withTimeout(0.5),
        // 5. Final position
        new PathPlannerAuto("shoot to right").withTimeout(6.0)
    );
     }
     public Command angledShot(){
            return Commands.sequence(
        // 1. Point wheels (adjust hub angle)
        drivetrain.applyRequest(() -> point.withModuleDirection(Rotation2d.fromDegrees(0)))
            .withTimeout(0.2),
        // 2. Backup
        new PathPlannerAuto("middle glide").withTimeout(6.0),
        // 3. Align
        new ChaseAprilTagCommand(drivetrain, Lime, 20, 1.8, 0,0).withTimeout(4.0),
        // 4a. FIRST shot: shooter spin + intake feed
        
        shooter.getShooterToggleCommand().withTimeout(1.3),  // spin up + shoot 1
        Commands.waitSeconds(1.7),
        intake.shootFuel(20, 18).withTimeout(5),
        // 4b. Stop shooting (shooter off, intake off)
        
        // 4e. Stop shooting
        shooter.getShooterToggleCommand().withTimeout(0.5),
        intake.turnOffIntakeHopperSystemCommand().withTimeout(0.5),
        // 5. Final position
        new PathPlannerAuto("glide to trench entrance").withTimeout(6.0)
    );
     }

}