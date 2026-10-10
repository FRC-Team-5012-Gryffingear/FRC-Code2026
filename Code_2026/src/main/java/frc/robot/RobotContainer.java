    // Copyright (c) FIRST and other WPILib contributors.
    // Open Source Software; you can modify and/or share it under the terms of
    // the WPILib BSD license file in the root directory of this project.

    package frc.robot;

    import static edu.wpi.first.units.Units.*;

    import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
    import com.ctre.phoenix.platform.can.AutocacheState;
    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.commands.PathPlannerAuto;
    import com.ctre.phoenix6.swerve.SwerveRequest;

    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.util.sendable.Sendable;
    import edu.wpi.first.wpilibj.counter.UpDownCounter;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
    import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
    import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

    import frc.robot.generated.TunerConstants;
    import frc.robot.subsystems.CommandSwerveDrivetrain;
    import frc.robot.subsystems.IntakeHopsubsys;
    import frc.robot.subsystems.ShooterSubsys;
    import frc.robot.subsystems.UpdatedHoppSubsys;
    import frc.robot.subsystems.agitatorsubsys;

    public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.9).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        private final ShooterSubsys shooter = new ShooterSubsys();
        private final IntakeHopsubsys intake = new IntakeHopsubsys();
        private final UpdatedHoppSubsys hopper = new UpdatedHoppSubsys();
        private final agitatorsubsys agitator = new agitatorsubsys();
        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operater = new CommandXboxController(1);


        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private final SendableChooser<Command> startToShoot;
        private final SendableChooser<Command> shootToEnd;

        public RobotContainer() {
            startToShoot = AutoBuilder.buildAutoChooser("Tests");
            SmartDashboard.putData("Starting to Shooting", startToShoot);
            shootToEnd = AutoBuilder.buildAutoChooser("Tests");
            SmartDashboard.putData("Shooting to Ending", shootToEnd);
            shooter.setDefaultCommand(shooter.getDefaultCommand());
            intake.setDefaultCommand(intake.turnOffIntakeHopperSystemCommand());

            startToShoot.addOption("Middle to Left", new PathPlannerAuto("StartM to ShootL"));
            startToShoot.addOption("Middle to Middle", new PathPlannerAuto("StartM to ShootM"));
            startToShoot.addOption("Middle to Right", new PathPlannerAuto("StartM to ShootR"));

            startToShoot.addOption("Left to Left", new PathPlannerAuto("StartL to ShootL"));
            startToShoot.addOption("Left to Middle", new PathPlannerAuto("StartL to ShootM"));
            startToShoot.addOption("Left to Right", new PathPlannerAuto("StartL to ShootR"));

            startToShoot.addOption("Right to Left", new PathPlannerAuto("StartR to ShootL"));
            startToShoot.addOption("Right to Middle", new PathPlannerAuto("StartR to ShootM"));
            startToShoot.addOption("Right to Right", new PathPlannerAuto("StartR to ShootR"));


            shootToEnd.addOption("Left to Left", new PathPlannerAuto("ShootL to EndL"));
            shootToEnd.addOption("Left to Right", new PathPlannerAuto("ShootL to EndR"));

            shootToEnd.addOption("Middle to Left", new PathPlannerAuto("ShootM to EndL"));
            shootToEnd.addOption("Middle to Right", new PathPlannerAuto("ShootM to EndR"));

            shootToEnd.addOption("Right to Left", new PathPlannerAuto("ShootR to EndL"));
            shootToEnd.addOption("Right to Right", new PathPlannerAuto("ShootR to EndR"));
            shootToEnd.setDefaultOption("DefaultLeftToClimb", new PathPlannerAuto("ShootL to Climb"));
            startToShoot.setDefaultOption("DefaultLeft", new PathPlannerAuto("StartL to ShootL"));
            // autoChooser.addOption("CenterMoveLeft", getShootLeftAuto());
            // autoChooser.addOption("LeftMoveCenter", getShootRightAuto());
            // autoChooser.addOption("RightMoveCenter", RightToMiddle());
            // autoChooser.addOption("CenterMoveRight", MiddleToRight());
            // autoChooser.addOption("glide", angledShot());
            //Operator commands
            operater.leftTrigger().whileTrue(intake.intakeFuel(25, 16.67)); //intake
            operater.rightTrigger().whileTrue(intake.shootFuel(18, 10));//shoot orginal hopper 16.67
            operater.leftBumper().onTrue(shooter.getShooterToggleCommand(80)); // shoot

            operater.y().onTrue(shooter.getShooterToggleCommand(61));
            operater.b().onTrue(shooter.getShooterToggleCommand(55));
            operater.a().onTrue(shooter.getShooterToggleCommand(63));
            operater.x().whileTrue(intake.outtakeFuel(22.5, 16.67)); //outtake
            operater.povUp().onTrue(shooter.getShooterToggleCommand(64));
            

            configureBindings();
        }

        private void configureBindings() {
            // Note that X is defined as forward according to WPILib convention,
            // and Y is defined as to the left according to WPILib convention.
            drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->
                    drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driver.getRightX() * MaxAngularRate)// Drive counterclockwise with negative X (left)
                )
            );

            // Idle while the robot is disabled. This ensures the configured
            // neutral mode is applied to the drive motors while disabled.
            final var idle = new SwerveRequest.Idle();
            RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true)
            );

            driver.x().whileTrue(drivetrain.applyRequest(() -> brake));
            // joystick.b().whileTrue(drivetrain.applyRequest(() ->
            //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
            // ));

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

            // Reset the field-centric heading on left bumper press.
            driver.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

            //hopper
            driver.povUp().whileTrue(hopper.hopperExtend());
            driver.povDown().whileTrue(hopper.hopperRetract());
            
            driver.rightTrigger().whileTrue(hopper.leftExtend());
            driver.rightBumper().whileTrue(hopper.leftRetract());

            driver.leftTrigger().whileTrue(hopper.rightExtend());
            driver.leftBumper().whileTrue(hopper.rightRetract());

            drivetrain.registerTelemetry(logger::telemeterize);
        
        }

        public Command getAutonomousCommand() {
           // return new PathPlannerAuto("Test");
           return Commands.sequence(
           shooter.getShooterToggleCommand(58.75).withTimeout(0.2), // sping up + shoot 1
           startToShoot.getSelected().withTimeout(5.0),
           intake.shootFuel(20, 13).withTimeout(5),
            // 4a. FIRST shot: shooter spin + intake feed
            // 4b. Stop shooting (shooter off, intake off)

            // 4e. stop shooting
            shooter.getShooterToggleCommand(50).withTimeout(0.01),
            intake.turnOffIntakeHopperSystemCommand().withTimeout(0.01),
            // 5. Final position
            shootToEnd.getSelected()
            //new TagFinderCommand(drivetrain, Lime).withTimeout(2.3)
            // climb.godownto(-5.7)
           );
        }
    }
