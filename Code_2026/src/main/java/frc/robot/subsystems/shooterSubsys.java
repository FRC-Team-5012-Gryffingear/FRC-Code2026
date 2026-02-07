  // Copyright (c) FIRST and other WPILib contributors.
  // Open Source Software; you can modify and/or share it under the terms of
  // the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;


  public class shooterSubsys extends SubsystemBase {
    /** Creates a new ShooterSubsys. */
    private final TalonFX hopperTalon = new TalonFX(1);
    private final TalonFX intakeTalon = new TalonFX(2);
    private final TalonFX shooterTalon = new TalonFX(3);
    
    private final SmartMotorControllerConfig velocityConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withClosedLoopController(0.5, 0, 0.2) //KP, Ki, KD for velocity
    .withSimClosedLoopController(0.5, 0, 0.2)
    .withFeedforward(new SimpleMotorFeedforward(0.2, 0.1, 0.01))  // ks, kv, ka
    .withTelemetry("VelocityMotor", TelemetryVerbosity.HIGH)
    .withGearing(1.0) //no reduction or adjust
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(60)); 

    private final SmartMotorControllerConfig intakeConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.OPEN_LOOP)
    .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH)
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40));

    private final SmartMotorController intakeMotor = new TalonFXWrapper(intakeTalon, DCMotor.getFalcon500(1), intakeConfig);
    private final SmartMotorController hopperMotor = new TalonFXWrapper(hopperTalon, DCMotor.getFalcon500(1), velocityConfig);
    private final SmartMotorController shooterMotor = new TalonFXWrapper(shooterTalon, DCMotor.getFalcon500(1), velocityConfig);
    private final FlyWheel hopperFlyWheel = new FlyWheel(new FlyWheelConfig(hopperMotor)); // seperate for hopper
    private final FlyWheel shooterFlyWheel = new FlyWheel(new FlyWheelConfig(shooterMotor));
  
    public shooterSubsys() {
    }
      /** Get shooter velocity (rad/s). */
    public AngularVelocity getShooterVelocity(){
      return shooterFlyWheel.getSpeed();
    }
      /** Set intake duty cycle (-1 to 1). */
    public void setIntake(double dutyCycle)  {
      intakeMotor.setDutyCycle(dutyCycle);
    }
    /** Set hopper velocity (RPM). */
    public Command setHopperVelocity(double rpm) {
      return hopperFlyWheel.setSpeed(AngularVelocity.ofBaseUnits(rpm/60, RotationsPerSecond));
    }
    
    /** Set shooter velocity (RPM). */
    public Command setShooterVelocity(double rpm) {
      return shooterFlyWheel.setSpeed(AngularVelocity.ofBaseUnits(rpm/60, RotationsPerSecond));
    }
    

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
      // Inline construction of command goes here.
      // Subsystem::RunOnce implicitly requires `this` subsystem.
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
      // Query some boolean state, such as a digital sensor.
      return false;
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      intakeMotor.updateTelemetry();
      hopperMotor.updateTelemetry();
      shooterMotor.updateTelemetry();
      hopperFlyWheel.updateTelemetry();
      shooterFlyWheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
      intakeMotor.simIterate();
      hopperMotor.simIterate();
      shooterMotor.simIterate();
      hopperFlyWheel.simIterate();
      shooterFlyWheel.simIterate();
    }
  }
