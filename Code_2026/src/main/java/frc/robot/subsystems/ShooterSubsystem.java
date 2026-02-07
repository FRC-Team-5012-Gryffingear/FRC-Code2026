// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX shooterMotor = new TalonFX(3);
  TalonFX intakeMotor = new TalonFX(2);
  TalonFX hopperMotor = new TalonFX(1);  

  private final MotionMagicVelocityVoltage shooterMMReq   = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityVoltage intakeMMReq = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityVoltage hopperMMReq  = new MotionMagicVelocityVoltage(0);

  // Acceleration when spinning UP (fast)
  private static final double SHOOTER_ACCEL_UP   = 800.0; // rps/s
  private static final double INTAKE_ACCEL_UP = 600.0;
  private static final double HOPPER_ACCEL_UP  = 1000.0;

  // Acceleration when spinning DOWN (slow)
  private static final double SHOOTER_ACCEL_DOWN   = 100.0; // rps/s
  private static final double INTAKE_ACCEL_DOWN = 80.0;
  private static final double HOPPER_ACCEL_DOWN  = 120.0;  

  public ShooterSubsystem()  {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs gains = config.Slot0;
    gains.kP = 0.11;  // Tune: output per RPS error
    gains.kI = 0.5;
    gains.kD = 0.01;
    gains.kV = 0.12;  // Key: output per RPS target
    gains.kS = 0.05;  // Static friction

    var motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 400; 
    motionMagic.MotionMagicAcceleration = 4000;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;


    intakeMotor.getConfigurator().apply(config);
    hopperMotor.getConfigurator().apply(config);
    shooterMotor.getConfigurator().apply(config);

    shooterMMReq.Acceleration = SHOOTER_ACCEL_DOWN;
    intakeMMReq.Acceleration = INTAKE_ACCEL_DOWN;
    hopperMMReq.Acceleration = HOPPER_ACCEL_DOWN;
  }

  public void intakeOn(double RPS){
    intakeMMReq.Acceleration = INTAKE_ACCEL_UP;
    intakeMotor.setControl(intakeMMReq.withVelocity(RPS));
  }

  public void shooterOn(double RPS){
    shooterMMReq.Acceleration = SHOOTER_ACCEL_UP;
    shooterMotor.setControl(shooterMMReq.withVelocity(RPS));
  }

  public void hopperOn(double RPS){
    hopperMMReq.Acceleration = HOPPER_ACCEL_UP;
    hopperMotor.setControl(hopperMMReq.withVelocity(RPS));
  }

  public void turnOffShooterSystem(){
    intakeOff();
    shooterOff();
    hopperOff();
  }

  public void turnOffIntakeHopper(){
    intakeOff();
    hopperOff();
  }

  public void intakeOff(){
    intakeMMReq.Acceleration = INTAKE_ACCEL_DOWN;
    intakeMotor.setControl(intakeMMReq.withVelocity(0));
  }

  public void shooterOff(){
    shooterMMReq.Acceleration = SHOOTER_ACCEL_DOWN;
    shooterMotor.setControl(shooterMMReq.withVelocity(0));
  }

  public void hopperOff(){
    hopperMMReq.Acceleration = HOPPER_ACCEL_DOWN;
    hopperMotor.setControl(hopperMMReq.withVelocity(0));
  }

  public double calculateShooterVelocity(){
    return 2000;
  }

  public Command turnOffShooterSystemCommand(){
    return run(()->{
      turnOffShooterSystem();
    });
  }

  public Command turnOffIntakeHopperSystemCommand(){
    return run(()->{
      turnOffIntakeHopper();
    });
  }

  public Command turnOffShooterCommand(){
    return run(()->{
      shooterOff();
    });
  }

  public Command intakeFuel(double intakeRPS, double hopperRPS){
    return run(() ->{
      intakeOn(-intakeRPS);
      hopperOn(hopperRPS);
    });
  }

  public Command shootFuel(double intakeRPS, double hopperRPS){
    return run(() ->{
      intakeOn(intakeRPS);
      hopperOn(-hopperRPS);
    });
  }

  public Command loadShooter(){
    return run(()-> {
      shooterOn(-calculateShooterVelocity());
    });
  }

  public Command outtakeFuel(double intakeRPS, double hopperRPS){
        return Commands.sequence(
            Commands.runOnce(() -> {
              hopperOn(-hopperRPS);
            }, this),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> {
              intakeOn(-intakeRPS);
            }, this)
        );
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
