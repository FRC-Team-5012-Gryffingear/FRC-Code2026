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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX shooterMotor = new TalonFX(3);
  TalonFX intakeMotor = new TalonFX(2);
  TalonFX hopperMotor = new TalonFX(1);
  boolean toggle = false;
  public ShooterSubsystem()  {
    SmartDashboard.putNumber("RPM Setpoint", 0.0);
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs gains = config.Slot0;
    gains.kP = 0.11;  // Tune: output per RPS error
    gains.kI = 0.5;
    gains.kD = 0.01;
    gains.kV = 0.12;  // Key: output per RPS target
    gains.kS = 0.05;  // Static friction
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    intakeMotor.getConfigurator().apply(config);
    hopperMotor.getConfigurator().apply(config);
    shooterMotor.getConfigurator().apply(config);
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

  public void runMotors(double intakeRPM, double hopperRPM, double shooterRPM){
    intakeMotor.set(intakeRPM);
    hopperMotor.set(hopperRPM);
    shooterMotor.set(shooterRPM);
  }

  public void shootMotors(double shooterRPM){
        VelocityVoltage shooterSetPoint = new VelocityVoltage(shooterRPM/60).withSlot(0);
        shooterMotor.setControl(shooterSetPoint);
    }
  

  private double getCalculatedSpeed() {
    // TODO: Calculate based on AprilTag distance and angle
    return 2000.0; // Placeholder
}

  public void getHopperRPM(){
    SmartDashboard.putNumber("HopperRPM", hopperMotor.get() * 6380.0);
  }

  public Command intakeFuelCommand(double intakeRPM, double hopperRPM, double shooterRPM){
    return run(() ->{
      runMotors(-intakeRPM, hopperRPM, shooterRPM);
    });
  }

  public Command shootFuelCommand(double intakeRPM, double hopperRPM, double shooterRPM){
    return run(()->{
      runMotors(intakeRPM, -hopperRPM, shooterRPM);
    });
  }

  public Command outtakeFuelCommand(double intakeRPM, double hopperRPM, double shooterRPM){
    return run(() ->{
      runMotors(intakeRPM, -hopperRPM, shooterRPM);
    });
  }

  public Command toggleShooterMotor() {
    return run(() -> {
        shootMotors(-getCalculatedSpeed());
    })
    .withName("ShooterMotorToggle");
}


  public Command stopMotorCommand(){
    return run (()->{
      runMotors(0, 0,0);
      shootMotors(0);
    });
  }

  public Command activateShooter(){
    return run (()->{
      activateShootMotor(2000);
    });
  }

  public Command disableShooter(){
    return run (()->{
      activateShootMotor(0);
    });
  }

  public double calculateShooterPower(){
    return 0;
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

  public void activateShootMotor(double shooterRPM){
    VelocityVoltage shooterSetPoint = new VelocityVoltage(shooterRPM/60).withSlot(0);
    shooterMotor.setControl(shooterSetPoint);
  }

  @Override
  public void periodic() {
    getHopperRPM();
    double rpmSetpoint = SmartDashboard.getNumber("RPM Setpoint", 0.0);
    // Simple example: map RPM to motor output (you'll probably use closed-loop control)
    double output = rpmSetpoint / 6000.0; // if 6000 is your max RPM
    output = Math.max(-1.0, Math.min(1.0, output)); // clamp to [-1,1]
    shooterMotor.set(output);
    

    // if(toggle){
    //   activateShootMotor(-calculateShooterPower());
    // }else{
    //   activateShootMotor(0);
    // }
    
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
