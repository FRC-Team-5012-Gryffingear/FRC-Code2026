// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Talons extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX talon1 = new TalonFX(1);
  TalonFX talon2 = new TalonFX(2);
  TalonFX talon3 = new TalonFX(3);
  SparkMax PWM = new SparkMax(14, MotorType.kBrushed);
  public Talons()  {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs gains = config.Slot0;
    gains.kP = 0.11;  // Tune: output per RPS error
    gains.kI = 0.5;
    gains.kD = 0.01;
    gains.kV = 0.12;  // Key: output per RPS target
    gains.kS = 0.05;  // Static friction

    talon1.getConfigurator().apply(config);
    talon2.getConfigurator().apply(config);
    talon3.getConfigurator().apply(config);
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

  public void runMotor(double targetRPM){
    double rps = targetRPM / 60.0;
    VelocityVoltage setpoint = new VelocityVoltage(rps).withSlot(0);

    // talon1.setControl(setpoint);
    // talon2.setControl(setpoint);
    // talon3.setControl(setpoint);
    talon1.set(0.5 * targetRPM);
    talon2.set(-0.3 * targetRPM);
    talon3.set(-targetRPM);
  }

  public Command Motor(double rpm){
    return run(() ->{
        runMotor(rpm);
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
