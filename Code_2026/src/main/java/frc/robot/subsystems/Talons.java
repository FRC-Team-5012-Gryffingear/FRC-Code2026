// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.DeviceIdentifier;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Talons extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX talon1 = new TalonFX(4);
  TalonFX talon2 = new TalonFX(5);
  TalonFX talon3 = new TalonFX(10);
  SparkMax motorController = new SparkMax(14, MotorType.kBrushless);
  public Talons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
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

  public void runMotor(){
    motorController.set(0.1);
  }

  public Command Motor(){
    return run(() ->{
        runMotor();
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
