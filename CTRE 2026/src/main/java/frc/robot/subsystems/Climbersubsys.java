// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climbersubsys extends SubsystemBase {
    TalonFX climberMotor = new TalonFX(16);
    double motorPower = 0.65;
    private boolean climbUp = false;


  /** Creates a new Climbersubsys. */
  public Climbersubsys() {
   resetPosition();
  }
  public void climbMove(double power){
    // climberMotor.set(ControlMode.PercentOutput, power);
    climberMotor.set(power);
  }

  public Command climbUp(){
    return run(()->
    {
        climbMove(motorPower);
    });
  }
  public Command climbDown(){
    return run(()->
    {
        climbMove(-motorPower);
    });
  }
  public Command climbStop(){
    return run(()->
    { 
        climbMove(0);
    });
  }

  public Command climb(DoubleSupplier rightTrigger, DoubleSupplier leftTrigger){
    return run(() -> {
      climbMove(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
    });
  }
  public double rotateGetPosition(){
    return climberMotor.getPosition().getValueAsDouble();
  }
  public void resetPosition(){
    climberMotor.setPosition(0);
  }
  public Command zeroPosition(){
    return run(()-> 
    {
      resetPosition();
    }
    );
  }
  public Command goDownto(double position){
    return run(()-> {
      double error = position - rotateGetPosition();
      double kP = 0.1;
      double power = kP * error;
      power = Math.max(-0.75, Math.min(0.75, power));
      climbMove(power);
    });
  }

  public Command startToClimb(double targetRotations){
    return Commands.startEnd(this::climbUp, this::climbStop, this)
    .until(() -> rotateGetPosition() >= targetRotations);
  }

  public Command climbToStart(double targetRotations){
    return Commands.startEnd(this::climbDown, this::climbStop, this)
    .until(() -> rotateGetPosition() <= targetRotations);
  }

  public Command getShooterToggleCommand(double shooterRPS) {
    return Commands.runOnce(
        () -> {
            if (climbUp) {
                climbToStart(0);
            } else {
                startToClimb(10);
          }
        climbUp = !climbUp;
        },
      this
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
    SmartDashboard.putNumber("climbPosition", rotateGetPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  } 
}