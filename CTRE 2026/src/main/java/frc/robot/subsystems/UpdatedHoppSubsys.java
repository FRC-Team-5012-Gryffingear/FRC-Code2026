// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpdatedHoppSubsys extends SubsystemBase {
  /** Creates a new UpdatedHoppSubsys. */
  private final TalonFX leftHopM = new TalonFX(18); // change to proper ID when added
  private final TalonFX rightHopM = new TalonFX(16);//ditto ^

  // double hopperPower = 0.06;
  double rightPowerE = 0.08; //OG 0.08
  double leftPowerE = 0.03; //OG 0.06

  double rightPowerR = 0.1; //OG 0.1
  double leftPowerR = 0.03; //OG 0.08

  public UpdatedHoppSubsys() {
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightHopM.getConfigurator().apply(rightConfig);

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftHopM.getConfigurator().apply(leftConfig);
  }


  public double rotateGetLeftPosition(){
    return leftHopM.getPosition().getValueAsDouble();
  }
  public void resetLeftPosition(){
    leftHopM.setPosition(0);
  }
  public Command zeroLeftPosition(){
    return run(()-> 
    {
      resetLeftPosition();
    }
    );
  }

  public double rotateGetRightPosition(){
    return rightHopM.getPosition().getValueAsDouble();
  }
  public void resetRightPosition(){
    rightHopM.setPosition(0);
  }
  public Command zeroRightPosition(){
    return run(()-> 
    {
      resetRightPosition();
    }
    );
  }

    public Command zeroPositions(){
      return Commands.sequence(zeroLeftPosition(), zeroRightPosition());
    }

    public void hoppermove(double powerRight, double powerLeft){
      rightHopM.set(powerRight);
      leftHopM.set(powerLeft);
    }
    public void rightMove(double powerR){
      rightHopM.set(powerR);
    }
    public void leftMove(double powerL){
      leftHopM.set(powerL);
    }

    public Command rightExtend(){
        return run(()-> rightMove(rightPowerE))
            .finallyDo(interrupted -> rightMove(0));
    }
    public Command leftExtend(){
      return run(()->leftMove(leftPowerE))
            .finallyDo(interrupted -> leftMove(0));
    }
    public Command hopperExtend() {
        return run(() -> hoppermove(rightPowerE, leftPowerE))
            .finallyDo(interrupted -> hoppermove(0, 0));
    }
    
    
    public Command rightRetract(){
      return run(()-> rightMove(-rightPowerR))
            .finallyDo(interrupted -> rightMove(0));
    }
    public Command leftRetract(){
      return run(()-> leftMove(-leftPowerR))
            .finallyDo(interrupted -> leftMove(0));
    }
    public Command hopperRetract() {
        return run(() -> hoppermove(-rightPowerR, -leftPowerR))
            .finallyDo(interrupted -> hoppermove(0,0));
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
    SmartDashboard.putNumber("Left Pos", rotateGetLeftPosition());
    SmartDashboard.putNumber("Right Pose", rotateGetRightPosition());
    SmartDashboard.putNumber("left Motor Speed", leftHopM.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Right Motor Speed", rightHopM.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}