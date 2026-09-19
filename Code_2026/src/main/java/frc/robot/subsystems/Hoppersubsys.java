// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hoppersubsys extends SubsystemBase {
  /** Creates a new Hoppersubsys. */
  private final TalonSRX MotorL = new TalonSRX(1);
  private final TalonSRX MotorR = new TalonSRX(2);

  public Hoppersubsys() {}

  public void moverightmotor(double power){
    MotorR.set(ControlMode.PercentOutput, power);    
  }

  public void moveleftmotor(double power){
    MotorL.set(ControlMode.PercentOutput, power);    
  }

  public void moveHopper(double power){
    MotorR.set(ControlMode.PercentOutput, power);
    MotorL.set(ControlMode.PercentOutput, power);  
  }

  public Command moveRCommand(double Rpower){
    return run(()->{
        moverightmotor(Rpower);
    });
  } 
  public Command moveLCommmand(double Lpower){
    return run(()->{
        moveleftmotor(Lpower);
    });
  }

  public Command moveHCommand(double Hpower){
   return run(()->{
    moveHopper(Hpower);
   });
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
