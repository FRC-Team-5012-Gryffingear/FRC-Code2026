// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

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


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeHopsubsys extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
 
  TalonFX intakeMotor = new TalonFX(13);
  TalonFX hopperMotor = new TalonFX(14);  
 


  private final MotionMagicVelocityVoltage intakeMMReq = new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityVoltage hopperMMReq  = new MotionMagicVelocityVoltage(0);

  // Acceleration when spinning UP (fast)

  private static final double INTAKE_ACCEL_UP = 600.0;
  private static final double HOPPER_ACCEL_UP  = 1000.0;

  // Acceleration when spinning DOWN (slow)

  private static final double INTAKE_ACCEL_DOWN = 80.0;
  private static final double HOPPER_ACCEL_DOWN  = 120.0;  

  public IntakeHopsubsys()  {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs gains = config.Slot0;
    gains.kP = 0.11;  // Tune: output per RPS error
    gains.kI = 0.01;
    gains.kD = 0.01;
    gains.kV = 0.12;  // Key: output per RPS target
    gains.kS = 0.05;  // Static friction

    var motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 0;  // Not used for velocity
    motionMagic.MotionMagicAcceleration = 4000; // OK as fallback


    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;


    intakeMotor.getConfigurator().apply(config);
    hopperMotor.getConfigurator().apply(config);


  
    intakeMMReq.Acceleration = INTAKE_ACCEL_DOWN;
    hopperMMReq.Acceleration = HOPPER_ACCEL_DOWN;
  }

  public void intakeOn(double RPS){
    intakeMMReq.Acceleration = INTAKE_ACCEL_UP;
    intakeMotor.setControl(intakeMMReq.withVelocity(RPS));
  }

 
  

  public void hopperOn(double RPS){
    hopperMMReq.Acceleration = HOPPER_ACCEL_UP;
    hopperMotor.setControl(hopperMMReq.withVelocity(RPS));
  }


  public void turnOffIntakeHopper(){
    intakeOff();
    hopperOff();
  }

  public void intakeOff(){
    intakeMMReq.Acceleration = INTAKE_ACCEL_DOWN;
    if(Math.abs(intakeMotor.getVelocity().getValueAsDouble()) > 1){
      intakeMotor.setControl(intakeMMReq.withVelocity(0));
    } else{
      intakeMotor.set(0);
    }
  }

 

  public void hopperOff(){
    hopperMMReq.Acceleration = HOPPER_ACCEL_DOWN;
    if(Math.abs(hopperMotor.getVelocity().getValueAsDouble()) > 1  ){
        hopperMotor.setControl(hopperMMReq.withVelocity(0));
    } else{
        hopperMotor.set(0);

    }
  
  }

  
  





  public Command turnOffIntakeHopperSystemCommand(){
    return run(()->{
      turnOffIntakeHopper();
    });
  }



  public Command intakeFuel(double intakeRPS, double hopperRPS){
     return Commands.sequence(
            Commands.runOnce(() -> {
              hopperOn(hopperRPS);
            }, this),
            Commands.waitSeconds(1.0),
            Commands.run(() -> {
              hopperOn(hopperRPS);
              intakeOn(-intakeRPS);
            }, this)
        );
  }



  public Command shootFuel(double intakeRPS, double hopperRPS){
    return run(() ->{
      intakeOn(-intakeRPS);
      hopperOn(-hopperRPS);
    });
  }




  public Command outtakeFuel(double intakeRPS, double hopperRPS){
        return Commands.sequence(
            Commands.runOnce(() -> {
              intakeOn(intakeRPS);
            }, this),
            Commands.waitSeconds(1.0),
            Commands.run(() -> {
              hopperOn(-hopperRPS);
              intakeOn(intakeRPS);
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
    SmartDashboard.putNumber("intakeRPS", intakeMotor.getVelocity().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
