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
import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  TalonFX shooterMotor = new TalonFX(15);
  private boolean shooterRunning = false;



  private final MotionMagicVelocityVoltage shooterMMReq = new MotionMagicVelocityVoltage(0);



  // Acceleration when spinning UP (fast)
  private static final double SHOOTER_ACCEL_UP = 800.0; // rps/s

  // Acceleration when spinning DOWN (slow)
  private static final double SHOOTER_ACCEL_DOWN = 100.0; // rps/s

  private double shooterRPS = 63.6;

  public ShooterSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs gains = config.Slot0;
    gains.kP = 0.11; // Tune: output per RPS error
    gains.kI = 0.01;
    gains.kD = 0.01;
    gains.kV = 0.12; // Key: output per RPS target
    gains.kS = 0.05; // Static friction

    var motionMagic = config.MotionMagic;
    motionMagic.MotionMagicCruiseVelocity = 0; // Not used for velocity
  motionMagic.MotionMagicAcceleration = 4000; // OK as fallback



    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;





    shooterMotor.getConfigurator().apply(config);

    shooterMMReq.Acceleration = SHOOTER_ACCEL_DOWN;
    SmartDashboard.putNumber("RPS Shooter", shooterRPS);
  }
  public double getDistanceFromTarget(){
    double distance = Math.sqrt(LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getX() *
    LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getX() +
    LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getZ() *
    LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getZ());
    SmartDashboard.putNumber("Distance from Apriltag", distance);
    return distance;
  }



  public void shooterOn(double RPS){
    shooterMMReq.Acceleration = SHOOTER_ACCEL_UP;
    shooterMotor.setControl(shooterMMReq.withVelocity(RPS));
  }





  public void turnOffShooterSystem(){

    shooterOff();

  }









  public void shooterOff(){
    shooterMMReq.Acceleration = SHOOTER_ACCEL_DOWN;
    shooterMotor.setControl(shooterMMReq.withVelocity(0));
  }



  public void calculateShooterVelocity(){
     
  }

  public Command turnOffShooterSystemCommand(){
    return Commands.runOnce(this::turnOffShooterSystem, this);
}







  public Command turnOffShooterCommand(){
    return run(()->{
      shooterOff();
    });
  }











public Command getShooterToggleCommand() {
    return Commands.runOnce(
        () -> {
            if (shooterRunning) {
                shooterOff();
            } else {
                shooterOn(-shooterRPS);
          }
        shooterRunning = !shooterRunning;
        },
      this
    );
}

public Command shooterRunCommand() {
    return run(
        () -> shooterOn(shooterRPS)
    ).finallyDo(
      () -> shooterOff()
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
    double rpmSetpoint = SmartDashboard.getNumber("RPS Shooter",63.6);
    shooterRPS = rpmSetpoint;
    SmartDashboard.putNumber("Distance from Apriltag",
    Math.sqrt(LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getX() *
    LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getX() +
    LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getZ() *
    LimelightHelpers.getCameraPose3d_TargetSpace("limelight-calvin").getZ())
    );
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}