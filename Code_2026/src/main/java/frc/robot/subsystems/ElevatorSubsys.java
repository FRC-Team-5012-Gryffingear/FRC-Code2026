// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsys extends SubsystemBase {
    // private intakeCombined intakeSubsys = new intakeCombined();
    private intakePneumatics intakePneu;

    private TalonSRX elevatorTalon = new TalonSRX(Constants.elev_Motor);
    private CANcoder elevEncoder = new CANcoder(Constants.elev_Encoder);
    private double offset = Constants.elevOffset;

    private TalonSRX elevClimb = new TalonSRX(Constants.elev_climb);


    private PIDController elevHold = new PIDController(1.55, 0, 0); // might not work because of gravity variable not taken into count


    public ElevatorSubsys(intakePneumatics intakePneu) {
        this.intakePneu = intakePneu; 
        
        elevatorTalon.configFactoryDefault();
        elevClimb.configFactoryDefault();
        
        elevatorTalon.setInverted(InvertType.InvertMotorOutput);
        elevatorTalon.setNeutralMode(NeutralMode.Brake);

        elevClimb.setNeutralMode(NeutralMode.Brake);
        //elevClimb.setInverted(InvertType.InvertMotorOutput);
        
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.MagnetOffset = offset;
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        elevEncoder.getConfigurator().apply(config); 
    }
    
    //Gather data and determine what value and unit we are using to determine our movement
    public void elevMovement(double goal){ // each call of this function will have a different goal value
        double currentPosition = elevEncoder.getPosition().getValueAsDouble();
        double percent = MathUtil.clamp(elevHold.calculate(currentPosition,goal),-.65,.75);
        elevatorTalon.set(ControlMode.PercentOutput, percent);
        SmartDashboard.putNumber("Power of elevator", percent);

    }

    public boolean isAtHeight(double goal, double error){
      return getEncoderPos() + error >= goal ;
    }

    // public void elevClimbMove(double power){
    //   if(intakeSubsys.getAbsolutePower() != 1){
    //     elevClimb.set(ControlMode.PercentOutput, power*0.75);
    //   }
    //   else{
    //     elevClimb.set(ControlMode.PercentOutput, 0);
    //   }
      
    // }
    
    public void elevClimbMove(double power){
      if(intakePneu.SolenoidState() != Value.kForward){
        elevClimb.set(ControlMode.PercentOutput, power*0.75);
      }
      else{
        elevClimb.set(ControlMode.PercentOutput, 0);
      }
      
    }



    public void elevUpAndDown(double power){
      // if(elevEncoder.getPosition().getValueAsDouble() > 10.2 && power > 0){
      //   power = 0;
      // }
      // else if(elevEncoder.getPosition().getValueAsDouble() < 0.3 && power < 0){
      //   power = 0;
      // }
      if(Math.abs(power) > 0.2){
        elevatorTalon.set(ControlMode.PercentOutput, power*.75);
      }
      else{
        if(!DriverStation.isAutonomous()){
          elevatorTalon.set(ControlMode.PercentOutput, 0.1);
        }
      }
      
    }
    
    public double getEncoderPos(){
      return elevEncoder.getPosition().getValueAsDouble();
    }


    public void resetEncoderPos(){
        elevEncoder.setPosition(0);
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder pos", elevEncoder.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
