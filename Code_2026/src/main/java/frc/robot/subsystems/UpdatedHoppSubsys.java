// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UpdatedHoppSubsys extends SubsystemBase {
  /** Creates a new UpdatedHoppSubsys. */
  private final TalonFX leftHopM = new TalonFX(16); // change to proper ID when added
  private final TalonFX rightHopM = new TalonFX(18);//ditto ^

  double hopperPower = 0.1;

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

  public void hoppermove(double powerm){
    rightHopM.set(powerm);
    leftHopM.set(powerm);
  }

  public void hopperMoveMaxMin(double power) {
        double leftPosition = rotateGetLeftPosition();
        double rightPosition = rotateGetRightPosition();

        // Stop extension if either side reaches the upper limit.
        if (power > 0 &&
            (leftPosition >= 5.0 || rightPosition >= 5.0)) {
            power = 0;
        }

        // Stop retraction if either side reaches the lower limit.
        if (power < 0 &&
            (leftPosition <= 0.0 || rightPosition <= 0.0)) {
            power = 0;
        }

        hoppermove(power);
    }

    public Command hopperExtend() {
        return run(() -> hopperMoveMaxMin(hopperPower))
            .finallyDo(interrupted -> hoppermove(0));
    }

    public Command hopperRetract() {
        return run(() -> hopperMoveMaxMin(-hopperPower))
            .finallyDo(interrupted -> hoppermove(0));
    }

    public Command extendHopper(
        DoubleSupplier rightTrigger,
        DoubleSupplier leftTrigger
    ) {
        return run(() -> {
            double power =
                (rightTrigger.getAsDouble() -
                leftTrigger.getAsDouble()) * hopperPower;

            hopperMoveMaxMin(power);
        }).finallyDo(interrupted -> hoppermove(0));
    }
//   public Command hopperExtend(){
//     return run(()->{
//         hoppermove(hopperPower);
//     });
//   }

//   public Command hopperRetract(){
//     return run(()->{
//         hoppermove(-hopperPower);
//     });
//   }

//   public void hopperMoveMaxMin(double power){
//     if ((rotateGetLeftPosition() > 5 || rotateGetRightPosition() > 5) && power > 0){
//         power = 0;
//     }
//     hoppermove(power);
//   }

  
//   public Command extendHopper(DoubleSupplier rightTrigger, DoubleSupplier leftTrigger){
//     return run(()->{
//         hopperMoveMaxMin(rightTrigger.getAsDouble() - leftTrigger.getAsDouble());
//     });
//   } 

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
