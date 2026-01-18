// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;

import java.lang.management.OperatingSystemMXBean;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** An example command that uses an example subsystem. */
public class ElevatorCom extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsys elev;
  // private final double goal;
  private final CommandXboxController controller2;
  private final double heightMoved;  

  /**
   * Creates a new ElevatorCom.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ElevatorCom(ElevatorSubsys subsystem, CommandXboxController controller, double height) {
    elev = subsystem;
    // this.goal = goal;
    controller2 = controller;
    heightMoved = height;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // elev.resetEncoderPos();
  
  }

// add .05 due to encoder error  
     //1st level 0.75 -- 1.25
    // 2st level 2.41 -- 3.14
    // 3nd level 5.2 ---5.75
    // 4rd level  9.65 -- 9.67
    // Human player station  1.2  --- 1.7

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elev.elevMovement(heightMoved);
    System.out.println("RUNNING RUNNING RUNNING ELEV CODE");
    
    if(controller2.a().getAsBoolean()){
      elev.elevMovement(0);
    }
    else if(controller2.b().getAsBoolean()){
      elev.elevMovement(2.34);
    }
    else if(controller2.x().getAsBoolean()){
      elev.elevMovement(5.36);
    }
    else if(controller2.y().getAsBoolean()){
      elev.elevMovement(9.65); 
    }
    else if(controller2.leftStick().getAsBoolean()){
      elev.elevMovement(1.23); 
    }
    else if(controller2.rightStick().getAsBoolean()){
      elev.resetEncoderPos();
    }
    else{
      if(!DriverStation.isAutonomous()){
        elev.elevUpAndDown(controller2.getRightTriggerAxis() - controller2.getLeftTriggerAxis());
      } 
    }
    elev.elevUpAndDown(controller2.getRightTriggerAxis() - controller2.getLeftTriggerAxis());
    

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("FINISHED THIS ELEVATOR CODE IS FINISHED FINISHED FINISHED FINISHED");
    return elev.getEncoderPos() >= 15;
  }
}
