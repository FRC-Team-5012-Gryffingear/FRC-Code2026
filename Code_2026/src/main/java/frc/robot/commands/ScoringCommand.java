// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsys;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.intakePneumatics;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ScoringCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ElevatorSubsys elev;
  private final intakePneumatics intake;
  private final double elevatorHeight;
  private boolean elevFinished;
  private boolean intakeFinished;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ScoringCommand(ElevatorSubsys elev, intakePneumatics intake, double elevatorHeight) {
    this.elev = elev;
    this.intake = intake;
    this.elevatorHeight = elevatorHeight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elev, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elev.resetEncoderPos();
    elevFinished = false;
    intakeFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elev.elevMovement(elevatorHeight);
    if(elev.isAtHeight(elevatorHeight, 0.2)){
        elevFinished = true;
    } else{
        elevFinished = false;
    }
    if(elevFinished && !intakeFinished){
        intake.toggle();
        intakeFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
