package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.intakeCombined;
import frc.robot.subsystems.intakePneumatics;

public class intakeState1 extends InstantCommand {
    private final intakePneumatics hookSubsystem;

    public intakeState1(intakePneumatics hookSubsystem) {
        this.hookSubsystem = hookSubsystem;
        addRequirements(hookSubsystem);
    }

    @Override
    public void initialize() {
       
    }
    @Override
    public void execute(){
        hookSubsystem.toggle();
        
     
    }

}