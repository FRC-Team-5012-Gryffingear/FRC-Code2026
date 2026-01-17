package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.intakeCombined;
import frc.robot.subsystems.intakePneumatics;

public class intakeState2 extends InstantCommand {
    private final intakePneumatics hookSubsystem;

    public intakeState2(intakePneumatics hookSubsystem) {
        this.hookSubsystem = hookSubsystem;
        addRequirements(hookSubsystem);
    }

    @Override
    public void initialize() {
        hookSubsystem.reverseHook();
    }
}