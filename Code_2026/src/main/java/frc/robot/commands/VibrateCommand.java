package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class VibrateCommand extends Command {
    private final SwerveSubsystem swerveDrive;  // Replace with your swerve instance name
    private final double amplitudeMps = 0.12;  // ~0.36 km/h; ~1 cm per cycle
    private final double periodSeconds = 0.2;  // 5 Hz frequency
    private final Timer timer = new Timer();
    private boolean positiveDirection = true;

    public VibrateCommand(SwerveSubsystem swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.restart();
        positiveDirection = true;
    }

    @Override
    public void execute() {
        double t = timer.get();
        double direction = Math.sin(2 * Math.PI * t / periodSeconds) > 0 ? 1 : -1;
        Translation2d translation = new Translation2d(direction * amplitudeMps, direction * amplitudeMps);
        swerveDrive.getSwerveDrive().drive(translation, 0, false, false);  // Robot-relative, closed-loop
    }

    @Override
    public boolean isFinished() {
        return false;  // Runs continuously until interrupted/scheduled off
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.getSwerveDrive().drive(new Translation2d(), 0, false, false);  // Stop
    }
}
