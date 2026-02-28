package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VibrateCommand extends Command {
    private final CommandSwerveDrivetrain swerveDrive;  // Replace with your swerve instance name
    private final double amplitudeMps = 1.5;  // ~0.36 km/h; ~1 cm per cycle
    private final double periodSeconds = 1;  // 5 Hz frequency
    private final Timer timer = new Timer();
    private boolean positiveDirection = true;

    public VibrateCommand(CommandSwerveDrivetrain swerveDrive) {
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
        Translation2d translation = new Translation2d(direction * amplitudeMps, 0);
        SwerveRequest.RobotCentric chaseReq = new SwerveRequest.RobotCentric()

        .withVelocityX(translation.getX())  // Forward
        .withVelocityY(translation.getY())  // Left positive in WPILib/Limelight
        .withRotationalRate(0);  // Tune signs based on testing
        
        swerveDrive.setControl(chaseReq);    
            }

    @Override
    public boolean isFinished() {
        return false;  // Runs continuously until interrupted/scheduled off
    }

    @Override
    public void end(boolean interrupted) {   
    SwerveRequest.RobotCentric chaseReq = new SwerveRequest.RobotCentric()

    .withVelocityX(0)  // Forward
    .withVelocityY(0)  // Left positive in WPILib/Limelight
    .withRotationalRate(0);  // Tune signs based on testing
    swerveDrive.setControl(chaseReq);

     }
}
