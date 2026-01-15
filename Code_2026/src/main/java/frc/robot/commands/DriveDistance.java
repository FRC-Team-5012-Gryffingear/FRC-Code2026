package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Simple command to drive a specific distance (forward/backward or left/right)
 * Uses odometry (encoders) to track movement
 */
public class DriveDistance extends Command {
    
    private final SwerveSubsystem swerve;
    private double startX;
    private double startY;
    private final double targetDistance;  // Distance in meters
    private final double vx;              // Forward speed (m/s)
    private final double vy;              // Sideways speed (m/s)
    
    /**
     * Drive a specific distance
     * @param swerve Your swerve subsystem
     * @param distanceX Distance forward (meters). Positive = forward, negative = backward
     * @param distanceY Distance sideways (meters). Positive = right, negative = left
     * @param speed Speed in m/s (typically 0.5 to 2.0)
     */
    public DriveDistance(SwerveSubsystem swerve, double distanceX, double distanceY, double speed) {
        this.swerve = swerve;
        
        // Calculate total distance and normalize direction
        this.targetDistance = Math.sqrt(distanceX * distanceX + distanceY * distanceY);
        
        if (targetDistance == 0) {
            this.vx = 0;
            this.vy = 0;
        } else {
            // Normalize to the desired speed
            this.vx = (distanceX / targetDistance) * speed;
            this.vy = (distanceY / targetDistance) * speed;
        }
        
        addRequirements(swerve);
    }
    
    @Override
    public void initialize() {
        startX = swerve.getSwerveDrive().getPose().getX();
        startY = swerve.getSwerveDrive().getPose().getY();
        System.out.println("Starting position: (" + startX + ", " + startY + ")");
        System.out.println("Target distance: " + targetDistance + "m");
    }
    
    @Override
    public void execute() {
        double currentX = swerve.getSwerveDrive().getPose().getX();
        double currentY = swerve.getSwerveDrive().getPose().getY();
        
        // Calculate how far we've moved
        double movedX = currentX - startX;
        double movedY = currentY - startY;
        double distanceMoved = Math.sqrt(movedX * movedX + movedY * movedY);
        
        // Drive in the direction specified
        swerve.getSwerveDrive().drive(new ChassisSpeeds(vx, vy, 0));
        
        // Dashboard
        SmartDashboard.putNumber("Drive/Distance Moved", distanceMoved);
        SmartDashboard.putNumber("Drive/Target Distance", targetDistance);
    }
    
    @Override
    public boolean isFinished() {
        double currentX = swerve.getSwerveDrive().getPose().getX();
        double currentY = swerve.getSwerveDrive().getPose().getY();
        double movedX = currentX - startX;
        double movedY = currentY - startY;
        double distanceMoved = Math.sqrt(movedX * movedX + movedY * movedY);
        
        // Finished when we're within 5cm of target
        return Math.abs(distanceMoved - targetDistance) < 0.05;
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, 0));
        System.out.println("Drive complete!");
    }
}