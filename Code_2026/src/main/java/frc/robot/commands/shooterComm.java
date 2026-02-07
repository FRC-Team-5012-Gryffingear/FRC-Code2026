// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.shooterSubsys;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.RotationsPerSecond;

/** An example command that uses an example subsystem. */
public class shooterComm extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final shooterSubsys m_subsystem;
  private final double SHOOTER_RPM = 3000.0;
  private final double HOPPER_RPM = 2000.0;
  private final double INTAKE_DUTY = 0.76;
  private final double VELOCITY_TOLERANCE_RPS = 10.0; // +/- tolerance for "at speed"
  private AngularVelocity m_targetShooterVel;
  /**
   * Creates a new shooterComm.
   *
   * @param subsystem The subsystem used by this command.
   */
  public shooterComm(shooterSubsys subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_targetShooterVel = AngularVelocity.ofBaseUnits(SHOOTER_RPM / 60.0, RotationsPerSecond);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setShooterVelocity(SHOOTER_RPM);
    m_subsystem.setHopperVelocity(HOPPER_RPM);
    m_subsystem.setIntake(INTAKE_DUTY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setIntake(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double errorRps = Math.abs(m_subsystem.getShooterVelocity().minus(m_targetShooterVel).in(RotationsPerSecond));
    return errorRps > VELOCITY_TOLERANCE_RPS;
  }
}
    