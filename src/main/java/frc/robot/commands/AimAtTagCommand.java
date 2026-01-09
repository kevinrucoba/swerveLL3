// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AimAtTagCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimelightSubsystem limelight;
  private final DriveSubsystem m_drive;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AimAtTagCommand(LimelightSubsystem limelight, DriveSubsystem m_drive) {
    this.limelight = limelight;
    this.m_drive = m_drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelight.setPipeline(Constants.Limelight.PIPELINE_APRILTAG);
    limelight.ledsPipelineControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!limelight.hasTarget()) {
      // sin target: detén giro o aplica una búsqueda lenta si quieres
      m_drive.drive(0, 0, 0, true);
      return;
    }
    double tx = limelight.getTx(); // grados, derecha es + (gira a la derecha)
    double turnCmd = tx * Constants.Aim.kP_TURN;
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
