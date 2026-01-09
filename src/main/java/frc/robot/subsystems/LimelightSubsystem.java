// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final String name = "limelight"; // "" usa la cámara "limelight" por defecto

  // Llama esto una vez (por ejemplo en robotInit o constructor de RobotContainer)
  public void configureCameraPose() {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        Constants.Limelight.CAM_X_M,  // forward + (m)
        Constants.Limelight.CAM_Y_M,  // left + (m)
        Constants.Limelight.CAM_Z_M,  // up + (m)
        Constants.Limelight.CAM_ROLL_DEG,
        Constants.Limelight.CAM_PITCH_DEG,
        Constants.Limelight.CAM_YAW_DEG
    );
  }

  /** -------- Lecturas básicas -------- */
  public boolean hasTarget() { return LimelightHelpers.getTV(name); }
  public double getTx()      { return LimelightHelpers.getTX(name); }  // grados, derecha (+)
  public double getTy()      { return LimelightHelpers.getTY(name); }
  public double getTa()      { return LimelightHelpers.getTA(name); }

  /** -------- Pipelines -------- */
  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(name, index);
  }

  /** -------- LEDs -------- */
  public void ledsPipelineControl() { LimelightHelpers.setLEDMode_PipelineControl(name); }
  public void ledsForceOn()         { LimelightHelpers.setLEDMode_ForceOn(name); }
  public void ledsForceOff()        { LimelightHelpers.setLEDMode_ForceOff(name); }
  public void ledsBlink()           { LimelightHelpers.setLEDMode_ForceBlink(name); }

  /** -------- Recorte opcional para rendimiento -------- */
  public void setCrop(double xmin, double xmax, double ymin, double ymax) {
    LimelightHelpers.setCropWindow(name, xmin, xmax, ymin, ymax);
  }

  public LimelightSubsystem() {}

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
