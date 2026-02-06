package org.frc3512.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

  @AutoLog
  public static class FlywheelIOInputs {
    public double leftVelocity = 0.0;
    public double middleVelocity = 0.0;
    public double rightVelocity = 0.0;
    public double leftAppliedVolts = 0.0;
    public double middleAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;

    public double rpmSetpoint = 0.0;
  }

  public default void updateInputs(FlywheelIOInputs inputs) {}

  public default void setOutput(double output) {}
}
