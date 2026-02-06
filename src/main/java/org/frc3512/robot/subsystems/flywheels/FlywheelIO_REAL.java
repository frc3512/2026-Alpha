package org.frc3512.robot.subsystems.flywheels;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class FlywheelIO_REAL implements FlywheelIO {

  private TalonFX leftMotor, middleMotor, rightMotor;

  private DutyCycleOut output = new DutyCycleOut(0.0);
  private double targetOutput = 0.0;

  public FlywheelIO_REAL() {

    leftMotor = new TalonFX(FlywheelConstants.leftMotorID);
    middleMotor = new TalonFX(FlywheelConstants.middleMotorID);
    rightMotor = new TalonFX(FlywheelConstants.rightMotorID);

    leftMotor.getConfigurator().apply(FlywheelConstants.flywheelConfig);
    middleMotor.getConfigurator().apply(FlywheelConstants.flywheelConfig);
    rightMotor.getConfigurator().apply(FlywheelConstants.flywheelConfig);
  }

  @Override
  public void setOutput(double wantedOutput) {
    targetOutput = wantedOutput;
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {

    // Log RPM - convert from RPS to RPM
    inputs.leftVelocity = leftMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.middleVelocity = middleMotor.getVelocity().getValueAsDouble() * 60.0;
    inputs.rightVelocity = rightMotor.getVelocity().getValueAsDouble() * 60.0;

    // Log applied volts
    inputs.leftAppliedVolts = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.middleAppliedVolts = middleMotor.getStatorCurrent().getValueAsDouble();
    inputs.rightAppliedVolts = rightMotor.getStatorCurrent().getValueAsDouble();

    // Update motor control
    leftMotor.setControl(output.withOutput(targetOutput));
    middleMotor.setControl(output.withOutput(targetOutput));
    rightMotor.setControl(output.withOutput(targetOutput));
  }
}
