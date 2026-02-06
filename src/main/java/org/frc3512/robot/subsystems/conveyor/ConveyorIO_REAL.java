package org.frc3512.robot.subsystems.conveyor;

import com.ctre.phoenix6.hardware.TalonFX;

public class ConveyorIO_REAL implements ConveyorIO {

  private TalonFX hopper, feeder;

  public ConveyorIO_REAL() {
    hopper = new TalonFX(ConveyorConstants.hopperMotorID);
    feeder = new TalonFX(ConveyorConstants.feederMotorID);

    hopper.getConfigurator().apply(ConveyorConstants.hopper);
    feeder.getConfigurator().apply(ConveyorConstants.feeder);
  }

  @Override
  public void setHopper(double speed) {
    hopper.set(speed);
  }

  @Override
  public void setFeeder(double speed) {
    feeder.set(speed);
  }

  @Override
  public void updateInputs(ConveyorIOInputs inputs) {
    // Log velocity
    inputs.feederVelocity = feeder.getVelocity().getValueAsDouble() * 60.0;
    inputs.hopperVelocity = hopper.getVelocity().getValueAsDouble() * 60.0;

    // Log applied volts
    inputs.feederVolts = feeder.getStatorCurrent().getValueAsDouble();
    inputs.hopperVolts = hopper.getStatorCurrent().getValueAsDouble();
  }
}
