package org.frc3512.robot.subsystems.flywheels;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FlywheelConstants {

  public static int leftMotorID = 18;
  public static int middleMotorID = 19;
  public static int rightMotorID = 20;

  public static final TalonFXConfiguration flywheelConfig =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(Amps.of(120.0))
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimit(Amps.of(70.0))
                  .withSupplyCurrentLimitEnable(true))
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.5)
                  .withKI(2.0)
                  .withKD(0.0)
                  .withKV(
                      12.0
                          / RPM.of(6000)
                              .in(RotationsPerSecond))); // Run 12 volts when we want max RPM
}
