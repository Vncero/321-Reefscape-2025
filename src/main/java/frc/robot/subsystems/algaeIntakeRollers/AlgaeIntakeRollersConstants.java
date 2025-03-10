/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakeRollersConstants {

  // motor, sensor IDs
  public static final int kBeamBreakId = 0;
  public static final int kMotorId = 17;

  // function constants
  public static final Voltage kRollerIntakeVoltage = Volts.of(8); // test
  public static final Voltage kRollerOuttakeVoltage = Volts.of(-8); // test
  public static final Voltage kStallVoltage = Volts.of(1);

  // motor configurations
  public static final boolean kRollerInverted = true;
  public static final int kSmartCurrentLimit = 40;

  // physical constants
  public static final double kRollerGearing = 40.0 / 16.0; // ask mech
  public static final double kRollerMOI = 0.01; // ask mech

  public static final double kRollerPositionConversionFactor = 1 / kRollerGearing; // ask mech
  public static final double kRollerVelocityConversionFactor =
      kRollerPositionConversionFactor / 60; // ask mech
  public static final Voltage kNominalVoltage = Volts.of(12);
}
