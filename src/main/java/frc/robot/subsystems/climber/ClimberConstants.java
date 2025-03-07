/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class ClimberConstants {
  // motor IDs
  public static final int kMotorId = 0;

  public static final boolean kClimbInverted = true;
  public static final int kSmartCurrentLimit = 40;

  public static final Voltage kNominalVoltage = Volts.of(12);
}
