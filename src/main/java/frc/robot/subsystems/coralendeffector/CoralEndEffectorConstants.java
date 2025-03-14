/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

// list of constants for the coral end effector
public class CoralEndEffectorConstants {
  // Motor configuration
  public static final int kMotorPort = 16;
  public static final int kTimeOfFlightId = 21;
  public static final boolean kInvertedMotor = true;
  public static final int kCurrentLimit = 40;

  // Physical constants
  public static final double kMomentOfInertia = 0.01;
  public static final double kGearing = 1;

  // Setpoints
  public static final Voltage kIntakeVoltage = Volts.of(8);
  public static final Voltage kOuttakeVoltage = Volts.of(-2.5);
  public static final Voltage kStallVoltage = Volts.of(0.6);
  public static final Voltage kAlgaeKnockVoltage = Volts.of(-8);

  // Tuned constants
  public static final Distance kDetectionRange = Millimeters.of(100);

  // for detecting if the end effector is intaking
  public static final Voltage kVoltageTolerance = Volts.of(0.5);
}
