/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class ClimberConstants {
  // climb motor IDs
  public static final int kMotorId = 20; // placeholder id
  public static final int kServoPort = 2;
  public static final int kLimitSwitchPort = 0;

  // climb physical constants
  public static final double kClimbGearing = 500;
  public static final double kClimbMOI = 0.1;
  public static final Angle kStartingAngle = Degrees.of(-90);
  public static final Distance kClimbLength = Inches.of(0.6);
  public static final Angle kClimbMinAngle = Degrees.of(-180);
  public static final Angle kClimbMaxAngle = Degrees.of(180);
  public static final Angle kClimbPrepAngle = Degrees.of(180);
  public static final Angle kDefaultAngle = Degrees.of(-90);

  // climb servo lock + unlock positions
  public static final Angle kServoLockPosition = Degrees.of(90);
  public static final Angle kServoUnlockPosition = Degrees.of(180);

  // climb motor config
  public static final boolean kClimbInverted = true;
  public static final int kSmartCurrentLimit = 40;
  public static final double kClimbPositionConversionFactor = 360 / kClimbGearing;
  public static final double kClimbVelocityConversionFactor = kClimbPositionConversionFactor / 60;
  public static final Voltage kClimbHomeVoltage = Volts.of(-4);

  public static final Voltage kNominalVoltage = Volts.of(12);
  public static final Angle kClimbThreshold = Degrees.of(75); // to be tuned

  public static final Current kClimbCurrentRampRate =
      Amps.of(20); // placeholder, will need to tune this
  public static final Current kClimbCurrent = Amps.of(40); // placeholder

  public static final Angle kControllerTolerance = Degrees.of(1);
}
