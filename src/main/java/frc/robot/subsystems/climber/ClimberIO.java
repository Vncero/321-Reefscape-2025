/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ClimberIO {
  default void updateInputs(ClimberInputs inputs) {} // updates inputs

  default void setClimbCurrent(Current Current) {} // sets roller current

  default void setClimbVoltage(Voltage voltage) {} // sets climb voltage

  default void resetEncoder(Angle angle) {} // resets climb encoder

  default void setLockServoAngle(Angle angle) {} // sets angle for servo to go to
}
