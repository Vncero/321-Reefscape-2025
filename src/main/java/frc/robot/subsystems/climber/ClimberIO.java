/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;

@Logged
public interface ClimberIO {
  default void updateInputs(ClimberInputs inputs) {} // updates inputs

  default void setClimbCurrent(Current Current) {} // sets roller current
}
