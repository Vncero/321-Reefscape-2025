/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Angle;

@Logged
public class ClimberInputs {
  public Current climbCurrent; // current that the climb will be running
  public Angle climbAngle; // climb angle 
}
