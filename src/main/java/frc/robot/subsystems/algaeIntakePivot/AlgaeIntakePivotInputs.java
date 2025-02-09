/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

@Logged
public class AlgaeIntakePivotInputs {
  public Angle pivotAngle; // current pivot angle
  public AngularVelocity pivotVelocity; // current pivot velocity
  public Current pivotCurrent; // current of ONE of the pivot motors
}
