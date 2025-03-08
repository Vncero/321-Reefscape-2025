/* (C) Robolancers 2025 */
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.coralendeffector.CoralEndEffectorConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.subsystems.elevatorarm.ElevatorArmConstants;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

public class CoralSuperstructure {

  private Elevator elevator;
  private ElevatorArm arm;
  private CoralEndEffector endEffector;

  private CoralScorerSetpoint targetState = CoralScorerSetpoint.FEED_CORAL;

  public CoralSuperstructure(Elevator elevator, ElevatorArm arm, CoralEndEffector endEffector) {
    this.elevator = elevator;
    this.arm = arm;
    this.endEffector = endEffector;
  }

  // moves the entire elevator+arm superstructure to a desired state; this should be the go-to way
  // of moving the superstructure, aside from the default subsystem commands
  public Command goToSetpoint(Supplier<CoralScorerSetpoint> setpoint) {
    return Commands.runOnce(() -> targetState = setpoint.get())
        .andThen(
            goToSetpoint(
                () -> setpoint.get().getElevatorHeight(), () -> setpoint.get().getArmAngle()));
  }

  public Command goToSetpoint(Supplier<Distance> height, Supplier<Angle> angle) {
    return elevator.goToHeight(() -> height.get()).alongWith(arm.goToAngle(() -> angle.get()));
  }

  public void goToSetpoint(CoralScorerSetpoint setpoint) {
    goToSetpoint(setpoint.getElevatorHeight(), setpoint.getArmAngle());
  }

  public void goToSetpoint(Distance height, Angle ang) {
    elevator.goToHeight(height);
    arm.goToAngle(ang);
  }

  public boolean atTargetState() {
    return elevator.atSetpoint() && arm.atSetpoint();
  }

  public boolean atTargetState(Distance height, Angle angle) {
    return elevator.atHeight(height) && arm.atAngle(angle);
  }

  public boolean atTargetState(CoralScorerSetpoint setpoint) {
    return atTargetState(setpoint.getElevatorHeight(), setpoint.getArmAngle());
  }

  public Command feedCoral() {
    return goToSetpoint(() -> CoralScorerSetpoint.FEED_CORAL).alongWith(endEffector.intakeCoral());
  }

  public Command outtakeCoral() {
    return endEffector.outtakeCoral();
  }

  public Command knockAlgae() {
    return endEffector.runVolts(() -> CoralEndEffectorConstants.kAlgaeKnockVoltage);
  }

  public CoralScorerSetpoint getTargetState() {
    return targetState;
  }

  public boolean hasCoral() {
    return endEffector.hasCoral();
  }

  public Command tune() {
    TunableConstant armAngle = new TunableConstant("/CoralSuperstructure/ArmAngle", 0);
    TunableConstant height = new TunableConstant("/CoralSuperstructure/ElevatorHeight", 0);

    return arm.goToAngle(() -> ElevatorArmConstants.kPreAlignAngle)
        .until(arm::atSetpoint)
        .andThen(elevator.goToHeight(() -> Meters.of(height.get())).until(elevator::atSetpoint))
        .andThen(arm.goToAngle(() -> Degrees.of(armAngle.get())));
  }

  public Elevator getElevator() {
    return elevator;
  }

  public enum CoralScorerSetpoint {
    // TODO: determine angles empirically
    NEUTRAL(
        ElevatorConstants.kElevatorStartingHeight.plus(Meters.of(0.1)),
        Degrees.of(-40)), // TODO: make
    FEED_CORAL(Meters.of(0.965), Degrees.of(-87)),
    L1(Inches.of(45), Degrees.of(30)), // TODO: actually tune
    L2(Meters.of(0.95).minus(Inches.of(0.5)), Degrees.of(95)),
    L3(Meters.of(1.3).plus(Inches.of(1.25)), Degrees.of(95)),
    L4(Meters.of(2.06).plus(Inches.of(1)), Degrees.of(85)),
    ALGAE_LOW(Meters.of(1), Degrees.of(40)), // TODO: actually tune
    ALGAE_HIGH(Meters.of(1.4), Degrees.of(40)), // TODO: actually tune
    CLIMB(Meters.of(1.4), Degrees.of(0));

    private Distance elevatorHeight; // the height of the elevator to got
    private Angle armAngle; // the angle the arm should go to

    CoralScorerSetpoint(Distance elevatorHeight, Angle armAngle) {
      this.armAngle = armAngle;
      this.elevatorHeight = elevatorHeight;
    }

    public Distance getElevatorHeight() {
      return elevatorHeight;
    }

    public Angle getArmAngle() {
      return armAngle;
    }
  }
}
