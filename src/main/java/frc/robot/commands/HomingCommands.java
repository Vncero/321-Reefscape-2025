/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class HomingCommands {
  public static Command homeEverything(Elevator elevator, AlgaeIntakePivot pivot) {
    return elevator
        .homeEncoder()
        .onlyIf(() -> !elevator.elevatorIsHomed())
        .andThen(
            pivot
                .homeMechanism()
                .onlyIf(() -> !pivot.pivotIsHomed())
                .deadlineFor(
                    elevator.goToHeight(
                        () -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))));
  }
}
