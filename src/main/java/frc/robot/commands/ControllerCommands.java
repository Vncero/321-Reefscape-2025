/* (C) Robolancers 2025 */
package frc.robot.commands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ControllerCommands {
  public static Command rumbleController(
      GenericHID controller, Time time, RumbleType side, double rumbleValue) {
    return Commands.run(
            () -> {
              controller.setRumble(side, rumbleValue);
            })
        .withTimeout(time)
        .finallyDo(
            () -> {
              controller.setRumble(side, 0);
            });
  }
}
