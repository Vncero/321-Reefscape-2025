/* (C) Robolancers 2025 */
package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Comparator;
import java.util.TreeSet;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Leds extends SubsystemBase {
  // priority (greater # means more important), condition for pattern to apply, pattern to apply
  public record Signal(int priority, BooleanSupplier condition, Supplier<LEDPattern> pattern) {}

  // LED
  public final AddressableLED strip;
  private final AddressableLEDBuffer buffer;
  private static TreeSet<Signal> signals;
  private LEDPattern currentPattern = LedsConstants.kDefault;

  public Leds() {
    this.strip = new AddressableLED(LedsConstants.kPort);
    this.buffer = new AddressableLEDBuffer(LedsConstants.kLength);
    this.strip.setLength(buffer.getLength());
    this.strip.setData(buffer);
    this.strip.start();

    // sorted in descending order of priority
    signals = new TreeSet<Signal>(Comparator.comparingInt(Signal::priority).reversed());
  }

  // register signals into the tree set. If priority is already taken, don't add it
  public void registerSignal(
      int priority, BooleanSupplier condition, Supplier<LEDPattern> pattern) {
    final var priorityIsAlreadyClaimed =
        signals.stream().mapToInt(Signal::priority).anyMatch(p -> p == priority);

    if (priorityIsAlreadyClaimed) {
      DriverStation.reportWarning("Priority " + priority + " is already claimed", true);
      return;
    }

    signals.add(new Signal(priority, condition, pattern));
  }

  // default command to turn off leds
  public Command off() {
    return run(
        () -> {
          LEDPattern.kOff.applyTo(buffer);
          strip.setData(buffer);
        });
  }

  // default command to turn on & update leds
  public Command updateLeds() {
    return run(
        () -> {
          // set new pattern to the pattern of the most important signal that is true
          for (var signal : signals) {
            if (signal.condition.getAsBoolean()) {
              currentPattern = signal.pattern.get();
              break;
            }
          }

          // apply the pattern to the led strip
          currentPattern.applyTo(buffer);
          strip.setData(buffer);
        });
  }
}
