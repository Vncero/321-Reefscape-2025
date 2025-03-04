/* (C) Robolancers 2025 */
package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import java.util.function.DoubleSupplier;

/*
 * There is also potential for LED patterns to indicate error states e.g. gyro disconnect/motor burnout,
 * which was part of the original intent of the priority system but has not been leveraged. Consider adding such
 * LED signals (otherwise the priority system is arbitrarily applied to the current patterns, which mostly reflect
 * steps in the scoring cycle). - Vincent Z
 */

@Logged
public class LedsConstants {
  public static final int kPort = 0;
  public static final int kLength = 30;
  public static final Time kBlinkSpeed = Seconds.of(0.3);
  public static final Color kAlgaeColor = new Color(133, 226, 203);

  // DRIVING PATTERNS
  // default mode - meteor yellow. TODO: add meteor pattern or something cool like that
  public static final LEDPattern kDefault = LEDPattern.kOff;

  // climb mode - solid blue
  public static final LEDPattern kClimbing = LEDPattern.solid(Color.kBlue);

  // when we are far away and only aligning rotationally - solid red
  public static final LEDPattern kRotationAligning = LEDPattern.solid(Color.kRed);

  // when the robot has a pose to align to - solid yellow
  public static final LEDPattern kReadyToAlign = LEDPattern.solid(Color.kYellow);

  // when the robot is aligning to a pose - progress bar
  private static final LEDPattern progressBarPattern =
      LEDPattern.gradient(
          GradientType.kDiscontinuous,
          Color.kRed,
          Color.kOrange,
          Color.kYellow,
          Color.kGreenYellow,
          Color.kGreen);

  public static final LEDPattern kReefAligning(DoubleSupplier supp) {
    return progressBarPattern.mask(LEDPattern.progressMaskLayer(supp));
  }

  // when the driver interrupts the aligning process
  public static final LEDPattern kAlignOverride = LEDPattern.solid(Color.kPurple);

  // aligned and ready to score - solid green
  public static final LEDPattern kAligned = LEDPattern.solid(Color.kGreen);

  // has coral - solid white
  public static final LEDPattern kHasCoral = LEDPattern.solid(Color.kWhite);

  // has algae - solid algaeColor
  public static final LEDPattern kHasAlgae = LEDPattern.solid(kAlgaeColor);

  // has coral and algae - gradient of pink & light green (patrick star)
  private static final LEDPattern hasCoralAndAlgaePattern =
      LEDPattern.gradient(
          LEDPattern.GradientType.kContinuous, Color.kHotPink, new Color(172, 220, 65));
  public static final LEDPattern kHasCoralAndAlgae =
      hasCoralAndAlgaePattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.75), Meters.of(1.5));

  // Intaking - solid orange
  public static final LEDPattern kIntaking = LEDPattern.solid(Color.kOrange);

  // Outtaking - blinking orange
  public static final LEDPattern kOuttaking = kIntaking.blink(kBlinkSpeed);

  // progress bar for aligning w/ aligning colors

  // ERROR STATE Patterns
  private static final LEDPattern visionDisconnectPattern = LEDPattern.rainbow(255, 255);
  public static final LEDPattern kVisionDisconnect =
      visionDisconnectPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), Meters.of(1));
}
