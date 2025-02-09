/* (C) Robolancers 2025 */
package frc.robot.util;

public class MathUtils {

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(b - a) < epsilon;
  }

  public static double squareKeepSign(double x) {
    return Math.signum(x) * x * x;
  }

  public static double deadband(double x, double deadband) {
    return Math.abs(x) < deadband ? 0 : x;
  }
}
