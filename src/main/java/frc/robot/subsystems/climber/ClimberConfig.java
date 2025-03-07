/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import edu.wpi.first.epilogue.Logged;

@Logged
public record ClimberConfig(double kP, double kI, double kD, double kV) {}
