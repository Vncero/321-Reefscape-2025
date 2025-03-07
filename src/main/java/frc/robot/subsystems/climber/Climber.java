/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

@Logged
public class Climber extends SubsystemBase {

  private ClimberIO io;
  private ClimberInputs inputs;

  private PIDController climbController;
  private ArmFeedforward feedForward;
  private ClimberConfig config;

  Timer timer = new Timer();

  public Climber(ClimberIO io, ClimberConfig config) {
    this.io = io;
    this.inputs = new ClimberInputs();
    this.config = config;

    climbController = new PIDController(config.kP(), config.kI(), config.kD());
    feedForward = new ArmFeedforward(0, config.kG(), 0);

    climbController.setTolerance(ClimberConstants.kControllerTolerance.in(Degrees));

    io.resetEncoder(ClimberConstants.kStartingAngle);
  }

  public static Climber create() {
    return RobotBase.isReal() // TODO: possibly change from spark to kraken
        ? new Climber(
            new ClimberIOSpark(),
            ClimberIOSpark.config) // creates real mechanism if the code is running on a robot
        : new Climber(
            new ClimberIOSim(),
            ClimberIOSim.config); // creates a sim mechanism if the code is not on a real robot
  }

  public Command tune() {
    TunableConstant kP = new TunableConstant("/ClimberPivos/kP", config.kP());
    TunableConstant kI = new TunableConstant("/ClimberPivot/kI", config.kI());
    TunableConstant kD = new TunableConstant("/ClimberPivot/kD", config.kD());
    TunableConstant kV = new TunableConstant("/ClimberPivot/kG", config.kG());
    TunableConstant desiredAngle = new TunableConstant("/ClimberPivot/desiredAngle", 0);

    return run(
        () -> {
          this.climbController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new ArmFeedforward(0, kG.get(), 0);
          goToAngle(Degrees.of(desiredAngle.get()));
        });
  }

  // creates placeholder implementation to disable robot
  public static Climber disable() {
    return new Climber(new ClimberIOIdeal(), ClimberIOIdeal.config);
  }

  // get to a desired angle by setting pivot voltage to sum of calculated pid and feedforward
  public void goToAngle(Angle desiredAngle) {
    Voltage desiredVoltage =
        Volts.of(
            feedForward.calculate(desiredAngle.in(Radians), 0)
                + climbController.calculate(
                    inputs.climbAngle.in(Degrees), desiredAngle.in(Degrees)));

    io.setClimbVoltage(desiredVoltage);
  }

  /*
   * Sets the motor to a current control mode, ramping up current over time
   */
  public void regulateClimbCurrent() {
    io.setClimbCurrent(
        Amps.of(
            Math.min(
                ClimberConstants.kClimbCurrentRampRate.in(Amps) * timer.get(),
                ClimberConstants.kClimbCurrent.in(Amps))));
  }

  public void stopClimbCurrent() {
    io.setClimbCurrent(Amps.of(0));
  }

  public Command climb() {
    Timer timer = new Timer();
    return runOnce(timer::restart)
        .andThen(
            run(() -> regulateClimbCurrent())
                .until(
                    () ->
                        inputs.climbAngle.in(Degrees)
                                >= ClimberConstants.kClimbThreshold.in(Degrees)
                            || inputs.climbCurrent.in(Amps) <= 0.0)) // Stop if cage slips
        .finallyDo(() -> stopClimbCurrent());
  }

  public Angle getAngle() {
    return inputs.climbAngle;
  }

  @Override
  public void periodic() { // updating inputs
    io.updateInputs(inputs);
  }
}
