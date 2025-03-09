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
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

// deep climb mechanism
@Logged
public class Climber extends SubsystemBase {

  private ClimberIO io;
  private ClimberInputs inputs;
  private ClimberConfig config;

  private PIDController climbController;
  private ArmFeedforward feedForward;

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
    return RobotBase.isReal()
        ? new Climber(
            new ClimberIOSpark(),
            ClimberIOSpark.config) // creates real mechanism if the code is running on a robot
        : new Climber(
            new ClimberIOSim(),
            ClimberIOSim.config); // creates a sim mechanism if the code is not on a real robot
  }

  public Command tune() {
    TunableConstant kP = new TunableConstant("/Climber/kP", config.kP());
    TunableConstant kI = new TunableConstant("/Climber/kI", config.kI());
    TunableConstant kD = new TunableConstant("/Climber/kD", config.kD());
    TunableConstant kG = new TunableConstant("/Climber/kG", config.kG());
    TunableConstant desiredAngle = new TunableConstant("/Climber/desiredAngle", 0);

    return run(
        () -> {
          this.climbController.setPID(kP.get(), kI.get(), kD.get());
          this.feedForward = new ArmFeedforward(0, kG.get(), 0);
          goToAngle(Degrees.of(desiredAngle.get()));
        });
  }

  // tunes kCurrentRampRate - how much should current be ramping per second
  public Command tuneCurrentRampRate() {
    TunableConstant climbCurrentRampRate =
        new TunableConstant(
            "/Climber/ClimbCurrentRampRate", ClimberConstants.kClimbCurrentRampRate.in(Amps));

    return run(
        () -> {
          double newRampRate = climbCurrentRampRate.get();
          io.setClimbCurrent(
              Amps.of(
                  // Ensures the current never exceeds the max
                  // ramps up current over time as we tune
                  Math.min(newRampRate * timer.get(), ClimberConstants.kClimbCurrent.in(Amps))));
        });
  }

  // creates placeholder implementation to disable robot
  public static Climber disable() {
    return new Climber(new ClimberIOIdeal(), ClimberIOIdeal.config);
  }

  /*
   * Climb command
   * Courtesy of 6328's implementation <3
   */
  public Command climb() {
    return runOnce(timer::restart)
        .andThen(
            run(() ->
                    io.setClimbCurrent(
                        Amps.of( // Sets the motor to a current control mode, ramping up current
                            // over time
                            Math.min(
                                ClimberConstants.kClimbCurrentRampRate.in(Amps) * timer.get(),
                                ClimberConstants.kClimbCurrent.in(Amps)))))
                .until(
                    () ->
                        inputs.climbAngle.in(Degrees)
                            <= ClimberConstants.kClimbThreshold.in(Degrees)))
        .andThen(() -> io.setLockServo(ClimberConstants.kServoLockPosition))
        .finallyDo(() -> io.stopClimbCurrent());
  }

  // get to a desired angle by setting pivot voltage to sum of calculated pid and feedforward
  public Command goToAngle(Angle desiredAngle) {
    return run(
        () -> {
          Voltage desiredVoltage =
              Volts.of(
                  feedForward.calculate(desiredAngle.in(Radians), 0)
                      + climbController.calculate(
                          inputs.climbAngle.in(Degrees), desiredAngle.in(Degrees)));

          io.setClimbVoltage(desiredVoltage);
        });
  }

  // sets voltage to the whole mechanism
  public Command setMechanismVoltage(Supplier<Voltage> volts) {
    return run(
        () -> {
          io.setClimbVoltage(volts.get());
        });
  }

  // sets current to the whole mechanism
  public Command setMechanismCurrent(Supplier<Current> current) {
    return run(
        () -> {
          io.setClimbCurrent(current.get());
        });
  }

  public Angle getAngle() {
    return inputs.climbAngle;
  }

  @Override
  public void periodic() { // updating inputs
    io.updateInputs(inputs);
  }
}
