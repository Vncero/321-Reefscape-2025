/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ClimberIOSim implements ClimberIO {

  public static final ClimberConfig config = new ClimberConfig(0, 0, 0.0, 0);

  private SingleJointedArmSim climbSim;

  public ClimberIOSim() {
    // configures a simulated arm with two pivot motors controlling one
    // pivot point
    climbSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getNEO(1), ClimberConstants.kClimbMOI, ClimberConstants.kClimbGearing),
            DCMotor.getNEO(1),
            ClimberConstants.kClimbGearing,
            ClimberConstants.kClimbLength.in(Meters),
            ClimberConstants.kClimbMinAngle.in(Radians),
            ClimberConstants.kClimbMaxAngle.in(Radians),
            true,
            ClimberConstants.kStartingAngle.in(Radians));
  }

  public void setClimbVoltage(Voltage volts) {
    climbSim.setInputVoltage(volts.in(Volts));
  }

  public void updateInputs(ClimberInputs inputs) { // gets info to update inputs
    climbSim.update(0.02);
    inputs.climbAngle = Radians.of(climbSim.getAngleRads());
    inputs.climbVelocity = RadiansPerSecond.of(climbSim.getVelocityRadPerSec());
    inputs.climbCurrent = Amps.of(climbSim.getCurrentDrawAmps());
  }
}
