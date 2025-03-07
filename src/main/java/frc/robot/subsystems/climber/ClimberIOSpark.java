/* (C) Robolancers 2025 */
package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

// spark implementation of the climb subsystem

@Logged
public class ClimberIOSpark implements ClimberIO {

  public static final ClimberConfig config = new ClimberConfig(0, 0, 0, 0);

  private SparkMax climbMotor = new SparkMax(ClimberConstants.kMotorId, MotorType.kBrushless);

  public void configureMotors() {
    climbMotor.configure( // configures single motor
        new SparkMaxConfig()
            .inverted(ClimberConstants.kClimbInverted)
            .voltageCompensation(ClimberConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(ClimberConstants.kSmartCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void regulateClimbCurrent() {
    
  }

}

