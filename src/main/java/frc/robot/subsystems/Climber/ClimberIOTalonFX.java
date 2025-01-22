import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.Climber.ClimberConstants;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  private VoltageOut climberVoltageRequest;

  private StatusSignal<Double> voltage;
  private StatusSignal<Double> statorCurrentAmps;
  private StatusSignal<Double> supplyCurrentAmps;
  private StatusSignal<Double> tempCelcius;
  private StatusSignal<Double> positionUnitsPerRotation;

  private final LoggedTunableNumber KP = new LoggedTunableNumber("Climber/KP", ClimberConstants.KP);
  private final LoggedTunableNumber KI = new LoggedTunableNumber("Climber/KI", ClimberConstants.KI);
  private final LoggedTunableNumber KD = new LoggedTunableNumber("Climber/KD", ClimberConstants.KD);
  private final LoggedTunableNumber KS = new LoggedTunableNumber("Climber/KS", ClimberConstants.KS);
  private final LoggedTunableNumber KV = new LoggedTunableNumber("Climber/KV", ClimberConstants.KV);
  private final LoggedTunableNumber KA = new LoggedTunableNumber("Climber/KA", ClimberConstants.KA);
  private final LoggedTunableNumber KVEXP = new LoggedTunableNumber("Climber/KVEXP", ClimberConstants.KVEXP);
  private final LoggedTunableNumber KAEXP = new LoggedTunableNumber("Climber/KAEXP", ClimberConstants.KAEXP);
  private final LoggedTunableNumber KG = new LoggedTunableNumber("Climber/KG", ClimberConstants.KG);

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);
    
    configClimberMotors();

    climberVoltageRequest = new VoltageOut(0);
    
    

  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
      // Update loggable values here (using status signals)
      BaseStatusSignal.refreshAll(
        voltage,
        statorCurrentAmps,
        supplyCurrentAmps,
        tempCelcius,
        positionUnitsPerRotation
      );

      inputs.voltage = voltage.getValueAsDouble();
      inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
      inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
      inputs.tempCelcius = tempCelcius.getValueAsDouble();
      inputs.position_units_per_rotation = positionUnitsPerRotation.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  public void zeroPosition() {
    //turn into a constant
    climberMotor.setPosition(0, 0);
  }

  private void configClimberMotors() {
    TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs climberMotorCurrentLimits = new CurrentLimitsConfigs();
    climberMotorCurrentLimits.SupplyCurrentLimit =
        ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
    climberMotorCurrentLimits.SupplyCurrentThreshold = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    climberMotorCurrentLimits.SupplyTimeThreshold = ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
    climberMotorCurrentLimits.SupplyCurrentLimitEnable = true;
    climberMotorConfig.CurrentLimits = climberMotorCurrentLimits;

    climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberMotorConfig.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    climberMotorConfig.MotorOutput.Inverted =
        ClimberConstants.CLIMBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
  }
}
