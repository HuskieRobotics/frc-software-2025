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
  private StatusSignal<Double> positionRotations;
  private StatusSignal<Double> positionInches;

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
    
    configMotor();

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
        positionRotations,
        positionInches
      );

      inputs.voltage = voltage.getValueAsDouble();
      inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
      inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
      inputs.tempCelcius = tempCelcius.getValueAsDouble();
      //convert below from position in rotation to position in inches
      inputs.positionRotations = positionRotations.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  public void zeroPosition() {
    //turn into a constant
    climberMotor.setPosition(0, 0);
  }

  private void configMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    //CurrentLimitsConfigs climberMotorCurrentLimits = new CurrentLimitsConfigs();
    config.CurrentLimits.SupplyCurrentLimit =
        ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
        //make sure this is all right
    config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    //config.CurrentLimits = climberMotorCurrentLimits;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    config.MotorOutput.Inverted =
        ClimberConstants.CLIMBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
  }
}
