import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.Climber.ClimberConstants;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  private VoltageOut climberVoltageRequest;

  private StatusSignal<Double> voltage;
  private StatusSignal<Double> staterCurrentAmps;
  private StatusSignal<Double> supplyCurrentAmps;
  private StatusSignal<Double> tempCelcius;
  private StatusSignal<Double> position_units_per_rotation;

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
    configClimberMotors();

    climberVoltageRequest = new VoltageOut(0);
    
    

  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
      // Update loggable values here (using status signals)
      BaseStatusSignal.refreshAll(
        voltage,
        staterCurrentAmps,
        supplyCurrentAmps,
        tempCelcius,
        position_units_per_rotation
      );

      inputs.voltage = voltage.getValueAsDouble();
      inputs.staterCurrentAmps = staterCurrentAmps.getValueAsDouble();
      inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
      inputs.tempCelcius = tempCelcius.getValueAsDouble();
      inputs.position_units_per_rotation = position_units_per_rotation.getValueAsDouble();
  }

  public void setVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  public void zeroPosition() {
    climberMotor.setPosition(0, 0);
  }

  private void configClimberMotors() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);
    CurrentLimitsConfigs climberMotorCurrentLimits =  new CurrentLimitsConfigs();
    climberMotorCurrentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    climberMotorCurrentLimits.SupplyCurrentLowerTime = ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
  
  }
}
