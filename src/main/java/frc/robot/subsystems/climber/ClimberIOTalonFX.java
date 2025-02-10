package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3061.sim.ElevatorSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  private VoltageOut climberVoltageRequest;
  private ElevatorSystemSim elevatorSystemSim;

  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Temperature> tempCelcius;
  private StatusSignal<Angle> positionRotations;

  private final LoggedTunableNumber KP = new LoggedTunableNumber("Climber/KP", ClimberConstants.KP);
  private final LoggedTunableNumber KI = new LoggedTunableNumber("Climber/KI", ClimberConstants.KI);
  private final LoggedTunableNumber KD = new LoggedTunableNumber("Climber/KD", ClimberConstants.KD);
  private final LoggedTunableNumber KS = new LoggedTunableNumber("Climber/KS", ClimberConstants.KS);
  private final LoggedTunableNumber KV = new LoggedTunableNumber("Climber/KV", ClimberConstants.KV);
  private final LoggedTunableNumber KA = new LoggedTunableNumber("Climber/KA", ClimberConstants.KA);
  private final LoggedTunableNumber KVEXP =
      new LoggedTunableNumber("Climber/KVEXP", ClimberConstants.KVEXP);
  private final LoggedTunableNumber KAEXP =
      new LoggedTunableNumber("Climber/KAEXP", ClimberConstants.KAEXP);
  private final LoggedTunableNumber KG = new LoggedTunableNumber("Climber/KG", ClimberConstants.KG);

  private Alert refreshAlert = new Alert("Failed to refresh all signals.", AlertType.kError);

  public ClimberIOTalonFX() {
    climberMotor = new TalonFX(ClimberConstants.CLIMBER_MOTOR_CAN_ID);

    configMotor();

    voltage = climberMotor.getMotorVoltage();
    statorCurrentAmps = climberMotor.getStatorCurrent();
    supplyCurrentAmps = climberMotor.getSupplyCurrent();
    tempCelcius = climberMotor.getDeviceTemp();
    positionRotations = climberMotor.getPosition();

    climberVoltageRequest = new VoltageOut(0);
    // ask lauren for mass and max height
    elevatorSystemSim =
        new ElevatorSystemSim(
            climberMotor,
            ClimberConstants.CLIMBER_MOTOR_INVERTED,
            ClimberConstants.GEAR_RATIO,
            3,
            Units.inchesToMeters(ClimberConstants.DRUM_DIAMETER / 2.0),
            0,
            1,
            0,
            "Climber");
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    // Update loggable values here (using status signals)
    StatusCode status =
        BaseStatusSignal.refreshAll(
            voltage, statorCurrentAmps, supplyCurrentAmps, tempCelcius, positionRotations);
    Phoenix6Util.checkError(status, "Failed to refresh climber motor signals.", refreshAlert);

    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.tempCelcius = tempCelcius.getValueAsDouble();
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.positionInches = inputs.positionRotations * Math.PI * ClimberConstants.DRUM_DIAMETER;
    elevatorSystemSim.updateSim();
  }

  @Override
  public void setVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  @Override
  public void zeroPosition() {
    // turn into a constant
    climberMotor.setPosition(0, 0);
  }

  private void configMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    config.MotorOutput.Inverted =
        ClimberConstants.CLIMBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
  }
}
