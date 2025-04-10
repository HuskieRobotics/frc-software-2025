package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ElevatorSystemSim;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private VoltageOut climberVoltageRequest;
  private ElevatorSystemSim elevatorSystemSim;

  private DigitalInput limitSwitch1;
  private DigitalInput limitSwitch2;

  private StatusSignal<Voltage> voltage;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Temperature> tempCelsius;
  private StatusSignal<Angle> positionRotations;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ClimberIOTalonFX() {
    climberMotor =
        new TalonFX(
            ClimberConstants.CLIMBER_MOTOR_CAN_ID, RobotConfig.getInstance().getCANBusName());

    limitSwitch1 = new DigitalInput(ClimberConstants.CLIMBER_LIMIT_SWITCH_DIO_1);
    limitSwitch2 = new DigitalInput(ClimberConstants.CLIMBER_LIMIT_SWITCH_DIO_2);

    configMotor();

    voltage = climberMotor.getMotorVoltage();
    statorCurrentAmps = climberMotor.getStatorCurrent();
    supplyCurrentAmps = climberMotor.getSupplyCurrent();
    tempCelsius = climberMotor.getDeviceTemp();
    positionRotations = climberMotor.getPosition();

    Phoenix6Util.registerSignals(
        true, voltage, statorCurrentAmps, supplyCurrentAmps, tempCelsius, positionRotations);

    climberVoltageRequest = new VoltageOut(0);
    elevatorSystemSim =
        new ElevatorSystemSim(
            climberMotor,
            ClimberConstants.CLIMBER_MOTOR_INVERTED,
            5.0, // should be ClimberConstants.GEAR_RATIO but that kills performance
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
    inputs.connected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                voltage, statorCurrentAmps, supplyCurrentAmps, tempCelsius, positionRotations));
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.tempCelsius = tempCelsius.getValueAsDouble();
    inputs.positionRotations = positionRotations.getValueAsDouble();
    inputs.positionInches = inputs.positionRotations * Math.PI * ClimberConstants.DRUM_DIAMETER;
    inputs.limitSwitch1Engaged = limitSwitch1.get();
    inputs.limitSwitch2Engaged = limitSwitch2.get();
    elevatorSystemSim.updateSim();
  }

  @Override
  public void setVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  @Override
  public void zeroPosition() {
    climberMotor.setPosition(0);
  }

  private void configMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLowerLimit =
        ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimit = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ClimberConstants.CLIMBER_STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // was coast earlier

    config.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    config.MotorOutput.Inverted =
        ClimberConstants.CLIMBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(climberMotor, config, configAlert);
  }
}
