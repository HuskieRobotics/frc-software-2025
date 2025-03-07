package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput; // imported this class for the sensors
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.operator_interface.OISelector;

/** TalonFX implementation of the generic SubsystemIO */
public class ManipulatorIOTalonFX implements ManipulatorIO {
  private TalonFX funnelMotor;
  private TalonFX indexerMotor;

  // ir sensors
  private DigitalInput funnelIRSensor;
  private DigitalInput indexerIRSensor;

  private DigitalInput backupFunnelIRSensor;
  private DigitalInput backupIndexerIRSensor;

  private VoltageOut funnelVoltageRequest;
  private VoltageOut indexerVoltageRequest;

  private TorqueCurrentFOC funnelCurrentRequest;
  private TorqueCurrentFOC indexerCurrentRequest;

  private VelocityTorqueCurrentFOC funnelVelocityRequest;
  private VelocityTorqueCurrentFOC indexerVelocityRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for manipulator.", AlertType.kError);

  private final LoggedTunableNumber funnelKp =
      new LoggedTunableNumber("Manipulator/Funnel/kP", FUNNEL_MOTOR_KP);
  private final LoggedTunableNumber funnelKi =
      new LoggedTunableNumber("Manipulator/Funnel/kI", FUNNEL_MOTOR_KI);
  private final LoggedTunableNumber funnelKd =
      new LoggedTunableNumber("Manipulator/Funnel/kD", FUNNEL_MOTOR_KD);
  private final LoggedTunableNumber funnelKs =
      new LoggedTunableNumber("Manipulator/Funnel/kS", FUNNEL_MOTOR_KS);
  private final LoggedTunableNumber funnelKv =
      new LoggedTunableNumber("Manipulator/Funnel/kV", FUNNEL_MOTOR_KV);
  private final LoggedTunableNumber funnelKa =
      new LoggedTunableNumber("Manipulator/Funnel/kA", FUNNEL_MOTOR_KA);

  private final LoggedTunableNumber indexerKp =
      new LoggedTunableNumber("Manipulator/Indexer/kP", INDEXER_MOTOR_KP);
  private final LoggedTunableNumber indexerKi =
      new LoggedTunableNumber("Manipulator/Indexer/kI", INDEXER_MOTOR_KI);
  private final LoggedTunableNumber indexerKd =
      new LoggedTunableNumber("Manipulator/Indexer/kD", INDEXER_MOTOR_KD);
  private final LoggedTunableNumber indexerKs =
      new LoggedTunableNumber("Manipulator/Indexer/kS", INDEXER_MOTOR_KS);
  private final LoggedTunableNumber indexerKv =
      new LoggedTunableNumber("Manipulator/Indexer/kV", INDEXER_MOTOR_KV);
  private final LoggedTunableNumber indexerKa =
      new LoggedTunableNumber("Manipulator/Indexer/kA", INDEXER_MOTOR_KA);

  private VelocitySystemSim funnelMotorSim;
  private VelocitySystemSim indexerMotorSim;

  // Create StatusSignal objects for each loggable input from the ManipulatorIO class in the
  // updateInputs method
  // change type of each status signal objecty to its corresponding type

  private StatusSignal<AngularVelocity> funnelMotorVelocity;
  private StatusSignal<AngularVelocity> indexerMotorVelocity;

  private StatusSignal<Current> funnelMotorStatorCurrent;
  private StatusSignal<Current> indexerMotorStatorCurrent;

  private StatusSignal<Temperature> funnelMotorTemp;
  private StatusSignal<Temperature> indexerMotorTemp;

  private StatusSignal<Current> funnelMotorSupplyCurrent;
  private StatusSignal<Current> indexerMotorSupplyCurrent;

  private StatusSignal<Voltage> funnelMotorVoltage;
  private StatusSignal<Voltage> indexerMotorVoltage;

  private final Debouncer funnelConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer indexerConnectedDebouncer = new Debouncer(0.5);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ManipulatorIOTalonFX() {

    funnelMotor = new TalonFX(FUNNEL_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    indexerMotor = new TalonFX(INDEXER_MOTOR_ID);

    funnelIRSensor = new DigitalInput(FUNNEL_IR_SENSOR_ID);
    indexerIRSensor = new DigitalInput(INDEXER_IR_SENSOR_ID);

    backupFunnelIRSensor = new DigitalInput(FUNNEL_IR_BACKUP_SENSOR_ID);
    backupIndexerIRSensor = new DigitalInput(INDEXER_IR_BACKUP_SENSOR_ID);

    funnelMotorSim =
        new VelocitySystemSim(
            funnelMotor,
            FUNNEL_MOTOR_INVERTED,
            FUNNEL_MOTOR_KV,
            FUNNEL_MOTOR_KA,
            GEAR_RATIO_FUNNEL);
    indexerMotorSim =
        new VelocitySystemSim(
            indexerMotor,
            INDEXER_MOTOR_INVERTED,
            INDEXER_MOTOR_KV,
            INDEXER_MOTOR_KA,
            GEAR_RATIO_MANIPULATOR);

    funnelVoltageRequest = new VoltageOut(0.0);
    indexerVoltageRequest = new VoltageOut(0.0);

    funnelCurrentRequest = new TorqueCurrentFOC(0.0);
    indexerCurrentRequest = new TorqueCurrentFOC(0.0);

    funnelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    indexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

    funnelMotorVelocity = funnelMotor.getVelocity();
    indexerMotorVelocity = indexerMotor.getVelocity();

    funnelMotorVoltage = funnelMotor.getMotorVoltage();
    indexerMotorVoltage = indexerMotor.getMotorVoltage();

    funnelMotorSupplyCurrent = funnelMotor.getStatorCurrent();
    indexerMotorSupplyCurrent = indexerMotor.getStatorCurrent();

    funnelMotorTemp = funnelMotor.getDeviceTemp();
    indexerMotorTemp = indexerMotor.getDeviceTemp();

    funnelMotorStatorCurrent = funnelMotor.getStatorCurrent();
    indexerMotorStatorCurrent = indexerMotor.getStatorCurrent();

    Phoenix6Util.registerSignals(
        true,
        funnelMotorVelocity,
        funnelMotorVoltage,
        funnelMotorStatorCurrent,
        funnelMotorTemp,
        funnelMotorSupplyCurrent);
    Phoenix6Util.registerSignals(
        false,
        indexerMotorVelocity,
        indexerMotorVoltage,
        indexerMotorStatorCurrent,
        indexerMotorTemp,
        indexerMotorSupplyCurrent);

    configFunnelMotor(funnelMotor);
    configIndexerMotor(indexerMotor);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {

    inputs.funnelConnected =
        funnelConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                funnelMotorVelocity,
                funnelMotorVoltage,
                funnelMotorStatorCurrent,
                funnelMotorTemp,
                funnelMotorSupplyCurrent));
    inputs.indexerConnected =
        indexerConnectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                indexerMotorVelocity,
                indexerMotorVoltage,
                indexerMotorStatorCurrent,
                indexerMotorTemp,
                indexerMotorSupplyCurrent));

    inputs.funnelVelocityRPS = funnelMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.indexerVelocityRPS = indexerMotorVelocity.getValue().in(RotationsPerSecond);

    inputs.funnelStatorCurrentAmps = funnelMotorStatorCurrent.getValueAsDouble();
    inputs.indexerStatorCurrentAmps = indexerMotorStatorCurrent.getValueAsDouble();

    inputs.funnelTempCelsius = funnelMotorTemp.getValueAsDouble();
    inputs.indexerTempCelsius = indexerMotorTemp.getValueAsDouble();

    inputs.funnelSupplyCurrentAmps = funnelMotorSupplyCurrent.getValueAsDouble();
    inputs.indexerSupplyCurrentAmps = indexerMotorSupplyCurrent.getValueAsDouble();

    // Getting the signal from the TalonFX is more time consuming than having the signal already
    // available. However, if we attempt to get the signal earlier, it won't be bound to the correct
    // control type. So, we only take the hit when tuning which is when this information is needed
    // more.
    if (Constants.TUNING_MODE) {
      inputs.funnelClosedLoopErrorRPS = funnelMotor.getClosedLoopError().getValueAsDouble();
      inputs.indexerClosedLoopErrorRPS = indexerMotor.getClosedLoopError().getValueAsDouble();

      inputs.funnelReferenceVelocityRPS = funnelMotor.getClosedLoopReference().getValueAsDouble();
      inputs.indexerReferenceVelocityRPS = indexerMotor.getClosedLoopReference().getValueAsDouble();
    }

    inputs.funnelMotorVoltage = funnelMotorVoltage.getValueAsDouble();
    inputs.indexerMotorVoltage = indexerMotorVoltage.getValueAsDouble();

    if (OISelector.getOperatorInterface().getEnablePrimaryIRSensorsTrigger().getAsBoolean()) {
      inputs.isFunnelIRBlocked = !funnelIRSensor.get();
      inputs.isIndexerIRBlocked = !indexerIRSensor.get();
    } else {
      inputs.isFunnelIRBlocked = !backupFunnelIRSensor.get();
      inputs.isIndexerIRBlocked = !backupIndexerIRSensor.get();
    }

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.funnelMotor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          config.Slot0.kS = pid[3];
          config.Slot0.kV = pid[4];
          config.Slot0.kA = pid[5];
          this.funnelMotor.getConfigurator().apply(config);
        },
        funnelKp,
        funnelKi,
        funnelKd,
        funnelKs,
        funnelKv,
        funnelKa);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.indexerMotor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          config.Slot0.kS = pid[3];
          config.Slot0.kV = pid[4];
          config.Slot0.kA = pid[5];
          this.indexerMotor.getConfigurator().apply(config);
        },
        indexerKp,
        indexerKi,
        indexerKd,
        indexerKs,
        indexerKv,
        indexerKa);

    // update funnel and indexer motor sim
    funnelMotorSim.updateSim();
    indexerMotorSim.updateSim();
  }

  /**
   * Set the motor voltage to the specified number of volts
   *
   * @param volts the volts to set the motor voltage to
   */
  @Override
  public void setFunnelMotorVoltage(double volts) {
    this.funnelMotor.setControl(funnelVoltageRequest.withOutput(volts));
  }

  /**
   * Set the motor power to the specified number of volts.
   *
   * @param volts the volts to set the motor voltage to
   */
  @Override
  public void setIndexerMotorVoltage(double volts) {
    this.indexerMotor.setControl(indexerVoltageRequest.withOutput(volts));
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  @Override
  public void setFunnelMotorCurrent(double current) {
    this.funnelMotor.setControl(funnelCurrentRequest.withOutput(current));
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  @Override
  public void setIndexerMotorCurrent(double current) {
    this.indexerMotor.setControl(indexerCurrentRequest.withOutput(current));
  }

  @Override
  public void setFunnelMotorVelocity(double velocity) {
    this.funnelMotor.setControl(funnelVelocityRequest.withVelocity(velocity));
  }

  @Override
  public void setIndexerMotorVelocity(double velocity) {
    this.indexerMotor.setControl(indexerVelocityRequest.withVelocity(velocity));
  }

  /*
   * This method configures the Funnel motor.
   */
  private void configFunnelMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = FUNNEL_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = FUNNEL_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = FUNNEL_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted =
        FUNNEL_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue
                .CounterClockwise_Positive; // if else statement based on the value of the
    // funnel_motor_inverted, sets the motor to a specific
    // value based on if the motor is inverted (first
    // choice) or not inverted (second choice)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = funnelKp.get();
    config.Slot0.kI = funnelKi.get();
    config.Slot0.kD = funnelKd.get();
    config.Slot0.kS = funnelKs.get();
    config.Slot0.kV = funnelKv.get();
    config.Slot0.kA = funnelKa.get();

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "funnel motor", motor);
  }

  /*
   * This method configures the Indexer motor.
   */
  private void configIndexerMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = INDEXER_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = INDEXER_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = INDEXER_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted =
        INDEXER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = indexerKp.get();
    config.Slot0.kI = indexerKi.get();
    config.Slot0.kD = indexerKd.get();
    config.Slot0.kS = indexerKs.get();
    config.Slot0.kV = indexerKv.get();
    config.Slot0.kA = indexerKa.get();

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "indexer motor", motor);
  }
}
