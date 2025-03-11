package frc.robot.subsystems.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
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
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.operator_interface.OISelector;

/** TalonFX implementation of the generic SubsystemIO */
public class ManipulatorIOTalonFX implements ManipulatorIO {
  private TalonFX funnelMotor;
  private TalonFX indexerMotor;
  private TalonFX pivotMotor; //new pivot motor for algae claw

  // ir sensors
  private DigitalInput funnelIRSensor;
  private DigitalInput indexerIRSensor;
  private DigitalInput algaeIRSensor; //new algae IR sensor

  private DigitalInput backupFunnelIRSensor;
  private DigitalInput backupIndexerIRSensor;
  private DigitalInput backupAlgaeIRSensor;

  private VoltageOut funnelVoltageRequest;
  private VoltageOut indexerVoltageRequest;
  private VoltageOut pivotVoltageRequest; //new for pivot motor

  private TorqueCurrentFOC funnelCurrentRequest;
  private TorqueCurrentFOC indexerCurrentRequest;

  private VelocityTorqueCurrentFOC funnelVelocityRequest;
  private VelocityTorqueCurrentFOC indexerVelocityRequest;

  private MotionMagicExpoVoltage pivotPositionRequest; //new for pivot motor

  private Alert configAlert =
      new Alert("Failed to apply configuration for manipulator.", AlertType.kError);

  private Alert refreshAlert = new Alert("Failed to refresh all signals.", AlertType.kError);

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
    
      //new tunable values for pivot motor

    private final LoggedTunableNumber pivotKp =
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
  private ArmSystemSim pivotMotorSim; //new for pivot motor might need to change type to ArmSystemSim

  // Create StatusSignal objects for each loggable input from the ManipulatorIO class in the
  // updateInputs method
  // change type of each status signal objecty to its corresponding type

  private StatusSignal<AngularVelocity> funnelMotorVelocity;
  private StatusSignal<AngularVelocity> indexerMotorVelocity;

  private StatusSignal<Current> funnelMotorStatorCurrent;
  private StatusSignal<Current> indexerMotorStatorCurrent;
  private StatusSignal<Current> pivotMotorStatorCurrent; //stator current for pivot motor

  private StatusSignal<Temperature> funnelMotorTemp;
  private StatusSignal<Temperature> indexerMotorTemp;
  private StatusSignal<Temperature> pivotMotorTemp; //temp for pivot motor

  private StatusSignal<Current> funnelMotorSupplyCurrent;
  private StatusSignal<Current> indexerMotorSupplyCurrent;
  private StatusSignal<Current> pivotMotorSupplyCurrent; //supply current for pivot motor

  private StatusSignal<Voltage> funnelMotorVoltage;
  private StatusSignal<Voltage> indexerMotorVoltage;
  private StatusSignal<Voltage> pivotMotorVoltage; //voltage for pivot motor

  private StatusSignal<Angle> pivotMotorAngle; //angle for pivot motor (of type angle)



  /** Create a TalonFX-specific generic SubsystemIO */
  public ManipulatorIOTalonFX() {

    funnelMotor = new TalonFX(FUNNEL_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    indexerMotor = new TalonFX(INDEXER_MOTOR_ID);
    pivotMotor = new TalonFX(PIVOT_MOTOR_ID);

    funnelIRSensor = new DigitalInput(FUNNEL_IR_SENSOR_ID);
    indexerIRSensor = new DigitalInput(INDEXER_IR_SENSOR_ID);
    algaeIRSensor = new DigitalInput(ALGAE_IR_SENSOR_ID);

    backupFunnelIRSensor = new DigitalInput(FUNNEL_IR_BACKUP_SENSOR_ID);
    backupIndexerIRSensor = new DigitalInput(INDEXER_IR_BACKUP_SENSOR_ID);
    backupAlgaeIRSensor = new DigitalInput(ALGAE_IR_BACKUP_SENSOR_ID);

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


    // Add sim for pivot motor
    pivotMotorSim =
      new ArmSystemSim(

      );

    funnelVoltageRequest = new VoltageOut(0.0);
    indexerVoltageRequest = new VoltageOut(0.0);
    pivotVoltageRequest = new VoltageOut(0.0); //new volatge request for pivot motor

    funnelCurrentRequest = new TorqueCurrentFOC(0.0);
    indexerCurrentRequest = new TorqueCurrentFOC(0.0);

    funnelVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    indexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

    pivotPositionRequest = new MotionMagicExpoVoltage(0);

    funnelMotorVelocity = funnelMotor.getVelocity();
    indexerMotorVelocity = indexerMotor.getVelocity();

    funnelMotorVoltage = funnelMotor.getMotorVoltage();
    indexerMotorVoltage = indexerMotor.getMotorVoltage();
    pivotMotorVoltage = pivotMotor.getMotorVoltage(); //new variable for pivot motor voltage

    funnelMotorSupplyCurrent = funnelMotor.getSupplyCurrent();
    indexerMotorSupplyCurrent = indexerMotor.getSupplyCurrent();
    pivotMotorSupplyCurrent = pivotMotor.getSupplyCurrent(); // new variable for pivot motor supply current

    funnelMotorTemp = funnelMotor.getDeviceTemp();
    indexerMotorTemp = indexerMotor.getDeviceTemp();
    pivotMotorTemp = pivotMotor.getDeviceTemp(); //new variable for temparture of pivot motor

    funnelMotorStatorCurrent = funnelMotor.getStatorCurrent();
    indexerMotorStatorCurrent = indexerMotor.getStatorCurrent();
    pivotMotorStatorCurrent = pivotMotor.getStatorCurrent(); //new variable for stator current of pivot motor
    
    pivotMotorAngle = pivotMotor.getPosition(); //get the position of the pivot motor

    configFunnelMotor(funnelMotor);
    configIndexerMotor(indexerMotor);
    configPivotMotor(pivotMotor); //configure pivot motor by calling congfigPivotMotor method
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ManipulatorIOInputs inputs) {

    // refresh all status signal objects for funnel motor
    StatusCode status =
        BaseStatusSignal.refreshAll(
            funnelMotorVelocity,
            funnelMotorStatorCurrent,
            funnelMotorTemp,
            funnelMotorSupplyCurrent,
            funnelMotorVoltage);
    Phoenix6Util.checkError(status, "Failed to refresh funnel motor signals.", refreshAlert);

    // refresh all status signal objects for indexer motor
    status =
        BaseStatusSignal.refreshAll(
            indexerMotorVelocity,
            indexerMotorStatorCurrent,
            indexerMotorTemp,
            indexerMotorSupplyCurrent,
            indexerMotorVoltage);
    Phoenix6Util.checkError(status, "Failed to refresh indexer motor signals.", refreshAlert);

    // refresh all status signal objects for pivot motor
    status =
        BaseStatusSignal.refreshAll(
            pivotMotorVelocity,
            pivotMotorStatorCurrent,
            pivotMotorTemp,
            pivotMotorSupplyCurrent,
            pivotMotorVoltage,
            pivotMotorAngle);
    Phoenix6Util.checkError(status, "Failed to refresh pivot motor signals.", refreshAlert);

    inputs.funnelVelocityRPS = funnelMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.indexerVelocityRPS = indexerMotorVelocity.getValue().in(RotationsPerSecond);

    inputs.funnelStatorCurrentAmps = funnelMotorStatorCurrent.getValueAsDouble();
    inputs.indexerStatorCurrentAmps = indexerMotorStatorCurrent.getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotMotorStatorCurrent.getValueAsDouble();

    inputs.funnelTempCelsius = funnelMotorTemp.getValueAsDouble();
    inputs.indexerTempCelsius = indexerMotorTemp.getValueAsDouble();
    inputs.pivotTempCelsius = pivotMotorTemp.getValueAsDouble();

    inputs.funnelSupplyCurrentAmps = funnelMotorSupplyCurrent.getValueAsDouble();
    inputs.indexerSupplyCurrentAmps = indexerMotorSupplyCurrent.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotMotorSupplyCurrent.getValueAsDouble();

    inputs.funnelClosedLoopErrorRPS = funnelMotor.getClosedLoopError().getValueAsDouble();
    inputs.indexerClosedLoopErrorRPS = indexerMotor.getClosedLoopError().getValueAsDouble();
    inputs.pivotClosedLoopErrorRPS = pivotMotor.getClosedLoopError().getValueAsDouble();

    inputs.funnelReferenceVelocityRPS = funnelMotor.getClosedLoopReference().getValueAsDouble();
    inputs.indexerReferenceVelocityRPS = indexerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.pivotReferencePositionRPS = pivotMotor.getClosedLoopReference().getValueAsDouble();

    inputs.funnelMotorVoltage = funnelMotorVoltage.getValueAsDouble();
    inputs.indexerMotorVoltage = indexerMotorVoltage.getValueAsDouble();
    inputs.pivotMotorVoltage = pivotMotorVoltage.getValueAsDouble();

    inputs.pivotMotorAngle = pivotMotorAngle.getValueAsDouble();

    if (OISelector.getOperatorInterface().getEnablePrimaryIRSensorsTrigger().getAsBoolean()) {
      inputs.isFunnelIRBlocked = !funnelIRSensor.get();
      inputs.isIndexerIRBlocked = !indexerIRSensor.get();
      inputs.isAlgaeIRBlocked = !algaeIRSensor.get(); //new for algae IR
    } else {
      inputs.isFunnelIRBlocked = !backupFunnelIRSensor.get();
      inputs.isIndexerIRBlocked = !backupIndexerIRSensor.get();
      inputs.isAlgaeIRBlocked = !backupAlgaeIRSensor.get();
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
    pivotMotorSim.updateSim();
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

  @Override
  public void setPivotMotorVoltage(double volts) {
    this.pivotMotor.setControl(pivotVoltageRequest.withOutput(volts));
  }

  @Override
  public void setPivotPosition(Angle angle) {
    this.pivotMotor.setControl(pivotPositionRequest.withPosition(angle));
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

   /*
   * This method configures the pivot motor.
   */
  private void configPivotMotor(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = PIVOT_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = PIVOT_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = PIVOT_MOTOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted =
        PIVOT_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue
                .CounterClockwise_Positive; // if else statement based on the value of the
    // PIVOT_motor_inverted, sets the motor to a specific
    // value based on if the motor is inverted (first
    // choice) or not inverted (second choice)

    Phoenix6Util.applyAndCheckConfiguration(motor, config, configAlert);

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "pivot motor", motor);
  }
}
