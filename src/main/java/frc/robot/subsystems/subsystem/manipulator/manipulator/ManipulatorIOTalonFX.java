package frc.robot.subsystems.subsystem.manipulator.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.subsystem.manipulator.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput; //imported this class for the sensors
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import com.ctre.phoenix6.signals.InvertedValue; //imported to invert the funnel motor

/** TalonFX implementation of the generic SubsystemIO */
public class ManipulatorIOTalonFX implements ManipulatorIO {
  private TalonFX funnelMotor;
  private TalonFX indexerMotor;

  //ir sensors
  private DigitalInput funnelIRSensor;
  private DigitalInput indexerIRSensor;

  private DigitalInput backupFunnelIrSensor;
  private DigitalInput backupIndexerIrSensor;

  private VoltageOut funnelVoltageRequest;
  private VoltageOut indexerVoltageRequest;

  private TorqueCurrentFOC funnelCurrentRequest;
  private TorqueCurrentFOC indexerCurrentRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for manipulator.", AlertType.kError);

/* You will create StatusSignal<> objects for each logged input. 
  These status signals will then be insantiated in the constructor with what value they should be tracking, which you do in updateInputs right now. 
  In updateInputs, you will refresh all of the status signals, 
  and set each input variable by retrieving each of the values of your statussignals with the statussignal.getValueAs___() methods.
*/
/*
  private StatusSignal<LoggedTunableNumber> kP;
  private StatusSignal<LoggedTunableNumber> kI;
  private StatusSignal<LoggedTunableNumber> kD;
  private StatusSignal<LoggedTunableNumber> kPeakOuput;
*/
  
  private final LoggedTunableNumber funnelKp = new LoggedTunableNumber("Subsystem/kP", FUNNEL_MOTOR_KP);
  private final LoggedTunableNumber funnelKi = new LoggedTunableNumber("Subsystem/kI", FUNNEL_MOTOR_KI);
  private final LoggedTunableNumber funnelKd = new LoggedTunableNumber("Subsystem/kD", FUNNEL_MOTOR_KD);
  private final LoggedTunableNumber funnelKs = new LoggedTunableNumber("Subsystem/kS", FUNNEL_MOTOR_KS);

  private final LoggedTunableNumber indexerKp = new LoggedTunableNumber("Subsystem/kP", INDEXER_MOTOR_KP);
  private final LoggedTunableNumber indexerKi = new LoggedTunableNumber("Subsystem/kI", INDEXER_MOTOR_KI);
  private final LoggedTunableNumber indexerKd = new LoggedTunableNumber("Subsystem/kD", INDEXER_MOTOR_KD);
  private final LoggedTunableNumber indexerKs = new LoggedTunableNumber("Subsystem/kS", INDEXER_MOTOR_KS);


  private VelocitySystemSim funnelMotorSim;
  private VelocitySystemSim indexerMotorSim;

  // Create StatusSignal objects for each loggable input from the ManipulatorIO class in the updateInputs method
  //change type of each status signal objecty to its corresponding type

  private StatusSignal<AngularVelocity> funnelMotorVelocity;
  private StatusSignal<AngularVelocity> indexerMotorVelocity;

 // private StatusSignal<Double> closedLoopError;
  private StatusSignal<Current> funnelMotorStatorCurrentAmps;
  private StatusSignal<Current> indexerMotorStatorCurrentAmps;
  private StatusSignal<Temperature> funnelMotorTempCelsius;
  private StatusSignal<Temperature> indexerMotorTempCelsius;
  private StatusSignal<Current> funnelMotorSupplyCurrentAmps;
  private StatusSignal<Current> indexerMotorSupplyCurrentAmps;

  /** Create a TalonFX-specific generic SubsystemIO */
  public ManipulatorIOTalonFX() {

    funnelMotor = new TalonFX(FUNNEL_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    indexerMotor = new TalonFX(INDEXER_MOTOR_ID);

    funnelIRSensor = new DigitalInput(FUNNEL_IR_SENSOR_CAN_ID);
    indexerIRSensor = new DigitalInput(INDEXER_IR_SENSOR_CAN_ID);

    backupFunnelIrSensor = new DigitalInput(FUNNEL_IR_BACKUP_SENSOR_ID);
    backupIndexerIrSensor = new DigitalInput(INDEXER_IR_BACKUP_SENSOR_ID);

    funnelMotorSim = new VelocitySystemSim(funnelMotor, FUNNEL_MOTOR_INVERTED, FUNNEL_MOTOR_KV, FUNNEL_MOTOR_KA, GEAR_RATIO);
    indexerMotorSim = new VelocitySystemSim(indexerMotor, INDEXER_MOTOR_INVERTED, INDEXER_MOTOR_KV, INDEXER_MOTOR_KA, GEAR_RATIO);

    funnelVoltageRequest = new VoltageOut(0.0);
    indexerVoltageRequest = new VoltageOut(0.0);

  
    funnelMotorVelocity = funnelMotor.getVelocity();
    indexerMotorVelocity = indexerMotor.getVelocity();
    
    funnelMotorSupplyCurrentAmps = funnelMotor.getStatorCurrent();
    indexerMotorSupplyCurrentAmps = indexerMotor.getStatorCurrent();

    funnelMotorTempCelsius = funnelMotor.getDeviceTemp();
    indexerMotorTempCelsius = indexerMotor.getDeviceTemp();

    funnelMotorStatorCurrentAmps = funnelMotor.getStatorCurrent();
    indexerMotorStatorCurrentAmps = indexerMotor.getStatorCurrent();


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

    //refresh all status signal objects
    BaseStatusSignal.refreshAll(
        funnelMotorVelocity,
        indexerMotorVelocity,
        funnelMotorStatorCurrentAmps,
        indexerMotorStatorCurrentAmps,
        funnelMotorTempCelsius,
        indexerMotorTempCelsius,
        funnelMotorSupplyCurrentAmps,
        indexerMotorSupplyCurrentAmps
    );

    inputs.velocityRPSFunnel = funnelMotorVelocity.getValue().in(RotationsPerSecond);
    inputs.velocityRPSIndexer = indexerMotorVelocity.getValue().in(RotationsPerSecond);

    inputs.statorCurrentAmpsFunnel = funnelMotorStatorCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmpsIndexer = indexerMotorStatorCurrentAmps.getValueAsDouble();

    inputs.tempCelsiusFunnel = funnelMotorTempCelsius.getValueAsDouble();
    inputs.tempCelsiusIndexer = indexerMotorTempCelsius.getValueAsDouble();

    inputs.supplyCurrentAmpsFunnel = funnelMotorSupplyCurrentAmps.getValueAsDouble();
    inputs.supplyCurrentAmpsIndexer = indexerMotorSupplyCurrentAmps.getValueAsDouble();

    inputs.funnelClosedLoopError = funnelMotor.getClosedLoopError().getValueAsDouble();
    inputs.indexerClosedLoopError = indexerMotor.getClosedLoopError().getValueAsDouble();

    inputs.funnelClosedLoopReference = funnelMotor.getClosedLoopReference().getValueAsDouble();
    inputs.indexerClosedLoopReference = indexerMotor.getClosedLoopReference().getValueAsDouble();


    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.funnelMotor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          this.funnelMotor.getConfigurator().apply(config);
        },
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        peak -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.indexerMotor.getConfigurator().refresh(config);
          config.Voltage.PeakForwardVoltage = peak[0];
          config.Voltage.PeakReverseVoltage = peak[0];
          this.indexerMotor.getConfigurator().apply(config);
        },
        kPeakOutput);
        
    LoggedTunableNumber.ifChanged(
        hashCode(),
        pid -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.indexerMotor.getConfigurator().refresh(config);
          config.Slot0.kP = pid[0];
          config.Slot0.kI = pid[1];
          config.Slot0.kD = pid[2];
          this.indexerMotor.getConfigurator().apply(config);
        },
        kP,
        kI,
        kD);
    LoggedTunableNumber.ifChanged(
      hashCode(),
        peak -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.indexerMotor.getConfigurator().refresh(config);
          config.Voltage.PeakForwardVoltage = peak[0];
          config.Voltage.PeakReverseVoltage = peak[0];
          this.indexerMotor.getConfigurator().apply(config);
        },
        kPeakOutput);

    //update funnel and indexer motor sim
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

  /*
   * This method configures the Funnel motor.
   */
  private void configFunnelMotor(TalonFX motor) {

   //this.funnelMotor = new TalonFX(motorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = FUNNEL_MOTOR_PEAK_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerLimit = FUNNEL_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerTime = FUNNEL_MOTOR_PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = FUNNEL_MOTOR_PEAK_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        FUNNEL_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive; //if else statement based on the value of the funnel_motor_inverted, sets the motor to a specific value based on if the motor is inverted (first choice) or not inverted (second choice)
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    Phoenix6Util.applyAndCheckConfiguration(this.funnelMotor, config, configAlert);

    /*
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    */

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", funnelMotor);
  }

  /*
   * This method configures the Indexer motor.
   */
  private void configIndexerMotor(TalonFX motor){ 

    //this.indexerMotor = new TalonFX(motorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = INDEXER_MOTOR_PEAK_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerLimit = INDEXER_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerTime = INDEXER_MOTOR_PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = INDEXER_MOTOR_PEAK_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
    INDEXER_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    Phoenix6Util.applyAndCheckConfiguration(this.indexerMotor, config, configAlert);

    this.indexerMotor.setPosition(0);

    /*
    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    */

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", indexerMotor);
  }
}
