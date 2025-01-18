package frc.robot.subsystems.subsystem.manipulator.subsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.subsystem.manipulator.subsystem.SubsystemConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team6328.util.LoggedTunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class ManipulatorIOTalonFX implements ManipulatorIO {
  private TalonFX funnelMotor;
  private TalonFX indexerMotor;

  private VoltageOut voltageRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private final LoggedTunableNumber kP = new LoggedTunableNumber("Subsystem/kP", POSITION_PID_P);
  private final LoggedTunableNumber kI = new LoggedTunableNumber("Subsystem/kI", POSITION_PID_I);
  private final LoggedTunableNumber kD = new LoggedTunableNumber("Subsystem/kD", POSITION_PID_D);
  private final LoggedTunableNumber kPeakOutput =
      new LoggedTunableNumber("Subsystem/kPeakOutput", POSITION_PID_PEAK_OUTPUT);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ManipulatorIOTalonFX() {
    configMotor(MOTOR_CAN_ID);
  }

  //NOTE FOR FATEMA: ALL THE RED LINES UNDER MOTOR IS BC I CHANGED THE INSTANCE VARIABLES FROM ONE MOTOR TO 2, SO ALL THE STUFF DEALING WITH THE INSTANCE VARIABLE NEEDS TO BE UPDATED ACCORDINGLY

  /**
   * Update the inputs based on the current state of the TalonFX funnel motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateFunnelMotorInputs(SubsystemIOInputs inputs) { 
    inputs.positionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            BaseStatusSignal.getLatencyCompensatedValue(
                    funnelMotor.getRotorPosition(), funnelMotor.getRotorVelocity())
                .in(Rotations),
            GEAR_RATIO);
    inputs.velocityRPM =
        Conversions.falconRPSToMechanismRPM(
            funnelMotor.getRotorVelocity().getValue().in(RotationsPerSecond), GEAR_RATIO);
    inputs.closedLoopError = funnelMotor.getClosedLoopError().getValue();
    inputs.setpoint = funnelMotor.getClosedLoopReference().getValue();
    inputs.power = funnelMotor.getDutyCycle().getValue();
    inputs.controlMode = funnelMotor.getControlMode().toString();
    inputs.statorCurrentAmps = funnelMotor.getStatorCurrent().getValue().in(Amps);
    inputs.tempCelsius = funnelMotor.getDeviceTemp().getValue().in(Celsius);
    inputs.supplyCurrentAmps = funnelMotor.getSupplyCurrent().getValue().in(Amps);

    // update configuration if tunables have changed
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
  }

  /**
   * Update the inputs based on the current state of the TalonFX indexer motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateIndexerMotorInputs(SubsystemIOInputs inputs) 
  {
    inputs.positionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            BaseStatusSignal.getLatencyCompensatedValue(
                    indexerMotor.getRotorPosition(), indexerMotor.getRotorVelocity())
                .in(Rotations),
            GEAR_RATIO);
    inputs.velocityRPM =
        Conversions.falconRPSToMechanismRPM(
            indexerMotor.getRotorVelocity().getValue().in(RotationsPerSecond), GEAR_RATIO);
    inputs.closedLoopError = indexerMotor.getClosedLoopError().getValue();
    inputs.setpoint = indexerMotor.getClosedLoopReference().getValue();
    inputs.power = indexerMotor.getDutyCycle().getValue();
    inputs.controlMode = indexerMotor.getControlMode().toString();
    inputs.statorCurrentAmps = indexerMotor.getStatorCurrent().getValue().in(Amps);
    inputs.tempCelsius = indexerMotor.getDeviceTemp().getValue().in(Celsius);
    inputs.supplyCurrentAmps = indexerMotor.getSupplyCurrent().getValue().in(Amps);

    // update configuration if tunables have changed
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
  }
  /**
   * Set the motor voltage to the specified number of volts
   *
   * @param volts the volts to set the motor voltage to
   */
  @Override
  public void setFunnelMotorVoltage(double volts) {
    this.funnelMotor.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Set the motor power to the specified number of volts.
   *
   * @param volts the volts to set the motor voltage to
   */
  @Override
  public void setIndexerMotorVoltage(double volts) {
    this.indexerMotor.setControl(voltageRequest.withOutput(volts));
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  @Override
  public void setFunnelMotorCurrent(double current) {
    this.funnelMotor.setControl(currentRequest.withOutput(current));
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  @Override
  public void setIndexerMotorCurrent(double current) {
    this.indexerMotor.setControl(currentRequest.withOutput(current));
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setFunnelMotorPosition(double position, double arbitraryFeedForward) {
    this.funnelMotor.setControl(
        positionRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setIndexerMotorPosition(double position, double arbitraryFeedForward) {
    this.indexerMotor.setControl(
        positionRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }

  //there was originally one one configMotor method but i added 2 because it should configure the funnel motor and the indexer motor
  private void configFunnelMotor(int motorID) {

    this.funnelMotor = new TalonFX(motorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = PEAK_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerLimit = CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerTime = PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = PEAK_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    Phoenix6Util.applyAndCheckConfiguration(this.funnelMotor, config, configAlert);

    this.funnelMotor.setPosition(0);

    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", funnelMotor);
  }

  private void configIndexerMotor(int motorID){ //used to be configMotor(int motorID)

    this.indexerMotor = new TalonFX(motorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = PEAK_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerLimit = CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentLowerTime = PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = PEAK_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.MotorOutput.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    Phoenix6Util.applyAndCheckConfiguration(this.indexerMotor, config, configAlert);

    this.indexerMotor.setPosition(0);

    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", indexerMotor);
  }
}
