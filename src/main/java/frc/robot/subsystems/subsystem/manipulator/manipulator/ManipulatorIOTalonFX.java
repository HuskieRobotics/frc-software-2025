package frc.robot.subsystems.subsystem.manipulator.manipulator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.subsystem.manipulator.manipulator.ManipulatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput; //imported this class for the sensors
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team6328.util.LoggedTunableNumber;
import com.ctre.phoenix6.signals.InvertedValue; //imported to invert the funnel motor

/** TalonFX implementation of the generic SubsystemIO */
public class ManipulatorIOTalonFX implements ManipulatorIO {
  private TalonFX funnelMotor;
  private TalonFX indexerMotor;

  //ir sensors
  private DigitalInput funnelIRSensor;
  private DigitalInput indexerIRSensor;
  
  private TorqueCurrentFOC voltageRequest;
  private VelocityTorqueCurrentFOC velocityRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

/* You will create StatusSignal<> objects for each logged input. 
  These status signals will then be insantiated in the constructor with what value they should be tracking, which you do in updateInputs right now. 
  In updateInputs, you will refresh all of the status signals, 
  and set each input variable by retrieving each of the values of your statussignals with the statussignal.getValueAs___() methods.
*/
  private StatusSignal<LoggedTunableNumber> kP;
  private StatusSignal<LoggedTunableNumber> kI;
  private StatusSignal<LoggedTunableNumber> kD;
  private StatusSignal<LoggedTunableNumber> kPeakOuput;

  //private final LoggedTunableNumber kP = new LoggedTunableNumber("Subsystem/kP", POSITION_PID_P);
  //private final LoggedTunableNumber kI = new LoggedTunableNumber("Subsystem/kI", POSITION_PID_I);
  //private final LoggedTunableNumber kD = new LoggedTunableNumber("Subsystem/kD", POSITION_PID_D);
  //private final LoggedTunableNumber kPeakOutput = new LoggedTunableNumber("Subsystem/kPeakOutput", POSITION_PID_PEAK_OUTPUT);

  // Create StatusSignal objects for each loggable input from the ManipulatorIO class in the updateInputs method
  private StatusSignal<Double> positionDeg;
  private StatusSignal<Double> velocityRPM;
  private StatusSignal<Double> closedLoopError;
  private StatusSignal<Double> setpoint;
  private StatusSignal<Double> power;
  private StatusSignal<String> controlMode;
  private StatusSignal<Double> statorCurrentAmps;
  private StatusSignal<Double> tempCelsius;
  private StatusSignal<Double> supplyCurrentAmps;

  /** Create a TalonFX-specific generic SubsystemIO */
  public ManipulatorIOTalonFX() {
    kP = new LoggedTunableNumber("Subsystem/kP", POSITION_PID_P);
    kI = new LoggedTunableNumber("Subsystem/kI", POSITION_PID_I);
    kD = new LoggedTunableNumber("Subsystem/kD", POSITION_PID_D);
    kPeakOuput = new LoggedTunableNumber("Subsystem/kPeakOutput", POSITION_PID_PEAK_OUTPUT);
    configFunnelMotor(12);
    configIndexerMotor(14);
    //the can id's for the funnel and indexer motor is in the ManipulatorConstants.java file
    this.funnelMotor.setPosition(0);
    this.voltageRequest = new VoltageOut(0.0);
  }

  /**
   * Update the inputs based on the current state of the TalonFX motor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ManipulatorIOInputs inputs) { 

    //This is part of the update inputs method for the funnel.
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

    //I added this part here to assign all the variables in the ManipulatorIO class keeping track of the logged tunable inputs to the acctual value of the logged tunable inputs.
    funnelMotorStatorCurrentAmps = inputs.statorCurrentAmps;
    funnelMotorSupplyCurrentAmps = inputs.supplyCurrentAmps;
    //funnelMotorVelocityRPS --> this value is not currently a logged input 
    //funnelMotorReferenceVelocityRPS --> this value is not currently a logged input
    funnelMotorTempCelsius = inputs.tempCelsuis;
    //funnelMotorVoltage --> not sure if this connects to a logged tunable input

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
        
        //This is the part of the update inputs method for the indexer motor.
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

    //Assign all variables for the indexer motor logged inputs in the ManipulatorIO class keeping track of the logged tunable inputs to the acctual value of the logged tunable inputs for the indexer motor
    indexerMotorStatorCurrentAmps 
    indexerMotorSupplyCurrentAmps 
    indexerMotorVelocityRPS
    indexerMotorReferenceVelocityRPS
    indexerMotorTempCelsius
    indexerMotorVoltage
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

  /**
   * Get the state of the IR sensor near the funnel motor.
   * 
   * @return true if the ir sensor near the funnel is blocked
   */
  @Override
  public boolean getFunnelIRState() {
    return funnelIRSensor.get();
  } 

  /**
   * Get the state of the IR sensor near the indexer motor.
   * 
   * @return true if the ir sensor near the indexer is blocked
   */
  @Override
  public boolean getIndexerIRState() {
    return indexerIRSensor.get();
  }

  /**
   * This method configures the Funnel motor with the motor ID.
   * @param motorID The ID of the motor to configure.
   */
  private void configFunnelMotor(int motorID) {

    this.funnelMotor = new TalonFX(motorID, RobotConfig.getInstance().getCANBusName());

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
        FUNNEL_MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

    Phoenix6Util.applyAndCheckConfiguration(this.funnelMotor, config, configAlert);

    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", funnelMotor);
  }

  /**
   * This method configures the Indexer motor with the motor ID.
   * @param motorID The ID of the motor to configure.
   */
  private void configIndexerMotor(int motorID){ 

    this.indexerMotor = new TalonFX(motorID, RobotConfig.getInstance().getCANBusName());

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

    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "subsystem motor", indexerMotor);
  }
}
