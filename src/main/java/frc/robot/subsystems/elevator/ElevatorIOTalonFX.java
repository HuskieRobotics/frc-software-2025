package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ElevatorSystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;
import frc.robot.Constants;

public class ElevatorIOTalonFX implements ElevatorIO {

  private TalonFX elevatorMotorLead;
  private TalonFX elevatorMotorFollower;

  private MotionMagicExpoVoltage leadPositionRequest;
  private DynamicMotionMagicVoltage leadPositionRequestDown;
  private VoltageOut leadVoltageRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private StatusSignal<Current> leadStatorCurrent;
  private StatusSignal<Current> followerStatorCurrent;

  private StatusSignal<Voltage> leadVoltageSupplied;
  private StatusSignal<Voltage> followerVoltageSupplied;

  private StatusSignal<Current> leadSupplyCurrent;
  private StatusSignal<Current> followerSupplyCurrent;

  private StatusSignal<Angle> elevatorPositionStatusSignal;

  private StatusSignal<Temperature> elevatorLeadTempStatusSignal;
  private StatusSignal<Temperature> elevatorFollowerTempStatusSignal;

  private StatusSignal<AngularVelocity> elevatorVelocityStatusSignal;

  private final Debouncer connectedLeadDebouncer = new Debouncer(0.5);
  private final Debouncer connectedFollowerDebouncer = new Debouncer(0.5);

  private double localPosition = 0.0;

  // Tunable constants
  private final LoggedTunableNumber kPslot0 =
      new LoggedTunableNumber("Elevator/kPslot0", ElevatorConstants.KP_SLOT0);
  private final LoggedTunableNumber kIslot0 =
      new LoggedTunableNumber("Elevator/kIslot0", ElevatorConstants.KI_SLOT0);
  private final LoggedTunableNumber kDslot0 =
      new LoggedTunableNumber("Elevator/kDslot0", ElevatorConstants.KD_SLOT0);
  private final LoggedTunableNumber kSslot0 =
      new LoggedTunableNumber("Elevator/kSslot0", ElevatorConstants.KS_SLOT0);
  private final LoggedTunableNumber kVslot0 =
      new LoggedTunableNumber("Elevator/kVslot0", ElevatorConstants.KV_SLOT0);
  private final LoggedTunableNumber kAslot0 =
      new LoggedTunableNumber("Elevator/kAslot0", ElevatorConstants.KA_SLOT0);
  private final LoggedTunableNumber kGslot0 =
      new LoggedTunableNumber("Elevator/kGslot0", ElevatorConstants.KG_SLOT0);

  private final LoggedTunableNumber kPslot1 =
      new LoggedTunableNumber("Elevator/kPslot1", ElevatorConstants.KP_SLOT1);
  private final LoggedTunableNumber kIslot1 =
      new LoggedTunableNumber("Elevator/kIslot1", ElevatorConstants.KI_SLOT1);
  private final LoggedTunableNumber kDslot1 =
      new LoggedTunableNumber("Elevator/kDslot1", ElevatorConstants.KD_SLOT1);
  private final LoggedTunableNumber kSslot1 =
      new LoggedTunableNumber("Elevator/kSslot1", ElevatorConstants.KS_SLOT1);
  private final LoggedTunableNumber kVslot1 =
      new LoggedTunableNumber("Elevator/kVslot1", ElevatorConstants.KV_SLOT1);
  private final LoggedTunableNumber kAslot1 =
      new LoggedTunableNumber("Elevator/kAslot1", ElevatorConstants.KA_SLOT1);
  private final LoggedTunableNumber kGslot1 =
      new LoggedTunableNumber("Elevator/kGslot1", ElevatorConstants.KG_SLOT1);

  private final LoggedTunableNumber kPslot2 =
      new LoggedTunableNumber("Elevator/kPslot2", ElevatorConstants.KP_SLOT2);
  private final LoggedTunableNumber kIslot2 =
      new LoggedTunableNumber("Elevator/kIslot2", ElevatorConstants.KI_SLOT2);
  private final LoggedTunableNumber kDslot2 =
      new LoggedTunableNumber("Elevator/kDslot2", ElevatorConstants.KD_SLOT2);
  private final LoggedTunableNumber kSslot2 =
      new LoggedTunableNumber("Elevator/kSslot2", ElevatorConstants.KS_SLOT2);
  private final LoggedTunableNumber kVslot2 =
      new LoggedTunableNumber("Elevator/kVslot2", ElevatorConstants.KV_SLOT2);
  private final LoggedTunableNumber kAslot2 =
      new LoggedTunableNumber("Elevator/kAslot2", ElevatorConstants.KA_SLOT2);
  private final LoggedTunableNumber kGslot2 =
      new LoggedTunableNumber("Elevator/kGslot2", ElevatorConstants.KG_SLOT2);

  private final LoggedTunableNumber kVExpo =
      new LoggedTunableNumber("Elevator/kVExpo", ElevatorConstants.KV_EXPO);
  private final LoggedTunableNumber kAExpo =
      new LoggedTunableNumber("Elevator/kAExpo", ElevatorConstants.KA_EXPO);

  private final LoggedTunableNumber cruiseVelocity =
      new LoggedTunableNumber("Elevator/Cruise Velocity", 0);

  private ElevatorSystemSim elevatorSystemSim;

  public ElevatorIOTalonFX() {

    elevatorMotorLead =
        new TalonFX(ElevatorConstants.LEAD_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    elevatorMotorFollower =
        new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    leadStatorCurrent = elevatorMotorLead.getStatorCurrent();
    followerStatorCurrent = elevatorMotorFollower.getStatorCurrent();

    leadVoltageSupplied = elevatorMotorLead.getMotorVoltage();
    followerVoltageSupplied = elevatorMotorFollower.getMotorVoltage();

    leadSupplyCurrent = elevatorMotorLead.getSupplyCurrent();
    followerSupplyCurrent = elevatorMotorFollower.getSupplyCurrent();

    elevatorPositionStatusSignal = elevatorMotorLead.getPosition();

    elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
    elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();

    elevatorVelocityStatusSignal = elevatorMotorLead.getVelocity();

    Phoenix6Util.registerSignals(
        true,
        leadStatorCurrent,
        followerStatorCurrent,
        leadVoltageSupplied,
        followerVoltageSupplied,
        leadSupplyCurrent,
        followerSupplyCurrent,
        elevatorPositionStatusSignal,
        elevatorLeadTempStatusSignal,
        elevatorFollowerTempStatusSignal,
        elevatorVelocityStatusSignal);

    leadPositionRequest = new MotionMagicExpoVoltage(0);
    leadPositionRequestDown = new DynamicMotionMagicVoltage(0, 10, 100, 500);
    leadVoltageRequest = new VoltageOut(0);

    configElevatorMotorLead(elevatorMotorLead);
    configElevatorMotorFollower(elevatorMotorFollower);

    elevatorMotorFollower.setControl(new Follower(elevatorMotorLead.getDeviceID(), true));

    elevatorSystemSim =
        new ElevatorSystemSim(
            elevatorMotorLead,
            ElevatorConstants.IS_INVERTED,
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.ELEVATOR_MASS_KG,
            Units.inchesToMeters(ElevatorConstants.PULLEY_CIRCUMFERENCE_INCHES / (Math.PI * 2)),
            ElevatorConstants.MIN_HEIGHT.in(Meters),
            ElevatorConstants.MAX_HEIGHT.in(Meters),
            0.0,
            ElevatorConstants.SUBSYSTEM_NAME);
  }

  private void configElevatorMotorLead(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    MotionMagicConfigs leadMotorConfig = config.MotionMagic;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    config.CurrentLimits.SupplyCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = ELEVATOR_PEAK_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.Inverted =
        IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0.kP = kPslot0.get();
    config.Slot0.kI = kIslot0.get();
    config.Slot0.kD = kDslot0.get();
    config.Slot0.kS = kSslot0.get();
    config.Slot0.kV = kVslot0.get();
    config.Slot0.kA = kAslot0.get();
    config.Slot0.kG = kGslot0.get();

    config.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

    config.Slot1.kP = kPslot1.get();
    config.Slot1.kI = kIslot1.get();
    config.Slot1.kD = kDslot1.get();
    config.Slot1.kS = kSslot1.get();
    config.Slot1.kV = kVslot1.get();
    config.Slot1.kA = kAslot1.get();
    config.Slot1.kG = kGslot1.get();

    config.Slot1.withGravityType(GravityTypeValue.Elevator_Static);

    config.Slot2.kP = kPslot2.get();
    config.Slot2.kI = kIslot2.get();
    config.Slot2.kD = kDslot2.get();
    config.Slot2.kS = kSslot2.get();
    config.Slot2.kV = kVslot2.get();
    config.Slot2.kA = kAslot2.get();
    config.Slot2.kG = kGslot2.get();

    config.Slot2.withGravityType(GravityTypeValue.Elevator_Static);

    leadMotorConfig.MotionMagicExpo_kA = kAExpo.get();
    leadMotorConfig.MotionMagicExpo_kV = kVExpo.get();

    leadMotorConfig.MotionMagicCruiseVelocity = cruiseVelocity.get();

    // configure a hardware limit switch that zeros the elevator when lowered; there is no hardware
    // limit switch, but we will set it using a control request
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    config.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0.0;
    config.HardwareLimitSwitch.ReverseLimitEnable = true;

    Phoenix6Util.applyAndCheckConfiguration(elevatorMotorLead, config, configAlert);

    FaultReporter.getInstance()
        .registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Lead", motor);
  }

  public void configElevatorMotorFollower(TalonFX motor) {

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Phoenix6Util.applyAndCheckConfiguration(elevatorMotorFollower, config, configAlert);

    FaultReporter.getInstance()
        .registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Follower", motor);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.connectedLead =
        connectedLeadDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                leadVoltageSupplied,
                leadStatorCurrent,
                leadSupplyCurrent,
                elevatorLeadTempStatusSignal,
                elevatorVelocityStatusSignal,
                elevatorPositionStatusSignal));
    inputs.connectedFollower =
        connectedFollowerDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                followerVoltageSupplied,
                followerStatorCurrent,
                followerSupplyCurrent,
                elevatorFollowerTempStatusSignal));

    inputs.voltageSuppliedLead = leadVoltageSupplied.getValueAsDouble();
    inputs.voltageSuppliedFollower = followerVoltageSupplied.getValueAsDouble();

    inputs.statorCurrentAmpsLead = leadStatorCurrent.getValueAsDouble();
    inputs.statorCurrentAmpsFollower = followerStatorCurrent.getValueAsDouble();

    inputs.supplyCurrentAmpsLead = leadSupplyCurrent.getValueAsDouble();
    inputs.supplyCurrentAmpsFollower = followerSupplyCurrent.getValueAsDouble();

    inputs.leadTempCelsius = elevatorLeadTempStatusSignal.getValueAsDouble();
    inputs.followerTempCelsius = elevatorFollowerTempStatusSignal.getValueAsDouble();

    inputs.velocityRPS = elevatorVelocityStatusSignal.getValueAsDouble();

    // Getting the signal from the TalonFX is more time consuming than having the signal already
    // available. However, if we attempt to get the signal earlier, it won't be bound to the correct
    // control type. So, we only take the hit when tuning which is when this information is needed
    // more.
    if (Constants.TUNING_MODE) {
      inputs.closedLoopError = elevatorMotorLead.getClosedLoopError().getValueAsDouble();
      inputs.closedLoopReference = elevatorMotorLead.getClosedLoopReference().getValueAsDouble();
    }

    inputs.positionRotations = elevatorPositionStatusSignal.getValueAsDouble();

    inputs.positionInches = inputs.positionRotations * PULLEY_CIRCUMFERENCE_INCHES;

    localPosition = inputs.positionInches;

    LoggedTunableNumber.ifChanged(
        hashCode(),
        motionMagic -> {
          TalonFXConfiguration config = new TalonFXConfiguration();
          this.elevatorMotorLead.getConfigurator().refresh(config);
          config.Slot0.kP = motionMagic[0];
          config.Slot0.kI = motionMagic[1];
          config.Slot0.kD = motionMagic[2];
          config.Slot0.kS = motionMagic[3];
          config.Slot0.kV = motionMagic[4];
          config.Slot0.kA = motionMagic[5];
          config.Slot0.kG = motionMagic[6];

          config.Slot1.kP = motionMagic[7];
          config.Slot1.kI = motionMagic[8];
          config.Slot1.kD = motionMagic[9];
          config.Slot1.kS = motionMagic[10];
          config.Slot1.kV = motionMagic[11];
          config.Slot1.kA = motionMagic[12];
          config.Slot1.kG = motionMagic[13];

          config.Slot2.kP = motionMagic[14];
          config.Slot2.kI = motionMagic[15];
          config.Slot2.kD = motionMagic[16];
          config.Slot2.kS = motionMagic[17];
          config.Slot2.kV = motionMagic[18];
          config.Slot2.kA = motionMagic[19];
          config.Slot2.kG = motionMagic[20];

          config.MotionMagic.MotionMagicExpo_kV = motionMagic[21];
          config.MotionMagic.MotionMagicExpo_kA = motionMagic[22];

          config.MotionMagic.MotionMagicCruiseVelocity = motionMagic[23];

          this.elevatorMotorLead.getConfigurator().apply(config);
        },
        kPslot0,
        kIslot0,
        kDslot0,
        kSslot0,
        kVslot0,
        kAslot0,
        kGslot0,
        kPslot1,
        kIslot1,
        kDslot1,
        kSslot1,
        kVslot1,
        kAslot1,
        kGslot1,
        kPslot2,
        kIslot2,
        kDslot2,
        kSslot2,
        kVslot2,
        kAslot2,
        kGslot2,
        kVExpo,
        kAExpo,
        cruiseVelocity);

    elevatorSystemSim.updateSim();
  }

  // Set motor voltage
  @Override
  public void setMotorVoltage(double voltage) {
    elevatorMotorLead.setControl(
        leadVoltageRequest.withLimitReverseMotion(false).withOutput(voltage));
  }

  @Override
  public void zeroPosition() {
    // we set the reverse limit instead of directly setting the position to avoid the overhead of a
    // config call
    elevatorMotorLead.setControl(leadVoltageRequest.withLimitReverseMotion(true).withOutput(0.0));
  }

  @Override
  public void setPosition(Distance position) {
    if (localPosition < position.in(Inches)) {

      // set the elevator to slot 0
      elevatorMotorLead.setControl(
          leadPositionRequest
              .withPosition(position.in(Inches) / PULLEY_CIRCUMFERENCE_INCHES)
              .withSlot(0));
    } else {
      elevatorMotorLead.setControl(
          leadPositionRequestDown
              .withPosition(position.in(Inches) / PULLEY_CIRCUMFERENCE_INCHES)
              .withSlot(0));
    }
  }
}
