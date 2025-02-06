package frc.robot.subsystems.intake;

import static frc.robot.subsystems.subsystem.SubsystemConstants.SUBSYSTEM_NAME;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
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
import frc.lib.team254.Phoenix6Util;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.ArmSystemSim;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX pivotMotor;
  private TalonFX rollerMotor;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

  private VelocityTorqueCurrentFOC rollerVelocityRequest;
  private VelocityTorqueCurrentFOC pivotPositionRequest;
  private VoltageOut pivotVoltageRequest;
  private VoltageOut rollerVoltageRequest;

  // roller status signals
  private StatusSignal<Voltage> rollerMotorVoltageStatusSignal;
  private StatusSignal<AngularVelocity> rollerMotorVelocityStatusSignal;
  private StatusSignal<Current> rollerStatorCurrentStatusSignal;
  private StatusSignal<Current> rollerSupplyCurrentStatusSignal;
  private StatusSignal<Temperature> rollerTempCelciusStatusSignal;
//  private StatusSignal<Double> rollerClosedLoopErrorStatusSignal;
 // private StatusSignal<Double> rollerReferenceVelocityStatusSignal;

  // pivot status signals
  private StatusSignal<Voltage> pivotMotorVoltageStatusSignal;
  private StatusSignal<Current> pivotStatorCurrentStatusSignal;
  private StatusSignal<Temperature> pivotTempCelciusStatusSignal;
 // private StatusSignal<Double> pivotClosedLoopErrorStatusSignal;
 // private StatusSignal<Double> pivotReferencePositionStatusSignal;
  private StatusSignal<Current> pivotSupplyStatusSignal;
  private StatusSignal<Angle> pivotPositionDegStatusSignal;

  private VelocitySystemSim rollerSim;
  private ArmSystemSim pivotSim;

  private final LoggedTunableNumber rollerMotorsKP =
      new LoggedTunableNumber("Intake/rollerMotorsKP",INTAKE_ROLLER_MOTORS_KP);

  private final LoggedTunableNumber rollerMotorKS =
      new LoggedTunableNumber("Intake/rollerMotorsKS", INTAKE_ROLLER_MOTORS_KS);

  private final LoggedTunableNumber rollerMotorKV =
      new LoggedTunableNumber("Intake/rollerMotorsKV", INTAKE_ROLLER_MOTORS_KV);

  private final LoggedTunableNumber pivotMotorsKP =
      new LoggedTunableNumber("Intake/pivotMotorsKP", INTAKE_PIVOT_MOTORS_KP);

  private final LoggedTunableNumber pivotMotorsKI =
      new LoggedTunableNumber("Intake/pivotMotorsKI", INTAKE_PIVOT_MOTORS_KI);

  private final LoggedTunableNumber pivotMotorsKD =
      new LoggedTunableNumber("Intake/pivotMotorsKD", INTAKE_PIVOT_MOTORS_KD);

  private final LoggedTunableNumber pivotMotorsKS =
      new LoggedTunableNumber("Intake/pivotMotorsKS", INTAKE_PIVOT_MOTORS_KS);

  private final LoggedTunableNumber pivotMotorsKV =
      new LoggedTunableNumber("Intake/pivotMotorsKV", INTAKE_PIVOT_MOTORS_KV);

  private final LoggedTunableNumber pivotMotorsKA =
      new LoggedTunableNumber("Intake/pivotMotorsKA", INTAKE_PIVOT_MOTORS_KA);

  private final LoggedTunableNumber pivotMotorsKG =
      new LoggedTunableNumber("Intake/pivotMotorsKG", INTAKE_PIVOT_MOTORS_KG);

  public IntakeIOTalonFX() {
    rollerMotor =
        new TalonFX(
            IntakeConstants.INTAKE_ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    pivotMotor =
        new TalonFX(
            IntakeConstants.INTAKE_PIVOT_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    configureIntakeRollerMotor(rollerMotor);
    configureIntakePivotMotor(pivotMotor);

    rollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    rollerVoltageRequest = new VoltageOut(0);
    pivotPositionRequest = new VelocityTorqueCurrentFOC(0);
    pivotVoltageRequest = new VoltageOut(0);

    rollerMotorVelocityStatusSignal = rollerMotor.getVelocity();

    rollerStatorCurrentStatusSignal = rollerMotor.getStatorCurrent();
    pivotStatorCurrentStatusSignal = pivotMotor.getStatorCurrent();

    rollerSupplyCurrentStatusSignal = rollerMotor.getSupplyCurrent();
    pivotSupplyStatusSignal = pivotMotor.getSupplyCurrent();

    

    rollerTempCelciusStatusSignal = rollerMotor.getDeviceTemp();
    pivotTempCelciusStatusSignal = pivotMotor.getDeviceTemp();

    rollerMotorVoltageStatusSignal = rollerMotor.getMotorVoltage();
    pivotMotorVoltageStatusSignal = pivotMotor.getMotorVoltage();
    pivotPositionDegStatusSignal = pivotMotor.getPosition();

   

    this.rollerSim = new VelocitySystemSim(rollerMotor, INTAKE_ROLLER_MOTORS_INVERTED, INTAKE_ROLLER_MOTORS_KV, 0, ROLLER_GEAR_RATIO);

    this.pivotSim = new ArmSystemSim(pivotMotor, INTAKE_PIVOT_MOTORS_INVERTED, PIVOT_GEAR_RATIO, PIVOT_BELT_RATIO, PIVOT_SIM_LENGTH, PIVOT_SIM_MASS, PIVOT_SIM_MIN_ANGLE, PIVOT_SIM_MAX_ANGLE, PIVOT_SIM_STARTING_ANGLE, SUBSYSTEM_NAME); //
  }
 
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    this.rollerSim.updateSim();
    this.pivotSim.updateSim();

    BaseStatusSignal.refreshAll(
        rollerMotorVelocityStatusSignal,
        rollerStatorCurrentStatusSignal,
        rollerSupplyCurrentStatusSignal,
       // rollerReferenceVelocityStatusSignal,
       // rollerClosedLoopErrorStatusSignal,
       // pivotClosedLoopErrorStatusSignal,
        pivotSupplyStatusSignal,
        pivotStatorCurrentStatusSignal,
      //  pivotReferencePositionStatusSignal,
        rollerTempCelciusStatusSignal,
        pivotTempCelciusStatusSignal,
        rollerMotorVoltageStatusSignal,
        pivotMotorVoltageStatusSignal,
        pivotPositionDegStatusSignal);

    inputs.rollerStatorCurrentAmps = rollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.pivotStatorCurrentAmps = pivotStatorCurrentStatusSignal.getValueAsDouble();

    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.pivotSupplyCurrentAmps = pivotSupplyStatusSignal.getValueAsDouble();

    inputs.rollerReferenceVelocityRPS = rollerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.pivotReferencePositionDeg = pivotMotor.getClosedLoopReference().getValueAsDouble();

    inputs.rollerTempCelcius = rollerTempCelciusStatusSignal.getValueAsDouble();
    inputs.pivotTempCelcius = pivotTempCelciusStatusSignal.getValueAsDouble();

    inputs.rollerMotorVelocityRPS = rollerMotorVelocityStatusSignal.getValueAsDouble();

    inputs.rollerMotorVoltage = rollerMotorVoltageStatusSignal.getValueAsDouble();
    inputs.pivotMotorVoltage = pivotMotorVoltageStatusSignal.getValueAsDouble();

    inputs.rollerClosedLoopError = rollerMotor.getClosedLoopError().getValueAsDouble();
    inputs.pivotClosedLoopError = pivotMotor.getClosedLoopError().getValueAsDouble();
    inputs.pivotPositionDeg = pivotPositionDegStatusSignal.getValueAsDouble();

    LoggedTunableNumber.ifChanged(hashCode(),
    rollerpidconstants -> {
      TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
      this.rollerMotor.getConfigurator().refresh(rollerConfig);
      rollerConfig.Slot0.kP = rollerpidconstants[0];
      rollerConfig.Slot0.kS = rollerpidconstants[1];
      rollerConfig.Slot0.kV = rollerpidconstants[2];
      this.rollerMotor.getConfigurator().apply(rollerConfig);
    },
    rollerMotorsKP, rollerMotorKS, rollerMotorKV);

   
   LoggedTunableNumber.ifChanged(hashCode(),
    pivotpidconstants -> {
      TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
      this.pivotMotor.getConfigurator().refresh(pivotConfig);
      pivotConfig.Slot0.kP = pivotpidconstants[0];
      pivotConfig.Slot0.kI = pivotpidconstants[1];
      pivotConfig.Slot0.kD = pivotpidconstants[2];
      pivotConfig.Slot0.kS = pivotpidconstants[3];
      pivotConfig.Slot0.kV = pivotpidconstants[4];
      pivotConfig.Slot0.kA = pivotpidconstants[5];
      pivotConfig.Slot0.kG = pivotpidconstants[6];
      this.pivotMotor.getConfigurator().apply(pivotConfig);
    },
    pivotMotorsKP, pivotMotorsKI, pivotMotorsKD, pivotMotorsKS, pivotMotorsKV, pivotMotorsKA, pivotMotorsKG);

    rollerSim.updateSim();
    pivotSim.updateSim();
  }

  @Override
  public void setPivotRotationPosition(double position) {
    pivotMotor.setControl(pivotPositionRequest.withAcceleration(position));
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setRollerMotorVoltage(double voltage) {
    rollerMotor.setControl(rollerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setRollerMotorVelocity(double rps) {
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(rps));
  }

  private void configureIntakeRollerMotor(TalonFX motor) {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs rollerTorqueCurrentConfigs = new TorqueCurrentConfigs();

    rollerTorqueCurrentConfigs.PeakForwardTorqueCurrent = 0.0;
    rollerTorqueCurrentConfigs.PeakReverseTorqueCurrent = 0.0;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    rollerConfig.MotorOutput.Inverted = INTAKE_PIVOT_MOTORS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    rollerConfig.Slot0.kP = rollerMotorsKP.get();
    rollerConfig.Slot0.kS = rollerMotorKS.get();
    rollerConfig.Slot0.kV = rollerMotorKV.get();

    Phoenix6Util.applyAndCheckConfiguration(rollerMotor, rollerConfig, configAlert);
    
    FaultReporter.getInstance().registerHardware("INTAKE", "IntakeRoller", rollerMotor);
  }

  private void configureIntakePivotMotor(TalonFX motor) {
    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs pivotTorqueCurrentConfigs = new TorqueCurrentConfigs();

    pivotTorqueCurrentConfigs.PeakForwardTorqueCurrent = 0.0;
    pivotTorqueCurrentConfigs.PeakReverseTorqueCurrent = 0.0;

    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    pivotConfig.Slot0.kP = pivotMotorsKP.get();
    pivotConfig.Slot0.kI = pivotMotorsKI.get();
    pivotConfig.Slot0.kD = pivotMotorsKD.get();
    pivotConfig.Slot0.kS = pivotMotorsKS.get();
    pivotConfig.Slot0.kV = pivotMotorsKV.get();
    pivotConfig.Slot0.kA = pivotMotorsKA.get();
    pivotConfig.Slot0.kG = pivotMotorsKG.get();

    pivotConfig.MotorOutput.Inverted =
        IntakeConstants.INTAKE_PIVOT_MOTORS_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

            
    Phoenix6Util.applyAndCheckConfiguration(pivotMotor, pivotConfig, configAlert);

    FaultReporter.getInstance().registerHardware("INTAKE", "IntakePivot", pivotMotor);
  }
}
