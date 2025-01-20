package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.sim.VelocitySystemSim;
import frc.lib.team6328.util.LoggedTunableNumber;

public class IntakeIOTalonFX implements IntakeIO {
    private TalonFX pivotMotor;
    private TalonFX rollerMotor;

    private Alert configAlert = 
        new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

    private VelocityTorqueCurrentFOC rollerVelocityRequest;
    private VelocityTorqueCurrentFOC pivotPositionRequest;
    private VoltageOut rollerVoltageRequest;  
    private VoltageOut pivotVoltageRequest;
// roller status signals
    private StatusSignal<Voltage> rollerMotorVoltageStatusSignal;
    private StatusSignal<AngularVelocity> rollerMotorVelocityStatusSignal;
    private StatusSignal<Current> rollerStatorCurrentStatusSignal;
    private StatusSignal<Current> rollerSupplyCurrentStatusSignal;
    private StatusSignal<Temperature> rollerTempCelciusStatusSignal;
    private StatusSignal<Double> rollerClosedLoopErrorStatusSignal;
    private StatusSignal<Double> rollerReferenceVelocityStatusSignal;

// pivot status signals
    private StatusSignal<AngularVelocity> pivotMotorVelocityStatusSignal;
    private StatusSignal<Voltage> pivotMotorVoltageStatusSignal;
    private StatusSignal<Current> pivotStatorCurrentStatusSignal;
    private StatusSignal<Temperature> pivotTempCelciusStatusSignal;
    private StatusSignal<Double> pivotClosedLoopErrorStatusSignal;
    private StatusSignal<Double> pivotReferencePositionStatusSignal;
    private StatusSignal<Current> pivotSupplyStatusSignal;
    private StatusSignal<Double> pivotPositionDegStatusSignal;

    private VelocitySystemSim rollerSim;
    private VelocitySystemSim pivotSim;

    private final LoggedTunableNumber rollerMotorsKP =
        new LoggedTunableNumber("Intake/rollerMotorsKP", 0.0);

    private final LoggedTunableNumber rollerMotorKS = 
        new LoggedTunableNumber("Intake/rollerMotorsKS", 0.0);

    private final LoggedTunableNumber rollerMotorKV =
        new LoggedTunableNumber("Intake/rollerMotorsKV", 0.0);

    private final LoggedTunableNumber pivotMotorsKP =
        new LoggedTunableNumber("Intake/pivotMotorsKP", 0.0);

    private final LoggedTunableNumber pivotMotorsKI =
        new LoggedTunableNumber("Intake/pivotMotorsKI", 0.0);

    private final LoggedTunableNumber pivotMotorsKD =
        new LoggedTunableNumber("Intake/pivotMotorsKD", 0.0);

    private final LoggedTunableNumber pivotMotorsKS =
        new LoggedTunableNumber("Intake/pivotMotorsKS", 0.0);

    private final LoggedTunableNumber pivotMotorsKV =
        new LoggedTunableNumber("Intake/pivotMotorsKV", 0.0);

    private final LoggedTunableNumber pivotMotorsKA =
        new LoggedTunableNumber("Intake/pivotMotorsKA", 0.0);

    private final LoggedTunableNumber pivotMotorsKG =
        new LoggedTunableNumber("Intake/pivotMotorsKG", 0.0);

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
        pivotMotorVelocityStatusSignal = pivotMotor.getVelocity();

        rollerStatorCurrentStatusSignal = rollerMotor.getStatorCurrent();
        pivotStatorCurrentStatusSignal = pivotMotor.getStatorCurrent();

        rollerSupplyCurrentStatusSignal = rollerMotor.getSupplyCurrent();
        pivotSupplyStatusSignal = pivotMotor.getSupplyCurrent();

        rollerReferenceVelocityStatusSignal = rollerMotor.getClosedLoopReference();
        pivotReferencePositionStatusSignal = pivotMotor.getClosedLoopReference();

        rollerTempCelciusStatusSignal = rollerMotor.getDeviceTemp();
        pivotTempCelciusStatusSignal = pivotMotor.getDeviceTemp();

        rollerMotorVoltageStatusSignal = rollerMotor.getMotorVoltage();
        pivotMotorVoltageStatusSignal = pivotMotor.getMotorVoltage();

        rollerClosedLoopErrorStatusSignal = rollerMotor.getClosedLoopError();
        pivotClosedLoopErrorStatusSignal = pivotMotor.getClosedLoopError();
        
        this.rollerSim = 
            new VelocitySystemSim(
                rollerMotor,
                0,
                0,
                0,
                0);

                this.pivotSim = 
                new VelocitySystemSim(
                    rollerMotor,
                    0,
                    0,
                    0,
                    0);
    }

    @Override
    public void updateInputs(IntakeIOINputs inputs) {
        this.rollerSim.updateSim();
        this.pivotSim.updateSim();

        BaseStatusSignal.refreshAll(
            rollerMotorVelocityStatusSignal,
            rollerStatorCurrentStatusSignal,
            rollerSupplyCurrentStatusSignal,
            rollerReferenceVelocityStatusSignal,
            rollerClosedLoopErrorStatusSignal,
            pivotClosedLoopErrorStatusSignal,
            pivotSupplyStatusSignal,
            pivotStatorCurrentStatusSignal,
            pivotMotorVelocityStatusSignal,
            pivotReferencePositionStatusSignal,
            rollerTempCelciusStatusSignal,
            pivotTempCelciusStatusSignal,
            rollerMotorVoltageStatusSignal,
            pivotMotorVoltageStatusSignal,
            pivotPositionDegStatusSignal);
            
        inputs.rollerStatorCurrentAmps = rollerStatorCurrentStatusSignal.getValueAsDouble();
        inputs.pivotStatorCurrentAmps = pivotStatorCurrentStatusSignal.getValueAsDouble();

        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentStatusSignal.getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotSupplyStatusSignal.getValueAsDouble();

        inputs.rollerReferenceVelocityRPS = rollerReferenceVelocityStatusSignal.getValueAsDouble();
        inputs.pivotReferencePositionDeg = pivotReferencePositionStatusSignal.getValueAsDouble();

        
        inputs.rollerTempCelcius = rollerTempCelciusStatusSignal.getValueAsDouble();
        inputs.pivotTempCelcius = pivotTempCelciusStatusSignal.getValueAsDouble();

        inputs.rollerMotorVoltage = rollerMotorVoltageStatusSignal.getValueAsDouble();
        inputs.pivotMotorVoltage = pivotMotorVoltageStatusSignal.getValueAsDouble();

        if (rollerMotorsKP.hasChanged(0)
            || rollerMotorKS.hasChanged(0)
            || rollerMotorKV.hasChanged(0)) {
                
                Slot0Configs slot0Configs = new Slot0Configs();
                rollerMotor.getConfigurator().refresh(slot0Configs);
                slot0Configs.kP = rollerMotorsKP.get();
                slot0Configs.kS = rollerMotorKS.get();
                slot0Configs.kV = rollerMotorKV.get();

                rollerMotor.getConfigurator().apply(slot0Configs);
            }

        if (pivotMotorsKP.hasChanged(0)
            || pivotMotorsKI.hasChanged(0)
            || pivotMotorsKD.hasChanged(0)
            || pivotMotorsKS.hasChanged(0)) {
                
                Slot0Configs slot0Configs = new Slot0Configs();
                pivotMotor.getConfigurator().refresh(slot0Configs);
                slot0Configs.kP = pivotMotorsKP.get();
                slot0Configs.kI = pivotMotorsKI.get();
                slot0Configs.kD = pivotMotorsKD.get();
                slot0Configs.kS = pivotMotorsKS.get();

                pivotMotor.getConfigurator().apply(slot0Configs);
            }
        
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

        rollerTorqueCurrentConfigs.PeakForwardTorqueCurrent = 
            0.0;
        rollerTorqueCurrentConfigs.PeakReverseTorqueCurrent =
            0.0;

        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        rollerConfig.Slot0.kP = rollerMotorsKP.get();
        rollerConfig.Slot0.kS = rollerMotorKS.get();
        rollerConfig.Slot0.kV = rollerMotorKV.get();

        
        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++){
            status = rollerMotor.getConfigurator().apply(rollerConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()){
            configAlert.set(true);
            configAlert.setText(status.toString());
        }

        FaultReporter.getInstance().registerHardware("INTAKE", "IntakeRoller", rollerMotor);
    }

    private void configureIntakePivotMotor(TalonFX motor) {
        TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
        TorqueCurrentConfigs pivotTorqueCurrentConfigs = new TorqueCurrentConfigs();

        pivotTorqueCurrentConfigs.PeakForwardTorqueCurrent = 
            0.0;
        pivotTorqueCurrentConfigs.PeakReverseTorqueCurrent =
            0.0;

        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        pivotConfig.Slot0.kP = pivotMotorsKP.get();
        pivotConfig.Slot0.kI = pivotMotorsKI.get();
        pivotConfig.Slot0.kD = pivotMotorsKD.get();
        pivotConfig.Slot0.kS = pivotMotorsKS.get();
        pivotConfig.Slot0.kV = pivotMotorsKV.get();
        pivotConfig.Slot0.kA = pivotMotorsKA.get();
        pivotConfig.Slot0.kG = pivotMotorsKG.get();

        
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++){
            status = pivotMotor.getConfigurator().apply(pivotConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()){
            configAlert.set(true);
            configAlert.setText(status.toString());
        }

        FaultReporter.getInstance().registerHardware("INTAKE", "IntakePivot", pivotMotor);
    }
}
