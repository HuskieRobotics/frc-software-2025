package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.LoggedTunableNumber;

public class ElevatorIOTalonFX implements ElevatorIO {

    private TalonFX elevatorMotorLead;
    private TalonFX elevatorMotorFollower;
    
    private MotionMagicExpoVoltage elevatorMotorLeadExpoVoltageRequest;
    
    private TalonFXConfiguration elevatorMotorLeadConfig;
    private TalonFXConfiguration elevatorMotorFollowerConfig;
    
    
    private Alert configAlert = 
        new Alert("Failed to apply configuration for subsystem.", AlertType.kError);
    
    
    private StatusSignal<Current> elevatorLeadStatorCurrentStatusSignal;
    private StatusSignal<Current> elevatorFollowerStatorCurrentStatusSignal;
    
    private StatusSignal<Current> elevatorLeadSupplyCurrentStatusSignal; 
    
    private StatusSignal<Angle> elevatorPositionStatusSignal; // Does this need to be of type Angle?
        
    private StatusSignal<Temperature> elevatorLeadTempStatusSignal;
    private StatusSignal<Temperature> elevatorFollowerTempStatusSignal;

        // Tunable constants
    private final LoggedTunableNumber kPslot0 = new LoggedTunableNumber("Elevator/kPslot0", ElevatorConstants.KP_SLOT0);
    private final LoggedTunableNumber kIslot0 = new LoggedTunableNumber("Elevator/kIslot0", ElevatorConstants.KI_SLOT0);
    private final LoggedTunableNumber kDslot0 = new LoggedTunableNumber("Elevator/kDslot0", ElevatorConstants.KD_SLOT0);
    private final LoggedTunableNumber kSslot0 = new LoggedTunableNumber("Elevator/kSslot0", ElevatorConstants.KS_SLOT0);
    private final LoggedTunableNumber kVslot0 = new LoggedTunableNumber("Elevator/kVslot0", ElevatorConstants.KV_SLOT0);
    private final LoggedTunableNumber kAslot0 = new LoggedTunableNumber("Elevator/kAslot0", ElevatorConstants.KA_SLOT0);
    private final LoggedTunableNumber kVExposlot0 = new LoggedTunableNumber("Elevator/kVExposlot0", ElevatorConstants.KV_EXPO_SLOT0);
    private final LoggedTunableNumber kAExposlot0 = new LoggedTunableNumber("Elevator/kAExposlot0", ElevatorConstants.KA_EXPO_SLOT0);
    private final LoggedTunableNumber kGslot0 = new LoggedTunableNumber("Elevator/kGslot0", ElevatorConstants.KG_SLOT0);

    private final LoggedTunableNumber kPslot1 = new LoggedTunableNumber("Elevator/kPslot1", ElevatorConstants.KP_SLOT1);
    private final LoggedTunableNumber kIslot1 = new LoggedTunableNumber("Elevator/kIslot1", ElevatorConstants.KI_SLOT1);
    private final LoggedTunableNumber kDslot1 = new LoggedTunableNumber("Elevator/kDslot1", ElevatorConstants.KD_SLOT1);
    private final LoggedTunableNumber kSslot1 = new LoggedTunableNumber("Elevator/kSslot1", ElevatorConstants.KS_SLOT1);
    private final LoggedTunableNumber kVslot1 = new LoggedTunableNumber("Elevator/kVslot1", ElevatorConstants.KV_SLOT1);
    private final LoggedTunableNumber kAslot1 = new LoggedTunableNumber("Elevator/kAslot1", ElevatorConstants.KA_SLOT1);
    private final LoggedTunableNumber kVExposlot1 = new LoggedTunableNumber("Elevator/kVExposlot1", ElevatorConstants.KV_EXPO_SLOT1);
    private final LoggedTunableNumber kAExposlot1 = new LoggedTunableNumber("Elevator/kAExposlot1", ElevatorConstants.KA_EXPO_SLOT1);
    private final LoggedTunableNumber kGslot1 = new LoggedTunableNumber("Elevator/kGslot1", ElevatorConstants.KG_SLOT1);

    private final LoggedTunableNumber kPslot2 = new LoggedTunableNumber("Elevator/kPslot2", ElevatorConstants.KP_SLOT2);
    private final LoggedTunableNumber kIslot2 = new LoggedTunableNumber("Elevator/kIslot2", ElevatorConstants.KI_SLOT2);
    private final LoggedTunableNumber kDslot2 = new LoggedTunableNumber("Elevator/kDslot2", ElevatorConstants.KD_SLOT2);
    private final LoggedTunableNumber kSslot2 = new LoggedTunableNumber("Elevator/kSslot2", ElevatorConstants.KS_SLOT2);
    private final LoggedTunableNumber kVslot2 = new LoggedTunableNumber("Elevator/kVslot2", ElevatorConstants.KV_SLOT2);
    private final LoggedTunableNumber kAslot2 = new LoggedTunableNumber("Elevator/kAslot2", ElevatorConstants.KA_SLOT2);
    private final LoggedTunableNumber kVExposlot2 = new LoggedTunableNumber("Elevator/kVExposlot2", ElevatorConstants.KV_EXPO_SLOT2);
    private final LoggedTunableNumber kAExposlot2 = new LoggedTunableNumber("Elevator/kAExposlot2", ElevatorConstants.KA_EXPO_SLOT2);
    private final LoggedTunableNumber kGslot2 = new LoggedTunableNumber("Elevator/kGslot2", ElevatorConstants.KG_SLOT2);

    private final LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Elevator/Cruise Velocity", 0);
    private final LoggedTunableNumber acceleration = new LoggedTunableNumber("Elevator/Acceleration", 0);
    private final LoggedTunableNumber jerk = new LoggedTunableNumber("Elevator/Jerk", 0);

    
    
    public ElevatorIOTalonFX() {

        elevatorMotorLead = new TalonFX(ElevatorConstants.LEAD_MOTOR_ID);
        elevatorMotorFollower = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);

        elevatorLeadStatorCurrentStatusSignal = elevatorMotorLead.getStatorCurrent();
        elevatorFollowerStatorCurrentStatusSignal = elevatorMotorFollower.getStatorCurrent();

        elevatorLeadSupplyCurrentStatusSignal = elevatorMotorLead.getSupplyCurrent();

        elevatorPositionStatusSignal = elevatorMotorLead.getPosition();

        elevatorLeadTempStatusSignal = elevatorMotorLead.getDeviceTemp();
        elevatorFollowerTempStatusSignal = elevatorMotorFollower.getDeviceTemp();

        elevatorMotorLeadExpoVoltageRequest = new MotionMagicExpoVoltage(0);

        configElevatorMotorLead(elevatorMotorLead);
        configElevatorMotorFollower(elevatorMotorFollower);
    }

    private void configElevatorMotorLead(TalonFX motor){

        elevatorMotorLeadConfig = new TalonFXConfiguration();

        MotionMagicConfigs leadMotorConfig  = elevatorMotorLeadConfig.MotionMagic;

        SoftwareLimitSwitchConfigs softLimitConfigs = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(ElevatorConstants.FORWARD_SOFT_LIMIT_THRESHOLD) 
            .withReverseSoftLimitEnable(true) 
            .withReverseSoftLimitThreshold(ElevatorConstants.REVERSE_SOFT_LIMIT_THRESHOLD);

        elevatorMotorLeadConfig.SoftwareLimitSwitch = softLimitConfigs;

        elevatorMotorLeadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        elevatorMotorLeadConfig.Slot0.kP = kPslot0.get();
        elevatorMotorLeadConfig.Slot0.kI = kIslot0.get();
        elevatorMotorLeadConfig.Slot0.kD = kDslot0.get();
        elevatorMotorLeadConfig.Slot0.kS = kSslot0.get();
        elevatorMotorLeadConfig.Slot0.kV = kVslot0.get();
        elevatorMotorLeadConfig.Slot0.kA = kAslot0.get();
        elevatorMotorLeadConfig.Slot0.kG = kGslot0.get();

        elevatorMotorLeadConfig.Slot0.withGravityType(GravityTypeValue.Elevator_Static);

        elevatorMotorLeadConfig.Slot1.kP = kPslot1.get();
        elevatorMotorLeadConfig.Slot1.kI = kIslot1.get();
        elevatorMotorLeadConfig.Slot1.kD = kDslot1.get();
        elevatorMotorLeadConfig.Slot1.kS = kSslot1.get();
        elevatorMotorLeadConfig.Slot1.kV = kVslot1.get();
        elevatorMotorLeadConfig.Slot1.kA = kAslot1.get();
        elevatorMotorLeadConfig.Slot1.kG = kGslot1.get();

        elevatorMotorLeadConfig.Slot1.withGravityType(GravityTypeValue.Elevator_Static);

        elevatorMotorLeadConfig.Slot2.kP = kPslot2.get();
        elevatorMotorLeadConfig.Slot2.kI = kIslot2.get();
        elevatorMotorLeadConfig.Slot2.kD = kDslot2.get();
        elevatorMotorLeadConfig.Slot2.kS = kSslot2.get();
        elevatorMotorLeadConfig.Slot2.kV = kVslot2.get();
        elevatorMotorLeadConfig.Slot2.kA = kAslot2.get();
        elevatorMotorLeadConfig.Slot2.kG = kGslot2.get();

        elevatorMotorLeadConfig.Slot2.withGravityType(GravityTypeValue.Elevator_Static);

        leadMotorConfig.MotionMagicCruiseVelocity = ElevatorConstants.CRUISE_VELOCITY;
        leadMotorConfig.MotionMagicAcceleration = ElevatorConstants.ACCELERATION;
        leadMotorConfig.MotionMagicJerk = ElevatorConstants.JERK;

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(elevatorMotorLeadConfig);
            if (status.isOK()) {
                break;
            }
        }
        
        if (!status.isOK()) {
            configAlert.set(true);
            configAlert.setText(status.toString());
        }
        
        FaultReporter.getInstance().registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Lead", motor);
    }

    public void configElevatorMotorFollower(TalonFX motor){

        elevatorMotorFollowerConfig = new TalonFXConfiguration();

        StatusCode status = StatusCode.StatusCodeNotInitialized;

        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(elevatorMotorFollowerConfig);
            if (status.isOK()) {
                break;
            }
        }
        
        if (!status.isOK()) {
            configAlert.set(true);
            configAlert.setText(status.toString());
        }
        
        FaultReporter.getInstance().registerHardware(ElevatorConstants.SUBSYSTEM_NAME, "Elevator Motor Follower", motor);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        BaseStatusSignal.refreshAll(
            elevatorPositionStatusSignal,

            elevatorLeadStatorCurrentStatusSignal,
            elevatorFollowerStatorCurrentStatusSignal,

            elevatorLeadSupplyCurrentStatusSignal,

            elevatorLeadTempStatusSignal,
            elevatorFollowerTempStatusSignal
        );

        inputs.voltageSuppliedLead = elevatorLeadSupplyCurrentStatusSignal.getValueAsDouble();

        inputs.statorCurrentAmpsLead = elevatorLeadStatorCurrentStatusSignal.getValueAsDouble();
        inputs.statorCurrentAmpsFollower = elevatorFollowerStatorCurrentStatusSignal.getValueAsDouble();

        inputs.leadTempCelsius = elevatorLeadTempStatusSignal.getValueAsDouble();
        inputs.followerTempCelsius = elevatorFollowerTempStatusSignal.getValueAsDouble();

        inputs.closedLoopError = elevatorMotorLead.getClosedLoopError().getValueAsDouble();

        inputs.closedLoopReference = elevatorMotorLead.getClosedLoopReference().getValueAsDouble();

        inputs.positionInches = elevatorPositionStatusSignal.getValueAsDouble();


        if (kPslot0.hasChanged(kPslot0.hashCode()) || kIslot0.hasChanged(kIslot0.hashCode()) || kDslot0.hasChanged(kDslot0.hashCode()) || kSslot0.hasChanged(kSslot0.hashCode())) {
            Slot0Configs leadMotorConfig  = elevatorMotorLeadConfig.Slot0;
            elevatorMotorLead.getConfigurator().refresh(leadMotorConfig);
            leadMotorConfig.kP = kPslot0.get();
            leadMotorConfig.kI = kIslot0.get();
            leadMotorConfig.kD = kDslot0.get();
            leadMotorConfig.kS = kSslot0.get();
            elevatorMotorLead.getConfigurator().apply(leadMotorConfig);
        }
    }

    // Set motor voltage
    public void setMotorVoltage(double voltage) {
        elevatorMotorLead.setControl(elevatorMotorLeadExpoVoltageRequest.withPosition(voltage));
    }

    public void zeroPosition() {
        elevatorMotorLead.setPosition(0.0);
    }

    public void setStepVoltage(double voltage){
        elevatorMotorLead.setControl(elevatorMotorLeadExpoVoltageRequest.withPosition(voltage));
    }
}


