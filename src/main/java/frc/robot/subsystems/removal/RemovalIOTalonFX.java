package frc.robot.subsystems.removal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.subsystems.removal.RemovalIO.RemovalIOInputs;

public class RemovalIOTalonFX implements RemovalIO {

    @Override
    public void setRollerMotorVoltage(double voltage) {
        // Implementation for setting the roller motor voltage
        VoltageOut rollVoltageOut = new VoltageOut(voltage);
        rollerMotor.setControl(rollVoltageOut);
    }
    private TalonFX rollerMotor;
    private TalonFX rollerMotor2;
    
    private Alert configAlert = new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

    private VoltageOut rollerMotorVoltageRequest;
    private VoltageOut rollerMotor2VoltageRequest;


    //Status signals
    private StatusSignal<Current> rollerMotorStatorCurrentStatusSignal;
    private StatusSignal<Current> rollerMotor2StatorCurrentStatusSignal;

    private StatusSignal<Temperature> rollerMotorTempCelsiusStatusSignal;
    private StatusSignal<Temperature> rollerMotor2TempCelsiusStatusSignal;

    private StatusSignal<Voltage> rollerMotorVoltageStatusSignal;
    private StatusSignal<Voltage> rollerMotor2VoltageStatusSignal;

    private StatusSignal<Current> rollerMotorSupplyCurrentStatusSignal;
    private StatusSignal<Current> rollerMotor2SupplyCurrentStatusSignal;

    public RemovalIOTalonFX() {

        rollerMotor = new TalonFX(RemovalConstants.ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
        rollerMotor2 = new TalonFX(RemovalConstants.ROLLER_MOTOR2_ID, RobotConfig.getInstance().getCANBusName());
        rollerMotorVoltageRequest = new VoltageOut(0);
        rollerMotor2VoltageRequest = new VoltageOut(0);

        rollerMotorStatorCurrentStatusSignal = rollerMotor.getStatorCurrent();
        rollerMotor2StatorCurrentStatusSignal = rollerMotor2.getStatorCurrent();

        rollerMotorTempCelsiusStatusSignal = rollerMotor.getDeviceTemp();
        rollerMotor2TempCelsiusStatusSignal = rollerMotor2.getDeviceTemp();

        rollerMotorVoltageStatusSignal = rollerMotor.getMotorVoltage();
        rollerMotor2VoltageStatusSignal = rollerMotor2.getMotorVoltage();

        rollerMotorSupplyCurrentStatusSignal = rollerMotor.getSupplyCurrent();
        rollerMotor2SupplyCurrentStatusSignal = rollerMotor2.getSupplyCurrent();

        configureRemovalMotor(rollerMotor);
        configureRemovalMotor(rollerMotor2);

    }

    @Override
    public void updateInputs(RemovalIOInputs inputs) {
        
        BaseStatusSignal.refreshAll(
            rollerMotorStatorCurrentStatusSignal,
            rollerMotor2StatorCurrentStatusSignal,

            rollerMotorTempCelsiusStatusSignal,
            rollerMotor2TempCelsiusStatusSignal,

            rollerMotorVoltageStatusSignal,
            rollerMotor2VoltageStatusSignal,

            rollerMotorSupplyCurrentStatusSignal,
            rollerMotor2SupplyCurrentStatusSignal
        );
        
        inputs.rollerMotorVoltage = rollerMotorVoltageStatusSignal.getValueAsDouble();
        inputs.rollerMotor2Voltage = rollerMotor2VoltageStatusSignal.getValueAsDouble();

        inputs.rollerContinuousStatorCurrentLimit = rollerMotorStatorCurrentStatusSignal.getValueAsDouble();
        inputs.roller2ContinuousStatorCurrentLimit = rollerMotor2StatorCurrentStatusSignal.getValueAsDouble();

        inputs.rollerContinuousSupplyCurrentLimit = rollerMotorSupplyCurrentStatusSignal.getValueAsDouble();
        inputs.roller2ContinuousSupplyCurrentLimit = rollerMotor2SupplyCurrentStatusSignal.getValueAsDouble();

        inputs.rollerTempCelsius = rollerMotorTempCelsiusStatusSignal.getValueAsDouble();
        inputs.roller2TempCelsius = rollerMotor2TempCelsiusStatusSignal.getValueAsDouble();

    }
    
    private void configureRemovalMotor(TalonFX Motor){
        TalonFXConfiguration removalMotorConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs removalMotorCurrentLimits = new CurrentLimitsConfigs();

        removalMotorConfig.CurrentLimits = removalMotorCurrentLimits;

        removalMotorCurrentLimits.SupplyCurrentLimit = RemovalConstants.ROLLERS_PEAK_SUPPLY_CURRENT_LIMIT;
        removalMotorCurrentLimits.SupplyCurrentLimitEnable = true;

        removalMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        FaultReporter.getInstance().registerHardware("removal", "removalMotor", Motor);
        

    }



    
}
