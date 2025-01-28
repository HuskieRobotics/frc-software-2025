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

    private TalonFX rollerMotor;
    
    private Alert configAlert = new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

    private VoltageOut rollerMotorVoltageRequest;


    //Status signals
    private StatusSignal<Current> rollerMotorStatorCurrentStatusSignal;

    private StatusSignal<Temperature> rollerMotorTempCelsiusStatusSignal;

    private StatusSignal<Voltage> rollerMotorVoltageStatusSignal;

    private StatusSignal<Current> rollerMotorSupplyCurrentStatusSignal;

    public RemovalIOTalonFX() {

        rollerMotor = new TalonFX(RemovalConstants.ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
        rollerMotorVoltageRequest = new VoltageOut(0);

        rollerMotorStatorCurrentStatusSignal = rollerMotor.getStatorCurrent();

        rollerMotorTempCelsiusStatusSignal = rollerMotor.getDeviceTemp();

        rollerMotorVoltageStatusSignal = rollerMotor.getMotorVoltage();

        rollerMotorSupplyCurrentStatusSignal = rollerMotor.getSupplyCurrent();

        configureRemovalMotor(rollerMotor);
    }

    @Override
    public void updateInputs(RemovalIOInputs inputs) {
        
        BaseStatusSignal.refreshAll(
            rollerMotorStatorCurrentStatusSignal,

            rollerMotorTempCelsiusStatusSignal,

            rollerMotorVoltageStatusSignal,

            rollerMotorSupplyCurrentStatusSignal
        );
        
        inputs.rollerMotorVoltage = rollerMotorVoltageStatusSignal.getValueAsDouble();

        inputs.rollerContinuousStatorCurrentLimit = rollerMotorStatorCurrentStatusSignal.getValueAsDouble();

        inputs.rollerContinuousSupplyCurrentLimit = rollerMotorSupplyCurrentStatusSignal.getValueAsDouble();

        inputs.rollerTempCelsius = rollerMotorTempCelsiusStatusSignal.getValueAsDouble();

    }

    @Override
    public void setRollerMotorVoltage(double voltage) {
        // Implementation for setting the roller motor voltage
        rollerMotor.setControl(rollerMotorVoltageRequest.withOutput(voltage));
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
