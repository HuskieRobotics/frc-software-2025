package frc.robot.subsystems.removal;

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

public class RemovalIOTalonFX implements RemovalIO {

    @Override
    public void setRollerMotorVoltage(double voltage) {
        // Implementation for setting the roller motor voltage
        rollVoltageOut = new VoltageOut(voltage);
        rollerMotor.setControl(rollVoltageOut);
    }
    private TalonFX rollerMotor;
    private Alert configAlert = new Alert("Failed to apply configuration for subsystem.", AlertType.kError);

    private VoltageOut rollVoltageOut;

    //Status siganls
    private StatusSignal<Current> rollerstatorCurrentStatusSignal;
    private StatusSignal<Temperature> rollerMotorTempCelciusStatusSignal;
    private StatusSignal<Voltage> rollerMotorVoltageStatusSignal;
    private StatusSignal<Current> rollerSupplyCurrentStatusSignal;
    private StatusSignal



    
}
