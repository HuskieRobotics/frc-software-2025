package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.LoggedTunableNumber;

public class Intake extends SubsystemBase {
    private static final String SUBSYSTEM_NAME = "INTAKE";

    private final IntakeIO io;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged(); 
    private final LEDs leds;
    
    private final IntakeMotor[] intakeMotors = {
        IntakeMotor.ROLLER, IntakeMotor.PIVOT,
    };

    private BooleanSupplier isShooterAngleReady;

    private IntakeState intakeState;
    private boolean automationEnabled;
    private int intakeAndKickerTimout;
    private boolean quickShootingEnabled;
    private double shootingTimestamp;

    private final LoggedTunableNumber rollerVelocity = new LoggedTunableNumber("Intake/rollerVelocity", 0.0);
}