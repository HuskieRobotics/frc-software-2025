package frc.robot.subsystems.removal;

public class RemovalConstants{ 
    private static  final String CONSTRUCTOR_EXCEPTION = "constant class";

    private RemovalConstants() {
        throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
    }

    public static final boolean TESTING = false;

    public static final double REMOVAL_VELOCITY_ROLLERS_RPS = 0.0;

    public static final double ROLLER_VELOCITY_TOLERANCE = 0.0;

    public static final double ROLLERS_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 0.0;
    public static final double ROLLERS_PEAK_SUPPLY_CURRENT_LIMIT = 0.0;

    public static final double ROLLERS_PEAK_SUPPLY_CURRENT_DURATION = 0.0;

    public static final double ROLLERS_CONTINUOUS_STATOR_CURRENT_LIMIT = 0.0;

    public static final double IN_BETWEEN_TIMEOUT_SECONDS = 0.0;
}
