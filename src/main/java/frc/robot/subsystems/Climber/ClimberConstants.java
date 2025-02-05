package frc.robot.subsystems.Climber;

public class ClimberConstants {
    public static final int CLIMBER_MOTOR_CAN_ID = 16;

    public static final boolean CLIMBER_MOTOR_INVERTED = false;
    public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 40.0;
    public static final double CLIMBER_STATOR_CURRENT_LIMIT = 60.0;
    public static final double CLIMBER_PEAK_CURRENT_LIMIT = 60.0;
    public static final double CLIMBER_PEAK_CURRENT_DURATION = 2.0;
    public static final double GEAR_RATIO = 25.0;

    //change extend + reset voltage constants later
    public static final double EXTEND_VOLTAGE = 15.0;
    public static final double RETRACT_VOLTAGE = 12.0;
    public static final double RESET_VOLTAGE = 20.0;

    public static final double KP = 0.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KS = 1.0;
    public static final double KV = 0.0;
    //potentially will be removed later
    public static final double KA = 0.0;
    public static final double KVEXP = 0.0;
    public static final double KAEXP = 0.0;
    public static final double KG = 0.0;
}
