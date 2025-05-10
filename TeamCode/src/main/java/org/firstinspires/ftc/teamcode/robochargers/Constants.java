package org.firstinspires.ftc.teamcode.robochargers;

/**
 * A class containing useful constant values that will be used frequently by teams.
 */
public final class Constants {
    /**
     * Constants relating to the UltraPlanetary gearbox.
     */
    public static class UltraPlanetary {
        /**The reduction of a 3:1 UltraPlanetary gear cartridge.  Equal to ~2.89 (84:29). */
        public static final double THREE_TO_ONE_REDUCTION = 84.0/29.0;
        /**The reduction of a 4:1 UltraPlanetary gear cartridge.  Equal to ~3.61 (76:21). */
        public static final double FOUR_TO_ONE_REDUCTION = 76.0/21.0;
        /**The reduction of a 5:1 UltraPlanetary gear cartridge.  Equal to ~5.23 (68:13). */
        public static final double FIVE_TO_ONE_REDUCTION = 68.0/13.0;
    }

    /**
     * Constants relating to the REV Robotics HD Hex motor.
     */
    public static class HDHexMotor {
        /**The stall torque of the HD Hex Motor in newton-meters (Nm) */
        public static final double STALL_TORQUE = 0.105;
        /**The free load speed of the HD Hex Motor in radians per second (rad/sec) */
        public static final double FREE_LOAD_SPEED = 624.12974;
    }
}
