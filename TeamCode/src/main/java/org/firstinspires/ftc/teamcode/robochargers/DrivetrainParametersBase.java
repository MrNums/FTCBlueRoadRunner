package org.firstinspires.ftc.teamcode.robochargers;

/**
 * An interface to provide a framework for all of the parameters that the {@code MecanumDrivetrain} class needs to run all of the functions of the drivetrain.
 * 
 * <p>Use this interface by creating a {@code public final} class to hold the parameters and {@code implements} this interface.
 * Set each of the values in the interface in the custom class.
 * 
 * <p>Note that not all of these parameters need to be set in order to use some functions (e.g. open loop teleop driving).
 * However if a value isn't set that is used by some method in the {@code MecanumDrivetrain} there will almost certainly be runtime errors.
 */
public class DrivetrainParametersBase {
    /**
     * The name of the front left motor in the robot configuration.
     */
    public String FRONT_LEFT_NAME = "";

    /**
     * The name of the front right motor in the robot configuration.
     */
    public String FRONT_RIGHT_NAME = "";

    /**
     * The name of the rear left motor in the robot configuration.
     */
    public String REAR_LEFT_NAME = "";

    /**
     * The name of the rear right motor in the robot configuration.
     */
    public String REAR_RIGHT_NAME = "";
    
    /**
     * The axis of the control hub that is pointing upwards on the robot and will be used as the heading axis.
     * This also needs to factor in whether the axis is pointing up in the positive or negative direction.
     */
    public MecanumDrivetrain.GyroUpDirection GYRO_UP_DIRECTION;// = null;
    
    /**
     * The number of encoder pulses per rotation of the motor.
     */
    public double COUNTS_PER_MOTOR_REV;// = 0.0;

    /**
     * The reduction factor between the motor and the wheel.
     * This should include gearboxes and belt/chain reductions.
     * Value should be >1.0 if geared down to a lower speed and >1.0 if geared up.
     */
    public double DRIVE_GEAR_REDUCTION;// = 0.0;

    /**
     * The wheel radius in meters (m).
     */
    public double WHEEL_RADIUS;// = 0.0;

    /**Whether to invert the front left motor or not.
     * <p>{@code true} is the same as {@code DcMotor.Direction.FORWARD}
     * <p>{@code false} is the same as {@code DcMotor.Direction.REVERSE}
     */
    public boolean FRONT_LEFT_INVERTED;// = false;
    /**Whether to invert the front right motor or not.
     * <p>{@code true} is the same as {@code DcMotor.Direction.FORWARD}
     * <p>{@code false} is the same as {@code DcMotor.Direction.REVERSE}
     */
    public boolean FRONT_RIGHT_INVERTED;// = false;
    /**Whether to invert the rear left motor or not.
     * <p>{@code true} is the same as {@code DcMotor.Direction.FORWARD}
     * <p>{@code false} is the same as {@code DcMotor.Direction.REVERSE}
     */
    public boolean REAR_LEFT_INVERTED;// = false;
    /**Whether to invert the rear right motor or not.
     * <p>{@code true} is the same as {@code DcMotor.Direction.FORWARD}
     * <p>{@code false} is the same as {@code DcMotor.Direction.REVERSE}
     */
    public boolean REAR_RIGHT_INVERTED;// = false;


}