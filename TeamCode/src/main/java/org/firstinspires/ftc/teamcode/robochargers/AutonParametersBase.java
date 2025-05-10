package org.firstinspires.ftc.teamcode.robochargers;

public class AutonParametersBase {
    /**
     * Whether to generate a data file for trapezoidal motion profiles and save it to storage.
     */
    public boolean MOTION_PROFILE_LOGGING = false;
    
    /**
     * The PID gains for maintaining heading during non-turning autonomous moves.
     */
    public PIDGains HEADING_GAINS = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    /**
     * The PID gains for maintaining position during turning moves in autonomous.
     */
    public PIDGains POSITION_GAINS = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);


    /**
     * The maximum speed for the trapezoidal motion profile when driving forwards/backwards in auton.
     * Speed is in meters per second (m/sec).
     */
    public double STRAIGHT_MAX_SPEED = 0.0;

    /**
     * The acceleration rate during the first phase of the trapezoidal motion profile while driving straight.
     * Rate is in meters per second per second (m/sec^2).
     * Should always be positive.
     */
    public double STRAIGHT_ACCELERATION_RATE = 0.0;

    /**
     * The deceleration rate during the third phase of the trapezoidal motion profile while driving straight.
     * Rate is in meters per second per second (m/sec^2).
     * Should always be positive.
     */
    public double STRAIGHT_DECELERATION_RATE = 0.0;

    /**
     * Used to calculate the default timeout time for straight autonomous moves.
     * The time to complete the ideal motion profile is multiplied by this value to set the timeout time.
     */
    public double STRAIGHT_TIMEOUT_MULTIPLIER = 0.0;

    /**
     * The feedforward and PID gains for straight (forwards/backwards) autonomous moves.
     */
    public PIDGains STRAIGHT_GAINS = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);



    /**
     * The maximum speed for the trapezoidal motion profile when driving sideways in auton.
     * Speed is in meters per second (m/sec).
     */
    public double STRAFE_MAX_SPEED = 0.0;

    /**
     * The acceleration rate during the first phase of the trapezoidal motion profile while driving sideways.
     * Rate is in meters per second per second (m/sec^2).
     * Should always be positive.
     */
    public double STRAFE_ACCELERATION_RATE = 0.0;

    /**
     * The deceleration rate during the third phase of the trapezoidal motion profile while driving sideways.
     * Rate is in meters per second per second (m/sec^2).
     * Should always be positive.
     */
    public double STRAFE_DECELERATION_RATE = 0.0;

    /**
     * Used to calculate the default timeout time for sideways autonomous moves.
     * The time to complete the ideal motion profile is multiplied by this value to set the timeout time.
     */
    public double STRAFE_TIMEOUT_MULTIPLIER = 0.0;

    /**
     * The feedforward and PID gains for sideways (left/right) autonomous moves.
     */
    public PIDGains STRAFE_GAINS = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);



    /**
     * The maximum angular speed for the trapezoidal motion profile when turning in place in auton.
     * Speed is in degrees per second (deg/sec).
     */
    public double TURNING_MAX_SPEED = 0.0;

    /**
     * The acceleration rate during the first phase of the trapezoidal motion profile while turning in place.
     * Rate is in degrees per second per second (deg/sec^2).
     * Should always be positive.
     */
    public double TURNING_ACCELERATION_RATE = 0.0;

    /**
     * The deceleration rate during the third phase of the trapezoidal motion profile while turning in place.
     * Rate is in degrees per second per second (deg/sec^2).
     * Should always be positive.
     */
    public double TURNING_DECELERATION_RATE = 0.0;

    /**
     * Used to calculate the default timeout time for turning autonomous moves.
     * The time to complete the ideal motion profile is multiplied by this value to set the timeout time.
     */
    public double TURNING_TIMEOUT_MULTIPLIER = 0.0;

    /**
     * The feedforward and PID gains for controlling heading during turning autonomous moves.
     */
    public PIDGains TURNING_GAINS = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}
