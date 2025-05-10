package org.firstinspires.ftc.teamcode.robochargers;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.DriveParametersTemplate;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.lang.Math;

/**
 * A class to contain the parameters and functions for an FTC mecanum drivetrain.
 */
public class MecanumDrivetrain {

    /**
     * An enum for specifying which of the four (4) drive motors is being referred to.  Used for setting various parameters for individual motors (e.g. FORWARD/REVERSE direction)
     */
    public enum DriveMotor {
        FRONT_LEFT,
        FRONT_RIGHT,
        REAR_LEFT,
        REAR_RIGHT
    }

    /**
     * An enum for indicating what gyro axis the "heading" of the robot should be mapped to.  This will change depending on the orientation that the control hub is mounted to the robot in.
     * <p>{@code POSITIVE_X} NOT IMPLEMENTED!  The rear (USB port) of the control hub is facing DOWN
     * <p>{@code NEGATIVE_X} The rear (USB port) of the control hub is facing UP
     * <p>{@code POSITIVE_Y} NOT IMPLEMENTED!  The right side (sensor ports) of the control hub is facing UP
     * <p>{@code NEGATIVE_Y} NOT IMPLEMENTED!  The left side (motor ports) of the control hub is facing UP
     * <p>{@code POSITIVE_Z} NOT IMPLEMENTED!  The top (REV logo) of the control hub is facing UP
     * <p>{@code NEGATIVE_Z} NOT IMPLEMENTED!  The bottom (serial number sticker) of the control hub is facing UP
     */
    public enum GyroUpDirection {
        POSITIVE_X,
        NEGATIVE_X,
        POSITIVE_Y,
        NEGATIVE_Y,
        POSITIVE_Z,
        NEGATIVE_Z
    }

    /**
     * An enum to indicate which axis of the gyro is being referred to for various methods.
     */
    private enum GyroAxis {
        X,
        Y,
        Z
    }

    /**The bit sequence used by the BNO055 to indicate the X axis when remapping axes. */
    private static final byte X_BYTE = 0x0;
    /**The bit sequence used by the BNO055 to indicate the Y axis when remapping axes. */
    private static final byte Y_BYTE = 0x1;
    /**The bit sequence used by the BNO055 to indicate the Z axis when remapping axes. */
    private static final byte Z_BYTE = 0x2;

    //fields and properties
    private double headingOffset = 0.0;
    private double heading = 0.0;
    private double lastHeading = 0.0;
    /**
     * This function only works if the {@code updateHeading()} method is being called in the main robot loop
     * @return The current heading value in degrees relative to the start position
     */
    public double getHeading() {
        return heading + headingOffset;
    }

    private double COUNTS_PER_METER;

    //important members
    private HardwareMap hardwareMap;
    private DrivetrainParametersBase drivetrainParameters;
    /**
     * @return The object containing the parameters for this drivetrain.  This object will implement the {@code IDrivetrainParameters}
     */
    public DrivetrainParametersBase getDrivetrainParameters() {
        return drivetrainParameters;
    }
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearLeft;
    private DcMotorEx rearRight;

    private BNO055IMU imu;

    /**
     * Creates a new instance of the {@code MecanumDrivetrain} class.
     * This instance can be used to perform autonomous or teleop functions for a mecanum drivetrain as a single organized unit.
     * @param _hardwareMap A reference to the hardware map object for the robot.
     * Generally this is a field inherited from the {@code OpMode} class and you can just pass in {@code hardwareMap} and it will work as intended.
     * @param _drivetrainParameters A reference to a class that {@code implements} the {@code IDrivetrainParameters} interface.
     * This class should contain constants for a variety of parameters unique to this robot's drivetrain.
     * @param _frontLeftName The (case-sensitive) name given in the robot hardware configuration for the front left motor of the drivetrain.
     * @param _frontRightName The (case-sensitive) name given in the robot hardware configuration for the front right motor of the drivetrain.
     * @param _rearLeftName The (case-sensitive) name given in the robot hardware configuration for the rear left motor of the drivetrain.
     * @param _rearRightName The (case-sensitive) name given in the robot hardware configuration for the rear right motor of the drivetrain.
     */
    public <T extends DrivetrainParametersBase> MecanumDrivetrain(HardwareMap _hardwareMap,
                                                                  Class<DriveParametersTemplate> _drivetrainParameters)
                            throws InvalidParameterClassException {
        hardwareMap = _hardwareMap;

        try {
            drivetrainParameters = _drivetrainParameters.newInstance();
        } catch (InstantiationException e) {
            throw new InvalidParameterClassException("The drivetrain parameters object could not be instantiated.  Make sure the type specified can be constructed with 'newInstance()'.");
        } catch (IllegalAccessException e) {
            throw new InvalidParameterClassException("The access level of the specified type for the drivetrain parameters is not appropriate for it to be instantiated.");
        }
        
        COUNTS_PER_METER = (drivetrainParameters.COUNTS_PER_MOTOR_REV * drivetrainParameters.DRIVE_GEAR_REDUCTION) /
            (drivetrainParameters.WHEEL_RADIUS * Math.PI * 2.0);
        
        setupGyro(drivetrainParameters.GYRO_UP_DIRECTION);

        frontLeft  = (DcMotorEx)hardwareMap.get(DcMotor.class, drivetrainParameters.FRONT_LEFT_NAME);
        frontRight = (DcMotorEx)hardwareMap.get(DcMotor.class, drivetrainParameters.FRONT_RIGHT_NAME);
        rearLeft  = (DcMotorEx)hardwareMap.get(DcMotor.class, drivetrainParameters.REAR_LEFT_NAME);
        rearRight = (DcMotorEx)hardwareMap.get(DcMotor.class, drivetrainParameters.REAR_RIGHT_NAME);

        setMotorsInverted(drivetrainParameters.FRONT_LEFT_INVERTED,
                            drivetrainParameters.FRONT_RIGHT_INVERTED,
                            drivetrainParameters.REAR_LEFT_INVERTED,
                            drivetrainParameters.REAR_RIGHT_INVERTED);

        setZeroPowerBehavior(true);

        stopMotorsAndReset();
    }

    /**
     * Sets the inversion state of each of the four (4) drive motors.
     * <p>For each of the parameters:
     * <p>{@code true} is the same as {@code DcMotor.Direction.REVERSE}
     * <p>{@code false} is the same as {@code DcMotor.Direction.FORWARD}
     * @param _frontLeftInverted Whether to invert the front left motor.
     * @param _frontRightInverted Whether to invert the front right motor.
     * @param _rearLeftInverted Whether to invert the rear left motor.
     * @param _rearRightInverted Whether to invert the rear right motor.
     */
    private void setMotorsInverted(boolean _frontLeftInverted, boolean _frontRightInverted, boolean _rearLeftInverted, boolean _rearRightInverted) {
        frontLeft.setDirection( _frontLeftInverted? DcMotor.Direction.REVERSE:DcMotor.Direction.FORWARD);
        frontRight.setDirection(_frontRightInverted?DcMotor.Direction.REVERSE:DcMotor.Direction.FORWARD);
        rearLeft.setDirection(  _rearLeftInverted?  DcMotor.Direction.REVERSE:DcMotor.Direction.FORWARD);
        rearRight.setDirection( _rearRightInverted? DcMotor.Direction.REVERSE:DcMotor.Direction.FORWARD);
    }

    /**
     * Set up the IMU to give a gyro heading that corresponds to the heading of the robot.
     * This method is largely necessary because all of the gyro axes don't read the same way as the default Z axis.
     * If the Control Hub is mounted in a different orientation, the axes need to be reconfigured to pretend as if there is a different Z axis.
     * <p>Note that this method actually sends settings to the IMU IC and thus will affect any other part of the program that uses the IMU.
     * This class and method are written assuming that the IMU is only being used for the drivetrain and that the heading is the only IMU measurement that is important.
     * The way that the axes are reconfigured may negatively affect the behavior of other IMU measurements.
     * @param _upDirection A value from the {@code GyroUpDirection} enum that indicates which of the six (6) sides of the Control Hub is facing upwards.
     */
    private void setupGyro(GyroUpDirection _upDirection) {
        //Gyro setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        byte AXIS_MAP_CONFIG_BYTE = 0x0;
        byte AXIS_MAP_SIGN_BYTE = 0x0;

        //https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf#page=26
        switch (_upDirection) {
            case POSITIVE_X:
                throw new NotYetImplementedException("This orientation of the gyro isn't supported yet.");
                /*AXIS_MAP_CONFIG_BYTE = buildGyroAxisMap(GyroAxis.Z, GyroAxis.Y, GyroAxis.X); //6 LH
                AXIS_MAP_SIGN_BYTE   = buildGyroAxisSigns(true, false, false);
                break;*/

            case NEGATIVE_X:
                AXIS_MAP_CONFIG_BYTE = buildGyroAxisMap(GyroAxis.Z, GyroAxis.Y, GyroAxis.X); //6 LH
                AXIS_MAP_SIGN_BYTE   = buildGyroAxisSigns(false, false, true);
                break;

            case POSITIVE_Y:
                throw new NotYetImplementedException("This orientation of the gyro isn't supported yet.");
                /*AXIS_MAP_CONFIG_BYTE = buildGyroAxisMap(GyroAxis.X, GyroAxis.Z, GyroAxis.Y); //24 LH
                AXIS_MAP_SIGN_BYTE   = buildGyroAxisSigns(false, true, false);
                break;*/

            case NEGATIVE_Y:
                throw new NotYetImplementedException("This orientation of the gyro isn't supported yet.");
                /*AXIS_MAP_CONFIG_BYTE = buildGyroAxisMap(GyroAxis.Y, GyroAxis.Z, GyroAxis.X); //9 RH
                AXIS_MAP_SIGN_BYTE   = buildGyroAxisSigns(false, true, true);
                break;*/

            case POSITIVE_Z:
                /*throw new NotYetImplementedException("This orientation of the gyro isn't supported yet.");
                AXIS_MAP_CONFIG_BYTE = buildGyroAxisMap(GyroAxis.X, GyroAxis.Y, GyroAxis.Z); //36 RH
                AXIS_MAP_SIGN_BYTE   = buildGyroAxisSigns(false, false, false);*/
                break;

            case NEGATIVE_Z:
                throw new NotYetImplementedException("This orientation of the gyro isn't supported yet.");
                /*AXIS_MAP_CONFIG_BYTE = buildGyroAxisMap(GyroAxis.Y, GyroAxis.X, GyroAxis.Z); //33 LH
                AXIS_MAP_SIGN_BYTE   = buildGyroAxisSigns(false, false, true);
                break;*/

            default:
                throw new RuntimeException("Unhandled value for the GyroUpDirection enum.");
        }

        if (_upDirection != GyroUpDirection.POSITIVE_Z) {
            //Need to be in CONFIG mode to write to registers
            imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            //Changing modes requires a delay before doing anything else
            try {Thread.sleep(100);}
            catch (InterruptedException e) {System.out.println("Got interrupted!");}
            
            //Write to the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);

            //Write to the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);

            //Need to change back into the IMU mode to use the gyro
            imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);

            //Changing modes again requires a delay
            try {Thread.sleep(100);}
            catch (InterruptedException e) {System.out.println("Got interrupted!");}
        }
        
    }

    /**
     * Builds the correct byte to remap the axes of the BNO055 as specified.
     * @param _newXAxis The axis to map into the X axis slot.
     * @param _newYAxis The axis to map into the Y axis slot.
     * @param _newZAxis The axis to map into the Z axis slot.
     * @return
     */
    private byte buildGyroAxisMap(GyroAxis _newXAxis, GyroAxis _newYAxis, GyroAxis _newZAxis) {
        //check if any axes have been mapped more than once
        if ((_newXAxis == _newYAxis) ||
            (_newXAxis == _newZAxis) ||
            (_newYAxis == _newZAxis)) {
                throw new IllegalArgumentException("Each axis must be mapped to a unique value");
        }
        return (byte)((axisByte(_newZAxis) << 4) | (axisByte(_newYAxis) << 2) | (axisByte(_newXAxis)));
    }

    /**
     * Builds the correct byte to specify whether or not to negate each axis.
     * This operation takes place AFTER remapping the axes.
     * So if the original X axis has the Y axis mapped to it, and the Y axis is negated, the new Y axis (the X axis "slot") is negated.
     * @param _xAxisNegated Whether to negate the X axis.
     * @param _yAxisNegated Whether to negate the Y axis.
     * @param _zAxisNegated Whether to negate the Z axis.
     * @return
     */
    private byte buildGyroAxisSigns(boolean _xAxisNegated, boolean _yAxisNegated, boolean _zAxisNegated) {
        return (byte)(((_xAxisNegated?1:0)<<2) |
                      ((_yAxisNegated?1:0)<<1) |
                      (_zAxisNegated?1:0));
    }

    /**
     * Returns a bit sequence that indicates the specified axis.
     * The bit sequence is used for remapping axes in the BNO055 IMU.
     * @param _gyroAxis The axis to return the bit sequence for.
     * @return The bit sequence that corresponds to the specified axis.
     */
    private byte axisByte(GyroAxis _gyroAxis) {
        switch (_gyroAxis) {
            case X:
                return X_BYTE;
            case Y:
                return Y_BYTE;
            case Z:
                return Z_BYTE;
            default:
                throw new RuntimeException("Unhandled value for the GyroAxis enum.");
        }
    }

    /**
     * Sets the zero power behavior for a single drive motor.  This is generally referred to as the brake/coast mode or the brake/float mode.
     * @param _motorToSet A value from the {@code DriveMotors} enum that specifies which drive motor to set zero power behavior for.
     * @param _brakeMode A boolean value specifiying whether the motor should be in brake mode or not:
     * <p>{@code true} is the same as {@code DcMotor.ZeroPowerBehavior.BRAKE}
     * <p>{@code false} is the same as {@code DcMotor.ZeroPowerBehavior.FLOAT}
     */
    public void setZeroPowerBehavior(DriveMotor _motorToSet, boolean _brakeMode) {
        //lil ternary statement to convert a boolean brake mode value to the correct enum values to send to the motor
        DcMotor.ZeroPowerBehavior brakeMode = _brakeMode?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT;
        switch (_motorToSet) {
            case FRONT_LEFT:
                frontLeft.setZeroPowerBehavior(brakeMode);
                break;
            case FRONT_RIGHT:
                frontRight.setZeroPowerBehavior(brakeMode);
                break;
            case REAR_LEFT:
                rearLeft.setZeroPowerBehavior(brakeMode);
                break;
            case REAR_RIGHT:
                rearRight.setZeroPowerBehavior(brakeMode);
                break;
            default:
                throw new RuntimeException("Unhandled value for the DriveMotor enum.");
        }
    }

    /**
     * Sets the zero power behavior for all drive motors at once.
     * It is seldom useful to change the mode for a single motor and not the rest so this method is the best way to address all motors at once.
     * <p>Technically the motors are set in this order: front left > front right > rear left > rear right.  However the order rarely matters.
     * @param _brakeMode A boolean value specifiying whether the motors should be in brake mode or not:
     * <p>{@code true} is the same as {@code DcMotor.ZeroPowerBehavior.BRAKE}
     * <p>{@code false} is the same as {@code DcMotor.ZeroPowerBehavior.FLOAT}
     */
    public void setZeroPowerBehavior(boolean _brakeMode) {
        //lil ternary statement to convert a boolean brake mode value to the correct enum values to send to the motor
        DcMotor.ZeroPowerBehavior brakeMode = _brakeMode?DcMotor.ZeroPowerBehavior.BRAKE:DcMotor.ZeroPowerBehavior.FLOAT;
        frontLeft.setZeroPowerBehavior(brakeMode);
        frontRight.setZeroPowerBehavior(brakeMode);
        rearLeft.setZeroPowerBehavior(brakeMode);
        rearRight.setZeroPowerBehavior(brakeMode);
    }

    /**
     * Sets the run mode for all four (4) motors to {@code STOP_AND_RESET_ENCODER} and then back to {@code RUN_WITHOUT_ENCODER}.
     */
    public void stopMotorsAndReset() {
        //Reset encoders and then change back to the normal mode
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * An alias for {@code runMotorsIndividual(0.0, 0.0, 0.0, 0.0)}.
     */
    public void stopMotors() {
        runMotorsIndividual(0.0, 0.0, 0.0, 0.0);
    }

    /**
     * Runs all four drive motors with a simple standard mecanum mixing equation.
     * @param _drive A value between -1.0 and +1.0 that corresponds to the forward/reverse axis of the robot.  Positive is forwards.
     * Note that this is OPPOSITE from how most gamepads read the Y axis of the joystick so the input will likely need to be inverted by the user.
     * @param _strafe A value between -1.0 and +1.0 that corresponds to the left/right translational axis of the robot.  Positive is to the right.
     * @param _rotate A value between -1.0 and +1.0 that corresponds to the rotational axis of the robot.
     * Positive is left or counterclockwise rotation when viewed from above.
     * Note that this is OPPOSITE from how most gamepads read the X axis of the joystick so the input will likely need to be inverted by the user.
     */
    public void runMotorsMixed(double _drive, double _strafe, double _rotate) {
        frontLeft.setPower( Range.clip(_drive + _strafe - _rotate, -1.0, 1.0));
        frontRight.setPower(Range.clip(_drive - _strafe + _rotate, -1.0, 1.0));
        rearLeft.setPower(  Range.clip(_drive - _strafe - _rotate, -1.0, 1.0));
        rearRight.setPower( Range.clip(_drive + _strafe + _rotate, -1.0, 1.0));
    }

    /**
     * Sets the power to all four drive motors simultaneously.
     * <p>Technically the power is set in the same order as the parameters are defined, although that rarely matters.
     * @param _frontLeftPower The power to set the front left motor to.  This should be a value between -1.0 and +1.0 (inclusive).
     * @param _frontRightPower The power to set the front right motor to.  This should be a value between -1.0 and +1.0 (inclusive).
     * @param _rearLeftPower The power to set the rear left motor to.  This should be a value between -1.0 and +1.0 (inclusive).
     * @param _rearRightPower The power to set the rear right motor to.  This should be a value between -1.0 and +1.0 (inclusive).
     */
    public void runMotorsIndividual(double _frontLeftPower, double _frontRightPower, double _rearLeftPower, double _rearRightPower) {
        frontLeft.setPower( Range.clip(_frontLeftPower, -1.0, 1.0));
        frontRight.setPower(Range.clip(_frontRightPower, -1.0, 1.0));
        rearLeft.setPower(  Range.clip(_rearLeftPower, -1.0, 1.0));
        rearRight.setPower( Range.clip(_rearRightPower, -1.0, 1.0));
    }
     
    /**
    * Sets the power to all four drive motors given field-relative inputs and accounting for the orientation of the drivetrain.
    * @param _downField A value between -1.0 and +1.0 that corresponds to the down field direction (typically positive is away from the driver).
    * @param _crossField A value between -1.0 and +1.0 that corresponds to the lateral direction (typically positive is to the right).
    * @param _rotate A value between -1.0 and +1.0 that corresponds to the rotational axis of the robot.
     * Positive is left or counterclockwise rotation when viewed from above.
     * Note that this is OPPOSITE from how most gamepads read the X axis of the joystick so the input will likely need to be inverted by the user.
    */
    public void runMotorsFieldOriented(double _downField, double _crossField, double _rotate) {
        double stickAngle = Math.atan2(_downField, _crossField);
        double stickMagnitude = Math.sqrt(Math.pow(_downField, 2) + Math.pow(_crossField, 2));
        double movementAngle = stickAngle - Math.toRadians(getHeading()); 

        double drive = Math.sin(movementAngle) * stickMagnitude;
        double strafe = Math.cos(movementAngle) * stickMagnitude;

        runMotorsMixed(drive, strafe, _rotate);
    }

    /**
     * Sets the power for a single specified drive motor.
     * @param _motorToRun A value from the {@code DriveMotors} enum that specifies which drive motor to set the power for.
     * @param _motorPower The power to set the motor to.  This should be a value between -1.0 and +1.0 (inclusive).
     */
    public void runMotor(DriveMotor _motorToRun, double _motorPower) {
        switch (_motorToRun) {
            case FRONT_LEFT:
                frontLeft.setPower(Range.clip(_motorPower, -1.0, 1.0));
                break;
            case FRONT_RIGHT:
                frontRight.setPower(Range.clip(_motorPower, -1.0, 1.0));
                break;
            case REAR_LEFT:
                rearLeft.setPower(Range.clip(_motorPower, -1.0, 1.0));
                break;
            case REAR_RIGHT:
                rearRight.setPower(Range.clip(_motorPower, -1.0, 1.0));
                break;
            default:
                throw new RuntimeException("Unhandled value for the DriveMotor enum.");
        }
    }

    /**
     * Updates the interally stored heading from the gyro.
     * This heading is modified from the heading reported directly from the gyro in order to make the wrapping behavior more useable in autonomous functions.
     * <p>This method should be called inside the main robot loop somewhere, preferably at the beginning or at the end, not in the middle.
     * The actual integration does not happen in this method so the rate at which this method is called is not critical.
     * However if the robot moves more than 90deg between calls of this method, it is possible for this method to break and return bad data.
     */
    public void updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double newHeading = (double)angles.thirdAngle;

        double deltaHeading = newHeading - lastHeading;

        if (deltaHeading < -180)
            deltaHeading += 360 ; 
        else if (deltaHeading >= 180)
            deltaHeading -= 360 ;

        heading += deltaHeading;

        lastHeading = newHeading;
    }

    /**
     * @return The current positions of all four (4) motor encoders averaged together and converted to meters.
     */
    public double getCurrentAvgStraightPosition() {
        return (double)(
            frontLeft.getCurrentPosition() +
            frontRight.getCurrentPosition() +
            rearLeft.getCurrentPosition() +
            rearRight.getCurrentPosition()
        ) / 4 / COUNTS_PER_METER; //average of all encoder positions converted to meters
    }

    /**
     * @return The current velocities of all four (4) motor encoders averaged together and converted to meters per second (m/sec).
     */
    public double getCurrentAvgStraightVelocity() {
        return (double)(
            frontLeft.getVelocity() +
            frontRight.getVelocity() +
            rearLeft.getVelocity() +
            rearRight.getVelocity()
        ) / 4 / COUNTS_PER_METER; //average of all encoder positions converted to meters
    }

    /**
     * @return The current positions of all four (4) motor encoders averaged together in a way that provides the sideways position of the robot in meters.
     */
    public double getCurrentAvgSidewaysPosition() {
        return (double)(
            frontLeft.getCurrentPosition() -
            frontRight.getCurrentPosition() -
            rearLeft.getCurrentPosition() +
            rearRight.getCurrentPosition()
        ) / 4 / COUNTS_PER_METER; //average of all encoder positions converted to meters
    }

    /**
     * @return The current velocities of all four (4) motor encoders averaged together in a way that provides the sideways motion of the robot 
     * in meters per second (m/sec).
     */
    public double getCurrentAvgSidewaysVelocity() {
        return (double)(
            frontLeft.getVelocity() -
            frontRight.getVelocity() -
            rearLeft.getVelocity() +
            rearRight.getVelocity()
        ) / 4 / COUNTS_PER_METER; //average of all encoder positions converted to meters
    }

    /**
     * @return The current rotation rate in degrees per second (deg/sec).  Positive is to the left or counter clockwise when viewed from the top.
     */
    public double getRotationRate() {
        AngularVelocity angles = imu.getAngularVelocity();
        return angles.zRotationRate;
    }

    /**
     * Offset the gyro so that the current position is equal to the provided value.
     * @param _newHeading The angle (in degrees) the current position should be equal to.
     */
    public void setHeading(double _newHeading) {
        headingOffset = _newHeading - heading;
    }

    /**
     * Set the current orientation of the robot to be the 0 position of the gyro.
     */
    public void zeroHeading() {
        headingOffset = -heading;
    }
}