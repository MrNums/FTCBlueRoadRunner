package org.firstinspires.ftc.teamcode.robochargers;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.Math;

import java.io.IOException;
import java.io.Writer;
import java.io.File;
import java.io.FileWriter;

public class MecanumAuton {
    
    //important members
    private LinearOpMode opMode;

    private MecanumDrivetrain drivetrain;
    public MecanumDrivetrain getDrivetrain() {
        return drivetrain;
    }

    private AutonParametersBase autonParameters;
    public AutonParametersBase getParameters() {
        return autonParameters;
    }

    public <T extends AutonParametersBase> MecanumAuton(LinearOpMode _opMode, MecanumDrivetrain _drivetrain, Class<T> _autonParameters) {
        drivetrain = _drivetrain;
        opMode = _opMode;

        try {
            autonParameters = _autonParameters.newInstance();
        } catch (InstantiationException e) {
            throw new InvalidParameterClassException("The auton parameters object could not be instantiated.  Make sure the type specified can be constructed with 'newInstance()'.");
        } catch (IllegalAccessException e) {
            throw new InvalidParameterClassException("The access level of the specified type for the auton parameters is not appropriate for it to be instantiated.");
        }
    }

    /**
     * Performs a precise autonomous movement in the forward/backward directions.
     * Wheel slip is avoided by using a trapezoidal motion profile.  The heading is maintained using the gyro.
     * <p>
     * The constraints for the motion profile as well as the PIDF gains can be found in the {@code AutonParametersBase} class.
     * @param _distanceTarget The distance (in meters) to drive.  Positive is forwards and negative is backwards.
     */
    public void profiledDriveStraight(double _distanceTarget) {
        if (autonParameters.STRAIGHT_TIMEOUT_MULTIPLIER < 1.0) {
            throw new IllegalArgumentException("The value of STRAIGHT_TIMEOUT_MULTIPLIER must be >= 1.0!");
        }

        //calculate how much time the maneuver should take
        double v_squared = Math.pow(autonParameters.STRAIGHT_MAX_SPEED, 2);
            //the distance below which the trapezoidal motion profile becomes a triangle because it never reaches the max velocity
        double triangle_crossover_distance = 
            v_squared / (2 * autonParameters.STRAIGHT_ACCELERATION_RATE) +
            v_squared / (2 * autonParameters.STRAIGHT_DECELERATION_RATE);
        double absDistanceTarget = Math.abs(_distanceTarget);
        double required_time = 0.0;
        if (absDistanceTarget <= triangle_crossover_distance) { //triangular motion profile
            double new_velocity = Math.sqrt(
                (2 * absDistanceTarget * autonParameters.STRAIGHT_ACCELERATION_RATE * autonParameters.STRAIGHT_DECELERATION_RATE) / 
                (autonParameters.STRAIGHT_DECELERATION_RATE + autonParameters.STRAIGHT_ACCELERATION_RATE)
            );
            required_time =
                new_velocity / autonParameters.STRAIGHT_ACCELERATION_RATE +
                new_velocity / autonParameters.STRAIGHT_DECELERATION_RATE;
        } else { //trapezoidal motion profile
            required_time =
                autonParameters.STRAIGHT_MAX_SPEED / autonParameters.STRAIGHT_ACCELERATION_RATE +
                autonParameters.STRAIGHT_MAX_SPEED / autonParameters.STRAIGHT_DECELERATION_RATE +
                (absDistanceTarget - triangle_crossover_distance) / autonParameters.STRAIGHT_MAX_SPEED;
        }
        profiledDriveStraight(_distanceTarget, required_time * autonParameters.STRAIGHT_TIMEOUT_MULTIPLIER);
    }
    /**
     * WARNING: This version of this function is intended to be used behind the scenes and shouldn't be called unless necessary.
     * This function allows manually setting the autonomous timeout time which may produce undesirable results if done incorrectly.
     * <p>
     * Performs a precise autonomous movement in the forward/backward directions.
     * Wheel slip is avoided by using a trapezoidal motion profile.  The heading is maintained using the gyro.
     * <p>
     * The constraints for the motion profile as well as the PIDF gains can be found in the {@code AutonParametersBase} class.
     * @param _distanceTarget The distance (in meters) to drive.  Positive is forwards and negative is backwards.
     * @param _timeout The time to allow for the maneuver to complete. The robot will abruptly stop when this time limit is reached.
     * Autonomous will continue after the timeout.
     */
    public void profiledDriveStraight(double _distanceTarget, double _timeout) {
        //variable for how long this move has been running
        ElapsedTime movementTime = new ElapsedTime();
        
        //tolerances for completetion
        double maxStoppedSpeed = 0.01; // meters/sec
        double maxFinalError = 0.005; // meters

        Writer fileWriter = null;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //set up a file writer
            if (autonParameters.MOTION_PROFILE_LOGGING) {
                try {
                    String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
                    File directory = new File(directoryPath);
                    directory.mkdir();
                    fileWriter = new FileWriter(directoryPath + "/straightMotionProfile.csv", false);
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("Log file failed to open.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                    return;
                }

                try {
                    fileWriter.write("Time,Target Velocity,Velocity,Distance,Phase\n");
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("First line write failed.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                    return;
                }
            }
            
            //Reset encoders
            drivetrain.stopMotorsAndReset();

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            //Update the gyro and capture an offset for our starting heading
            drivetrain.updateHeading();
            double startHeading = drivetrain.getHeading();

            // reset the timeout time and start motion.
            movementTime.reset();

            // keep looping while we are still active
            boolean cont = true;
            boolean reachedTarget = false; //reached the target distance?
            boolean stopped = false; //did the robot reach zero speed?
            int phase = 1;
            
            while (opMode.opModeIsActive() &&
                   (movementTime.seconds() < _timeout) &&
                   cont
                   ) {
                
                //UPDATE THE GYRO INTEGRATION
                drivetrain.updateHeading();
                double angleError = drivetrain.getHeading() - startHeading; //calculate a heading relative to the start orientation (positive is left)

                double currentDistance = drivetrain.getCurrentAvgStraightPosition(); //average of all encoder positions converted to meters
                    
                double currentVelocity = drivetrain.getCurrentAvgStraightVelocity(); //average of all encoder velocities converted to m/sec

                double currentDrift = drivetrain.getCurrentAvgSidewaysPosition(); //average of all encoder positions in the sideways direction converted to meters
                
                //sign of the distance remaining to target
                double r = Math.signum(_distanceTarget - currentDistance);

                //intermediate variables
                double targetVelocity = 0.0;
                double velocityError  = 0.0;
                double deliveredPower = 0.0;
                
                if (phase == 1) { //ACCELERATION PHASE
                    //exit condition for phase 1
                    if (movementTime.seconds() >= autonParameters.STRAIGHT_MAX_SPEED / autonParameters.STRAIGHT_ACCELERATION_RATE) {
                        phase = 2;
                        //cont = false;
                    }

                    //skip to phase 3 if it's time to start decelerating in order to make the distance target
                    if (Math.abs(currentDistance) >=
                            Math.abs(_distanceTarget) - (  Math.pow(currentVelocity, 2)/(2*autonParameters.STRAIGHT_DECELERATION_RATE)  )) {
                        phase = 3;
                    }

                    targetVelocity = movementTime.seconds() * autonParameters.STRAIGHT_ACCELERATION_RATE;
                    velocityError = (targetVelocity * r) - currentVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.STRAIGHT_GAINS.kS +
                                        autonParameters.STRAIGHT_GAINS.kV * targetVelocity +
                                        autonParameters.STRAIGHT_GAINS.kE * Math.pow(targetVelocity, 2) +
                                        autonParameters.STRAIGHT_GAINS.kA * autonParameters.STRAIGHT_ACCELERATION_RATE
                                        ) +
                                        autonParameters.STRAIGHT_GAINS.kP * velocityError;
                }
                else if (phase == 2) { //CRUISE PHASE
                    //exit condition for phase 2
                    if (Math.abs(currentDistance) >=
                            Math.abs(_distanceTarget) - (  Math.pow(currentVelocity, 2)/(2*autonParameters.STRAIGHT_DECELERATION_RATE)  )) {
                        phase = 3;
                    }
                    
                    targetVelocity = autonParameters.STRAIGHT_MAX_SPEED;
                    velocityError = (targetVelocity * r) - currentVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.STRAIGHT_GAINS.kS +
                                        autonParameters.STRAIGHT_GAINS.kV * targetVelocity +
                                        autonParameters.STRAIGHT_GAINS.kE * Math.pow(targetVelocity, 2)
                                        ) +
                                        autonParameters.STRAIGHT_GAINS.kP * velocityError;
                }
                else if (phase == 3) { //DECELERATION PHASE
                    //exit condition for the entire move
                    if (Math.abs(_distanceTarget - currentDistance) < maxFinalError) {
                        reachedTarget = true;
                    }
                    
                    if (Math.abs(currentVelocity) < maxStoppedSpeed) {
                        stopped = true;
                    }
                    
                    cont = !(reachedTarget || stopped); //keep waiting if either condition is true

                    targetVelocity = Math.sqrt(2 * autonParameters.STRAIGHT_DECELERATION_RATE * Math.abs(_distanceTarget - currentDistance));
                    velocityError = (targetVelocity * r) - currentVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.STRAIGHT_GAINS.kS +
                                        autonParameters.STRAIGHT_GAINS.kV * targetVelocity +
                                        autonParameters.STRAIGHT_GAINS.kE * Math.pow(targetVelocity, 2) - //minus here because deceleration
                                        autonParameters.STRAIGHT_GAINS.kA * autonParameters.STRAIGHT_DECELERATION_RATE
                                        ) +
                                        autonParameters.STRAIGHT_GAINS.kP * velocityError;
                }
                else {cont = false;} //invalid phase, just exit the loop

                double turningPower = autonParameters.HEADING_GAINS.kP * angleError; //this will be added to the left side and subtacted from the right
                double driftPower = autonParameters.POSITION_GAINS.kP * (-currentDrift); //this will counteract lateral drift

                drivetrain.runMotorsIndividual(
                        deliveredPower + turningPower + driftPower,
                        deliveredPower - turningPower - driftPower,
                        deliveredPower + turningPower - driftPower,
                        deliveredPower - turningPower + driftPower);
                
                opMode.telemetry.addData("Phase", phase);
                opMode.telemetry.addData("Elapsed Time (sec)", movementTime.seconds());
                opMode.telemetry.addData("Gyro Error (deg)", angleError);
                opMode.telemetry.addData("Distance (m)", currentDistance);
                opMode.telemetry.addData("Velocity (m/sec)", currentVelocity);
                opMode.telemetry.addData("Velocity Error (m/sec)", velocityError);
                opMode.telemetry.addData("Delivered Power (%)", deliveredPower);
                opMode.telemetry.update();

                if (autonParameters.MOTION_PROFILE_LOGGING) {
                    //Write data to the data file
                    try {
                        fileWriter.write(
                            String.format("%.4f",movementTime.seconds()) + "," + 
                            String.format("%.4f",targetVelocity) + "," + 
                            String.format("%.4f",currentVelocity) + "," + 
                            String.format("%.4f",currentDistance) + "," + 
                            String.valueOf(phase) + "," + 
                            "\n");
                    }
                    catch (IOException e) {
                        opMode.telemetry.addLine("Writing to the log file failed during execution.");
                        opMode.telemetry.update();
                        opMode.sleep(3000);
                        return;
                    }
                }
            }

            //Close the data file
            if (autonParameters.MOTION_PROFILE_LOGGING) {
                try {
                    fileWriter.close();
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("Characterization file failed to close.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                }
            }

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            // Stop all motion;
            drivetrain.stopMotors();
        }
    }

    /**
     * Performs a precise autonomous movement sideways.
     * Wheel slip is avoided by using a trapezoidal motion profile.  The heading is maintained using the gyro.
     * <p>
     * The constraints for the motion profile as well as the PIDF gains can be found in the {@code AutonParametersBase} class.
     * @param _distanceTarget The distance (in meters) to drive.  Positive is right and negative is left.
     */
    public void profiledDriveSideways(double _distanceTarget) {
        if (autonParameters.STRAFE_TIMEOUT_MULTIPLIER < 1.0) {
            throw new IllegalArgumentException("The value of STRAFE_TIMEOUT_MULTIPLIER must be >= 1.0!");
        }

        //calculate how much time the maneuver should take
        double v_squared = Math.pow(autonParameters.STRAFE_MAX_SPEED, 2);
            //the distance below which the trapezoidal motion profile becomes a triangle because it never reaches the max velocity
        double triangle_crossover_distance = 
            v_squared / (2 * autonParameters.STRAFE_ACCELERATION_RATE) +
            v_squared / (2 * autonParameters.STRAFE_DECELERATION_RATE);
        double absDistanceTarget = Math.abs(_distanceTarget);
        double required_time = 0.0;
        if (absDistanceTarget <= triangle_crossover_distance) { //triangular motion profile
            double new_velocity = Math.sqrt(
                (2 * absDistanceTarget * autonParameters.STRAFE_ACCELERATION_RATE * autonParameters.STRAFE_DECELERATION_RATE) / 
                (autonParameters.STRAFE_DECELERATION_RATE + autonParameters.STRAFE_ACCELERATION_RATE)
            );
            required_time =
                new_velocity / autonParameters.STRAFE_ACCELERATION_RATE +
                new_velocity / autonParameters.STRAFE_DECELERATION_RATE;
        } else { //trapezoidal motion profile
            required_time =
                autonParameters.STRAFE_MAX_SPEED / autonParameters.STRAFE_ACCELERATION_RATE +
                autonParameters.STRAFE_MAX_SPEED / autonParameters.STRAFE_DECELERATION_RATE +
                (absDistanceTarget - triangle_crossover_distance) / autonParameters.STRAFE_MAX_SPEED;
        }
        profiledDriveSideways(_distanceTarget, required_time * autonParameters.STRAFE_TIMEOUT_MULTIPLIER);
    }
    /**
     * WARNING: This version of this function is intended to be used behind the scenes and shouldn't be called unless necessary.
     * This function allows manually setting the autonomous timeout time which may produce undesirable results if done incorrectly.
     * <p>
     * Performs a precise autonomous movement sideways.
     * Wheel slip is avoided by using a trapezoidal motion profile.  The heading is maintained using the gyro.
     * <p>
     * The constraints for the motion profile as well as the PIDF gains can be found in the {@code AutonParametersBase} class.
     * @param _distanceTarget The distance (in meters) to drive.  Positive is right and negative is left.
     * @param _timeout The time to allow for the maneuver to complete. The robot will abruptly stop when this time limit is reached.
     * Autonomous will continue after the timeout.
     */
    public void profiledDriveSideways(double _distanceTarget, double _timeout) {
        //variable for how long this move has been running
        ElapsedTime movementTime = new ElapsedTime();
        
        //tolerances for completetion
        double maxStoppedSpeed = 0.01; // meters/sec
        double maxFinalError = 0.005; // meters

        Writer fileWriter = null;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //set up a file writer
            if (autonParameters.MOTION_PROFILE_LOGGING) {
                try {
                    String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
                    File directory = new File(directoryPath);
                    directory.mkdir();
                    fileWriter = new FileWriter(directoryPath + "/sidewaysMotionProfile.csv", false);
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("Log file failed to open.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                    return;
                }

                try {
                    fileWriter.write("Time,Target Velocity,Velocity,Distance,Phase\n");
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("First line write failed.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                    return;
                }
            }
            
            //Reset encoders
            drivetrain.stopMotorsAndReset();

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            //Update the gyro and capture an offset for our starting heading
            drivetrain.updateHeading();
            double startHeading = drivetrain.getHeading();

            // reset the timeout time and start motion.
            movementTime.reset();

            // keep looping while we are still active
            boolean cont = true;
            boolean reachedTarget = false; //reached the target distance?
            boolean stopped = false; //did the robot reach zero speed?
            int phase = 1;
            
            while (opMode.opModeIsActive() &&
                   (movementTime.seconds() < _timeout) &&
                   cont
                   ) {
                
                //UPDATE THE GYRO INTEGRATION
                drivetrain.updateHeading();
                double angleError = drivetrain.getHeading() - startHeading; //calculate a heading relative to the start orientation (positive is left)

                double currentDistance = drivetrain.getCurrentAvgSidewaysPosition(); //average of all encoder positions converted to meters
                    
                double currentVelocity = drivetrain.getCurrentAvgSidewaysVelocity(); //average of all encoder velocities converted to m/sec

                double currentDrift = drivetrain.getCurrentAvgStraightPosition(); //average of all encoder positions in the forward/reverse direction converted to meters
                
                //sign of the distance remaining to target
                double r = Math.signum(_distanceTarget - currentDistance);

                //intermediate variables
                double targetVelocity = 0.0;
                double velocityError  = 0.0;
                double deliveredPower = 0.0;
                
                if (phase == 1) { //ACCELERATION PHASE
                    //exit condition for phase 1
                    if (movementTime.seconds() >= autonParameters.STRAFE_MAX_SPEED / autonParameters.STRAFE_ACCELERATION_RATE) {
                        phase = 2;
                        //cont = false;
                    }

                    //skip to phase 3 if it's time to start decelerating in order to make the distance target
                    if (Math.abs(currentDistance) >=
                            Math.abs(_distanceTarget) - (  Math.pow(currentVelocity, 2)/(2*autonParameters.STRAFE_DECELERATION_RATE)  )) {
                        phase = 3;
                    }

                    targetVelocity = movementTime.seconds() * autonParameters.STRAFE_ACCELERATION_RATE;
                    velocityError = (targetVelocity * r) - currentVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.STRAFE_GAINS.kS +
                                        autonParameters.STRAFE_GAINS.kV * targetVelocity +
                                        autonParameters.STRAFE_GAINS.kE * Math.pow(targetVelocity, 2) +
                                        autonParameters.STRAFE_GAINS.kA * autonParameters.STRAFE_ACCELERATION_RATE
                                        ) +
                                        autonParameters.STRAFE_GAINS.kP * velocityError;
                }
                else if (phase == 2) { //CRUISE PHASE
                    //exit condition for phase 2
                    if (Math.abs(currentDistance) >=
                            Math.abs(_distanceTarget) - (  Math.pow(currentVelocity, 2)/(2*autonParameters.STRAFE_DECELERATION_RATE)  )) {
                        phase = 3;
                    }
                    
                    targetVelocity = autonParameters.STRAFE_MAX_SPEED;
                    velocityError = (targetVelocity * r) - currentVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.STRAFE_GAINS.kS +
                                        autonParameters.STRAFE_GAINS.kV * targetVelocity +
                                        autonParameters.STRAFE_GAINS.kE * Math.pow(targetVelocity, 2)
                                        ) +
                                        autonParameters.STRAFE_GAINS.kP * velocityError;
                }
                else if (phase == 3) { //DECELERATION PHASE
                    //exit condition for the entire move
                    if (Math.abs(_distanceTarget - currentDistance) < maxFinalError) {
                        reachedTarget = true;
                    }
                    
                    if (Math.abs(currentVelocity) < maxStoppedSpeed) {
                        stopped = true;
                    }
                    
                    cont = !(reachedTarget || stopped); //keep waiting if either condition is true

                    targetVelocity = Math.sqrt(2 * autonParameters.STRAFE_DECELERATION_RATE * Math.abs(_distanceTarget - currentDistance));
                    velocityError = (targetVelocity * r) - currentVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.STRAFE_GAINS.kS +
                                        autonParameters.STRAFE_GAINS.kV * targetVelocity +
                                        autonParameters.STRAFE_GAINS.kE * Math.pow(targetVelocity, 2) - //minus here because deceleration
                                        autonParameters.STRAFE_GAINS.kA * autonParameters.STRAFE_DECELERATION_RATE
                                        ) +
                                        autonParameters.STRAFE_GAINS.kP * velocityError;
                }
                else {cont = false;} //invalid phase, just exit the loop

                double turningPower = autonParameters.HEADING_GAINS.kP * angleError; //this will be added to the left side and subtacted from the right
                double driftPower = autonParameters.POSITION_GAINS.kP * (-currentDrift); //this will counteract forward/reverse drift

                drivetrain.runMotorsIndividual(
                          deliveredPower + turningPower + driftPower,
                        - deliveredPower - turningPower + driftPower,
                        - deliveredPower + turningPower + driftPower,
                          deliveredPower - turningPower + driftPower);
                
                opMode.telemetry.addData("Phase", phase);
                opMode.telemetry.addData("Elapsed Time (sec)", movementTime.seconds());
                opMode.telemetry.addData("Gyro Error (deg)", angleError);
                opMode.telemetry.addData("Distance (m)", currentDistance);
                opMode.telemetry.addData("Velocity (m/sec)", currentVelocity);
                opMode.telemetry.addData("Velocity Error (m/sec)", velocityError);
                opMode.telemetry.addData("Delivered Power (%)", deliveredPower);
                opMode.telemetry.update();

                if (autonParameters.MOTION_PROFILE_LOGGING) {
                    //Write data to the data file
                    try {
                        fileWriter.write(
                            String.format("%.4f",movementTime.seconds()) + "," + 
                            String.format("%.4f",targetVelocity) + "," + 
                            String.format("%.4f",currentVelocity) + "," + 
                            String.format("%.4f",currentDistance) + "," + 
                            String.valueOf(phase) + "," + 
                            "\n");
                    }
                    catch (IOException e) {
                        opMode.telemetry.addLine("Writing to the log file failed during execution.");
                        opMode.telemetry.update();
                        opMode.sleep(3000);
                        return;
                    }
                }
            }

            //Close the data file
            if (autonParameters.MOTION_PROFILE_LOGGING) {
                try {
                    fileWriter.close();
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("Characterization file failed to close.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                }
            }

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            // Stop all motion;
            drivetrain.stopMotors();
        }
    }

    /**
     * Performs a precise autonomous rotation.
     * Wheel slip is avoided by using a trapezoidal motion profile.  The heading is maintained using the gyro.
     * <p>
     * The constraints for the motion profile as well as the PIDF gains can be found in the {@code AutonParametersBase} class.
     * @param _angleTarget The angle (in degrees) to turn.  Positive is left and negative is right.
     */
    public void profiledTurnHeading(double _angleTarget) {
        if (autonParameters.TURNING_TIMEOUT_MULTIPLIER < 1.0) {
            throw new IllegalArgumentException("The value of TURNING_TIMEOUT_MULTIPLIER must be >= 1.0!");
        }

        //calculate how much time the maneuver should take
        double v_squared = Math.pow(autonParameters.TURNING_MAX_SPEED, 2);
            //the angle below which the trapezoidal motion profile becomes a triangle because it never reaches the max velocity
        double triangle_crossover_angle = 
            v_squared / (2 * autonParameters.TURNING_ACCELERATION_RATE) +
            v_squared / (2 * autonParameters.TURNING_DECELERATION_RATE);
        double absAngleTarget = Math.abs(_angleTarget);
        double required_time = 0.0;
        if (absAngleTarget <= triangle_crossover_angle) { //triangular motion profile
            double new_velocity = Math.sqrt(
                (2 * absAngleTarget * autonParameters.TURNING_ACCELERATION_RATE * autonParameters.TURNING_DECELERATION_RATE) / 
                (autonParameters.TURNING_DECELERATION_RATE + autonParameters.TURNING_ACCELERATION_RATE)
            );
            required_time =
                new_velocity / autonParameters.TURNING_ACCELERATION_RATE +
                new_velocity / autonParameters.TURNING_DECELERATION_RATE;
        } else { //trapezoidal motion profile
            required_time =
                autonParameters.TURNING_MAX_SPEED / autonParameters.TURNING_ACCELERATION_RATE +
                autonParameters.TURNING_MAX_SPEED / autonParameters.TURNING_DECELERATION_RATE +
                (absAngleTarget - triangle_crossover_angle) / autonParameters.TURNING_MAX_SPEED;
        }
        profiledTurnHeading(_angleTarget, required_time * autonParameters.TURNING_TIMEOUT_MULTIPLIER);
    }
    /**
     * WARNING: This version of this function is intended to be used behind the scenes and shouldn't be called unless necessary.
     * This function allows manually setting the autonomous timeout time which may produce undesirable results if done incorrectly.
     * <p>
     * Performs a precise autonomous rotation.
     * Wheel slip is avoided by using a trapezoidal motion profile.  The heading is maintained using the gyro.
     * <p>
     * The constraints for the motion profile as well as the PIDF gains can be found in the {@code AutonParametersBase} class.
     * @param _angleTarget The angle (in degrees) to turn.  Positive is left and negative is right.
     * @param _timeout The time to allow for the maneuver to complete. The robot will abruptly stop when this time limit is reached.
     * Autonomous will continue after the timeout.
     */
    public void profiledTurnHeading(double _angleTarget, double _timeout) {
        //variable for how long this move has been running
        ElapsedTime movementTime = new ElapsedTime();
        
        //tolerances for completetion
        double maxStoppedSpeed = 1.0; // degrees/sec
        double maxFinalError = 0.5; // degrees

        Writer fileWriter = null;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            //set up a file writer
            if (autonParameters.MOTION_PROFILE_LOGGING) {
                try {
                    String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
                    File directory = new File(directoryPath);
                    directory.mkdir();
                    fileWriter = new FileWriter(directoryPath + "/turningMotionProfile.csv", false);
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("Log file failed to open.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                    return;
                }

                try {
                    fileWriter.write("Time,Target Angular Velocity,Angular Velocity,Angle,Phase\n");
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("First line write failed.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                    return;
                }
            }
            
            //Reset encoders
            drivetrain.stopMotorsAndReset();

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            //Update the gyro and capture an offset for our starting heading
            drivetrain.updateHeading();
            double startHeading = drivetrain.getHeading();

            // reset the timeout time and start motion.
            movementTime.reset();

            // keep looping while we are still active
            boolean cont = true;
            boolean reachedTarget = false; //reached the target distance?
            boolean stopped = false; //did the robot reach zero speed?
            int phase = 1;
            
            while (opMode.opModeIsActive() &&
                   (movementTime.seconds() < _timeout) &&
                   cont
                   ) {
                
                //UPDATE THE GYRO INTEGRATION
                drivetrain.updateHeading();
                double currentAngle = drivetrain.getHeading() - startHeading; //calculate a heading relative to the start orientation (positive is left)

                double currentPosition = drivetrain.getCurrentAvgStraightPosition(); //average of all encoder positions converted to meters
                    
                double currentAngVelocity = drivetrain.getRotationRate(); //average of all encoder velocities converted to m/sec

                double currentDrift = drivetrain.getCurrentAvgSidewaysPosition(); //average of all encoder positions in the sideways direction converted to meters
                
                //sign of the distance remaining to target
                double r = Math.signum(_angleTarget - currentAngle);

                //intermediate variables
                double targetAngVelocity = 0.0;
                double angVelocityError  = 0.0;
                double deliveredPower = 0.0;
                
                if (phase == 1) { //ACCELERATION PHASE
                    //exit condition for phase 1
                    if (movementTime.seconds() >= autonParameters.TURNING_MAX_SPEED / autonParameters.TURNING_ACCELERATION_RATE) {
                        phase = 2;
                        //cont = false;
                    }

                    //skip to phase 3 if it's time to start decelerating in order to make the distance target
                    if (Math.abs(currentAngle) >=
                            Math.abs(_angleTarget) - (  Math.pow(currentAngVelocity, 2)/(2*autonParameters.TURNING_DECELERATION_RATE)  )) {
                        phase = 3;
                    }

                    targetAngVelocity = movementTime.seconds() * autonParameters.TURNING_ACCELERATION_RATE;
                    angVelocityError = (targetAngVelocity * r) - currentAngVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.TURNING_GAINS.kS +
                                        autonParameters.TURNING_GAINS.kV * targetAngVelocity +
                                        autonParameters.TURNING_GAINS.kE * Math.pow(targetAngVelocity, 2) +
                                        autonParameters.TURNING_GAINS.kA * autonParameters.TURNING_ACCELERATION_RATE
                                        ) +
                                        autonParameters.TURNING_GAINS.kP * angVelocityError;
                }
                else if (phase == 2) { //CRUISE PHASE
                    //exit condition for phase 2
                    if (Math.abs(currentAngle) >=
                            Math.abs(_angleTarget) - (  Math.pow(currentAngVelocity, 2)/(2*autonParameters.TURNING_DECELERATION_RATE)  )) {
                        phase = 3;
                    }
                    
                    targetAngVelocity = autonParameters.TURNING_MAX_SPEED;
                    angVelocityError = (targetAngVelocity * r) - currentAngVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.TURNING_GAINS.kS +
                                        autonParameters.TURNING_GAINS.kV * targetAngVelocity +
                                        autonParameters.TURNING_GAINS.kE * Math.pow(targetAngVelocity, 2)
                                        ) +
                                        autonParameters.TURNING_GAINS.kP * angVelocityError;
                }
                else if (phase == 3) { //DECELERATION PHASE
                    //exit condition for the entire move
                    if (Math.abs(_angleTarget - currentAngle) < maxFinalError) {
                        reachedTarget = true;
                    }
                    
                    if (Math.abs(currentAngVelocity) < maxStoppedSpeed) {
                        stopped = true;
                    }
                    
                    cont = !(reachedTarget || stopped); //keep waiting if either condition is true

                    targetAngVelocity = Math.sqrt(2 * autonParameters.TURNING_DECELERATION_RATE * Math.abs(_angleTarget - currentAngle));
                    angVelocityError = (targetAngVelocity * r) - currentAngVelocity;
                    
                    deliveredPower    = r * (
                                        autonParameters.TURNING_GAINS.kS +
                                        autonParameters.TURNING_GAINS.kV * targetAngVelocity +
                                        autonParameters.TURNING_GAINS.kE * Math.pow(targetAngVelocity, 2) - //minus here because deceleration
                                        autonParameters.TURNING_GAINS.kA * autonParameters.TURNING_DECELERATION_RATE
                                        ) +
                                        autonParameters.TURNING_GAINS.kP * angVelocityError;
                }
                else {cont = false;} //invalid phase, just exit the loop

                double centeringPower = autonParameters.POSITION_GAINS.kP * (-currentPosition); //this will be added to all drive motors to keep the robot centered while turning
                double driftPower = autonParameters.POSITION_GAINS.kP * (-currentDrift); //this will counteract lateral drift

                drivetrain.runMotorsIndividual(
                        - deliveredPower + centeringPower + driftPower,
                          deliveredPower + centeringPower - driftPower,
                        - deliveredPower + centeringPower - driftPower,
                          deliveredPower + centeringPower + driftPower);
                
                opMode.telemetry.addData("Phase", phase);
                opMode.telemetry.addData("Elapsed Time (sec)", movementTime.seconds());
                opMode.telemetry.addData("Gyro Angle (deg)", currentAngle);
                opMode.telemetry.addData("Position (m)", currentPosition);
                opMode.telemetry.addData("Angular Velocity (deg/sec)", currentAngVelocity);
                opMode.telemetry.addData("Velocity Error (deg/sec)", angVelocityError);
                opMode.telemetry.addData("Delivered Power (%)", deliveredPower);
                opMode.telemetry.update();

                if (autonParameters.MOTION_PROFILE_LOGGING) {
                    //Write data to the data file
                    try {
                        fileWriter.write(
                            String.format("%.4f",movementTime.seconds()) + "," + 
                            String.format("%.4f",targetAngVelocity) + "," + 
                            String.format("%.4f",currentAngVelocity) + "," + 
                            String.format("%.4f",currentAngle) + "," + 
                            String.valueOf(phase) + "," + 
                            "\n");
                    }
                    catch (IOException e) {
                        opMode.telemetry.addLine("Writing to the log file failed during execution.");
                        opMode.telemetry.update();
                        opMode.sleep(3000);
                        return;
                    }
                }
            }

            //Close the data file
            if (autonParameters.MOTION_PROFILE_LOGGING) {
                try {
                    fileWriter.close();
                }
                catch (IOException e) {
                    opMode.telemetry.addLine("Characterization file failed to close.");
                    opMode.telemetry.update();
                    opMode.sleep(3000);
                }
            }

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            // Stop all motion;
            drivetrain.stopMotors();
        }
    }

            /**
     * Performs precise autonomous movement in a diagonal direction.
     * Wheel slip is avoided by using a trapezoidal motion profile, and heading is maintained using the gyro.
     *
     * @param _distanceTarget The diagonal distance (in meters) to drive. Positive is forwards, and negative is backwards.
     * @param diagonalDirection Direction of diagonal movement:
     *                          "FR" = Forward-Right,
     *                          "FL" = Forward-Left,
     *                          "BR" = Backward-Right,
     *                          "BL" = Backward-Left.
     */
    public void profiledDriveDiagonal(double _distanceTarget, String diagonalDirection) {
        if (!isValidDiagonalDirection(diagonalDirection)) {
            throw new IllegalArgumentException("Invalid diagonal direction! Use FR, FL, BR, or BL.");
        }
    
        // Calculate time for trapezoidal motion profile
        double v_squared = Math.pow(autonParameters.STRAIGHT_MAX_SPEED, 2);
        double triangle_crossover_distance = 
            v_squared / (2 * autonParameters.STRAIGHT_ACCELERATION_RATE) +
            v_squared / (2 * autonParameters.STRAIGHT_DECELERATION_RATE);
        double absDistanceTarget = Math.abs(_distanceTarget);
        double required_time;
    
        if (absDistanceTarget <= triangle_crossover_distance) { // Triangular profile
            double new_velocity = Math.sqrt(
                (2 * absDistanceTarget * autonParameters.STRAIGHT_ACCELERATION_RATE * autonParameters.STRAIGHT_DECELERATION_RATE) /
                (autonParameters.STRAIGHT_DECELERATION_RATE + autonParameters.STRAIGHT_ACCELERATION_RATE)
            );
            required_time =
                new_velocity / autonParameters.STRAIGHT_ACCELERATION_RATE +
                new_velocity / autonParameters.STRAIGHT_DECELERATION_RATE;
        } else { // Trapezoidal profile
            required_time =
                autonParameters.STRAIGHT_MAX_SPEED / autonParameters.STRAIGHT_ACCELERATION_RATE +
                autonParameters.STRAIGHT_MAX_SPEED / autonParameters.STRAIGHT_DECELERATION_RATE +
                (absDistanceTarget - triangle_crossover_distance) / autonParameters.STRAIGHT_MAX_SPEED;
        }
    
        // Perform diagonal movement
        profiledDriveDiagonal(_distanceTarget, required_time * autonParameters.STRAIGHT_TIMEOUT_MULTIPLIER, diagonalDirection);
    }
    
    /**
     * WARNING: Internal use function. Allows manually setting timeout for diagonal motion.
     *
     * @param _distanceTarget The diagonal distance (in meters) to drive.
     * @param _timeout The timeout in seconds.
     * @param diagonalDirection The diagonal direction ("FR", "FL", "BR", or "BL").
     */
    private void profiledDriveDiagonal(double _distanceTarget, double _timeout, String diagonalDirection) {
        ElapsedTime movementTime = new ElapsedTime();
    
        // Define diagonal-specific tolerances
        double maxStoppedSpeed = 0.01; // meters/sec
        double maxFinalError = 0.005; // meters
    
        // Ensure the opmode is still active
        if (opMode.opModeIsActive()) {
            drivetrain.stopMotorsAndReset();
            drivetrain.setZeroPowerBehavior(true);
        
            // Update the gyro and capture the initial heading
            drivetrain.updateHeading();
            double startHeading = drivetrain.getHeading();
        
            movementTime.reset();
            boolean cont = true;
        
            while (opMode.opModeIsActive() && movementTime.seconds() < _timeout && cont) {
                drivetrain.updateHeading();
                double angleError = drivetrain.getHeading() - startHeading;
            
                double currentDistance = drivetrain.getCurrentAvgStraightPosition();
                double currentVelocity = drivetrain.getCurrentAvgStraightVelocity();
            
                double r = Math.signum(_distanceTarget - currentDistance);
                double targetVelocity, velocityError, deliveredPower;
            
                // Simplified acceleration, cruise, and deceleration logic
                targetVelocity = Math.sqrt(2 * autonParameters.STRAIGHT_DECELERATION_RATE * Math.abs(_distanceTarget - currentDistance));
                velocityError = (targetVelocity * r) - currentVelocity;
                deliveredPower = r * (
                    autonParameters.STRAIGHT_GAINS.kS +
                    autonParameters.STRAIGHT_GAINS.kV * targetVelocity +
                    autonParameters.STRAIGHT_GAINS.kE * Math.pow(targetVelocity, 2)
                ) + autonParameters.STRAIGHT_GAINS.kP * velocityError;
            
                // Determine motor power distribution based on diagonal direction
                double frontLeftPower = 0, frontRightPower = 0, rearLeftPower = 0, rearRightPower = 0;
                switch (diagonalDirection) {
                    case "FR": // Forward-Right
                        frontLeftPower = deliveredPower;
                        rearRightPower = deliveredPower;
                        break;
                    case "FL": // Forward-Left
                        frontRightPower = deliveredPower;
                        rearLeftPower = deliveredPower;
                        break;
                    case "BR": // Backward-Right
                        frontLeftPower = -deliveredPower;
                        rearRightPower = -deliveredPower;
                        break;
                    case "BL": // Backward-Left
                        frontRightPower = -deliveredPower;
                        rearLeftPower = -deliveredPower;
                        break;
                }
            
                // Apply motor powers
                drivetrain.runMotorsIndividual(
                    frontLeftPower, frontRightPower,
                    rearLeftPower, rearRightPower
                );
            
                opMode.telemetry.addData("Diagonal Direction", diagonalDirection);
                opMode.telemetry.addData("Distance (m)", currentDistance);
                opMode.telemetry.addData("Velocity (m/s)", currentVelocity);
                opMode.telemetry.update();
            }
        
            drivetrain.stopMotors();
        }
    }
    
    /**
     * Helper function to validate the diagonal direction.
     */
    private boolean isValidDiagonalDirection(String diagonalDirection) {
        return diagonalDirection.equals("FR") || diagonalDirection.equals("FL") ||
               diagonalDirection.equals("BR") || diagonalDirection.equals("BL");
    }
    
    /**
     * Run a characterization procedure in the forward/reverse directions and log all data to a file on the controller.
     */
    public void characterizeStraight(double _movementDistance) {
        double powerStepSize = 0.05;
        double stallTimeoutTime = 4.0; //seconds, amount of time the robot will attempt to move before giving up if the power is too low to move
        double distanceThreshold = 0.020; //meters, amount of displacement required for the robot to be considered "moving"

        //set up a file writer
        Writer fileWriter;

        try {
            String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
            File directory = new File(directoryPath);
            directory.mkdir();
            fileWriter = new FileWriter(directoryPath + "/straight.characterization", false);
        }
        catch (IOException e) {
            opMode.telemetry.addLine("Characterization file failed to open.");
            opMode.telemetry.update();
            opMode.sleep(3000);
            return;
        }

        try {
            fileWriter.write("Power,Time,Distance,Velocity\n");
        }
        catch (IOException e) {
            opMode.telemetry.addLine("First line write failed.");
            opMode.telemetry.update();
            opMode.sleep(3000);
            return;
        }

        ElapsedTime movementTime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            
            //Reset encoders
            drivetrain.stopMotorsAndReset();

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            //Update the gyro and capture an offset for our starting heading
            drivetrain.updateHeading();
            double startHeading = drivetrain.getHeading();

            for (double p = powerStepSize; p <= 1.0; p += powerStepSize) {
                // keep looping while we are still active
                boolean cont = true;
                double direction = 1.0;

                // reset the timeout time and start motion.
                movementTime.reset();
            
                while (opMode.opModeIsActive() && cont) {
                
                    //UPDATE THE GYRO INTEGRATION
                    drivetrain.updateHeading();
                    double angleError = drivetrain.getHeading() - startHeading; //calculate a heading relative to the start orientation (positive is left)

                    double currentDistance = drivetrain.getCurrentAvgStraightPosition(); //average of all encoder positions converted to meters
                    double currentVelocity = drivetrain.getCurrentAvgStraightVelocity(); //average of all encoder velocities converted to m/sec
                    double currentDrift = drivetrain.getCurrentAvgSidewaysPosition(); //average of all encoder positions in the sideways direction converted to meters

                    if (currentDistance >= _movementDistance) {direction = -1.0;}
                    if (direction < 0.0 && currentDistance <= 0.0) {cont = false;}
                    if (movementTime.seconds() > stallTimeoutTime && currentDistance < distanceThreshold) {cont = false;}

                    double deliveredPower = p * direction;

                    double turningPower = autonParameters.HEADING_GAINS.kP * angleError; //this will be added to the left side and subtacted from the right
                    double driftPower = autonParameters.POSITION_GAINS.kP * (-currentDrift); //this will counteract lateral drift

                    drivetrain.runMotorsIndividual(
                            deliveredPower + turningPower + driftPower,
                            deliveredPower - turningPower - driftPower,
                            deliveredPower + turningPower - driftPower,
                            deliveredPower - turningPower + driftPower);

                    opMode.telemetry.addData("Delivered Power (%)", deliveredPower);
                    opMode.telemetry.addData("Elapsed Time (sec)", movementTime.seconds());
                    opMode.telemetry.addData("Distance (m)", currentDistance);
                    opMode.telemetry.addData("Velocity (m/sec)", currentVelocity);
                    opMode.telemetry.update();
                    
                    //Write data to the data file
                    try {
                        fileWriter.write(
                            String.format("%.4f",deliveredPower) + "," + 
                            String.format("%.4f",movementTime.seconds()) + "," + 
                            String.format("%.4f",currentDistance) + "," + 
                            String.format("%.4f",currentVelocity) + "," + 
                            "\n");
                    }
                    catch (IOException e) {
                        opMode.telemetry.addLine("Writing to the characterization file failed during execution.");
                        opMode.telemetry.update();
                        opMode.sleep(3000);
                        return;
                    }
                }
            }
            
            //Close the data file
            try {
                fileWriter.close();
            }
            catch (IOException e) {
                opMode.telemetry.addLine("Characterization file failed to close.");
                opMode.telemetry.update();
                opMode.sleep(3000);
            }

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            // Stop all motion;
            drivetrain.stopMotors();
        }
    }

    /**
     * Run a characterization procedure in the left/right directions and log all data to a file on the controller.
     */
    public void characterizeSideways(double _movementDistance) {
        double powerStepSize = 0.05;
        double stallTimeoutTime = 4.0; //seconds, amount of time the robot will attempt to move before giving up if the power is too low to move
        double distanceThreshold = 0.020; //meters, amount of displacement required for the robot to be considered "moving"

        //set up a file writer
        Writer fileWriter;

        try {
            String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
            File directory = new File(directoryPath);
            directory.mkdir();
            fileWriter = new FileWriter(directoryPath + "/sideways.characterization", false);
        }
        catch (IOException e) {
            opMode.telemetry.addLine("Characterization file failed to open.");
            opMode.telemetry.update();
            opMode.sleep(3000);
            return;
        }

        try {
            fileWriter.write("Power,Time,Distance,Velocity\n");
        }
        catch (IOException e) {
            opMode.telemetry.addLine("First line write failed.");
            opMode.telemetry.update();
            opMode.sleep(3000);
            return;
        }

        ElapsedTime movementTime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            
            //Reset encoders
            drivetrain.stopMotorsAndReset();

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            //Update the gyro and capture an offset for our starting heading
            drivetrain.updateHeading();
            double startHeading = drivetrain.getHeading();

            for (double p = powerStepSize; p <= 1.0; p += powerStepSize) {
                // keep looping while we are still active
                boolean cont = true;
                double direction = 1.0;

                // reset the timeout time and start motion.
                movementTime.reset();
            
                while (opMode.opModeIsActive() && cont) {
                
                    //UPDATE THE GYRO INTEGRATION
                    drivetrain.updateHeading();
                    double angleError = drivetrain.getHeading() - startHeading; //calculate a heading relative to the start orientation (positive is left)

                    double currentDistance = drivetrain.getCurrentAvgSidewaysPosition(); //average of all encoder positions converted to meters
                    double currentVelocity = drivetrain.getCurrentAvgSidewaysVelocity(); //average of all encoder velocities converted to m/sec
                    double currentDrift = drivetrain.getCurrentAvgStraightPosition(); //average of all encoder positions in the forward/reverse direction converted to meters

                    if (currentDistance >= _movementDistance) {direction = -1.0;}
                    if (direction < 0.0 && currentDistance <= 0.0) {cont = false;}
                    if (movementTime.seconds() > stallTimeoutTime && currentDistance < distanceThreshold) {cont = false;}

                    double deliveredPower = p * direction;

                    double turningPower = autonParameters.HEADING_GAINS.kP * angleError; //this will be added to the left side and subtacted from the right
                    double driftPower = autonParameters.POSITION_GAINS.kP * (-currentDrift); //this will counteract forward/reverse drift

                    drivetrain.runMotorsIndividual(
                              deliveredPower + turningPower + driftPower,
                            - deliveredPower - turningPower + driftPower,
                            - deliveredPower + turningPower + driftPower,
                              deliveredPower - turningPower + driftPower);

                    opMode.telemetry.addData("Delivered Power (%)", deliveredPower);
                    opMode.telemetry.addData("Elapsed Time (sec)", movementTime.seconds());
                    opMode.telemetry.addData("Distance (m)", currentDistance);
                    opMode.telemetry.addData("Velocity (m/sec)", currentVelocity);
                    opMode.telemetry.update();
                    
                    //Write data to the data file
                    try {
                        fileWriter.write(
                            String.format("%.4f",deliveredPower) + "," + 
                            String.format("%.4f",movementTime.seconds()) + "," + 
                            String.format("%.4f",currentDistance) + "," + 
                            String.format("%.4f",currentVelocity) + "," + 
                            "\n");
                    }
                    catch (IOException e) {
                        opMode.telemetry.addLine("Writing to the characterization file failed during execution.");
                        opMode.telemetry.update();
                        opMode.sleep(3000);
                        return;
                    }
                }
            }
            
            //Close the data file
            try {
                fileWriter.close();
            }
            catch (IOException e) {
                opMode.telemetry.addLine("Characterization file failed to close.");
                opMode.telemetry.update();
                opMode.sleep(3000);
            }

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            // Stop all motion;
            drivetrain.stopMotors();
        }
    }

    /**
     * Run a characterization procedure on the rotation of the robot and log all data to a file on the controller.
     */
    public void characterizeTurning(double _turnAngle) {
        double powerStepSize = 0.05;
        double stallTimeoutTime = 3.0; //seconds, amount of time the robot will attempt to move before giving up if the power is too low to move
        double angleThreshold = 2.0; //degrees, amount of displacement required for the robot to be considered "moving"

        //set up a file writer
        Writer fileWriter;

        try {
            String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+"FIRST";
            File directory = new File(directoryPath);
            directory.mkdir();
            fileWriter = new FileWriter(directoryPath + "/turning.characterization", false);
        }
        catch (IOException e) {
            opMode.telemetry.addLine("Characterization file failed to open.");
            opMode.telemetry.update();
            opMode.sleep(3000);
            return;
        }

        try {
            fileWriter.write("Power,Time,Angle,Angular Velocity\n");
        }
        catch (IOException e) {
            opMode.telemetry.addLine("First line write failed.");
            opMode.telemetry.update();
            opMode.sleep(3000);
            return;
        }

        ElapsedTime movementTime = new ElapsedTime();

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            
            //Reset encoders
            drivetrain.stopMotorsAndReset();

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            //Update the gyro and capture an offset for our starting heading
            drivetrain.updateHeading();
            double startHeading = drivetrain.getHeading();

            for (double p = powerStepSize; p <= 1.0; p += powerStepSize) {
                // keep looping while we are still active
                boolean cont = true;
                double direction = 1.0;

                // reset the timeout time and start motion.
                movementTime.reset();
            
                while (opMode.opModeIsActive() && cont) {
                
                    //UPDATE THE GYRO INTEGRATION
                    drivetrain.updateHeading();
                    double currentAngle = drivetrain.getHeading() - startHeading; //calculate a heading relative to the start orientation (positive is left)

                    double currentPosition = drivetrain.getCurrentAvgStraightPosition(); //average of all encoder positions converted to meters
                    double currentAngVelocity = drivetrain.getRotationRate(); //average of all encoder velocities converted to m/sec
                    double currentDrift = drivetrain.getCurrentAvgSidewaysPosition(); //average of all encoder positions in the sideways direction converted to meters

                    if (currentAngle >= _turnAngle) {direction = -1.0;}
                    if (direction < 0.0 && currentAngle <= 0.0) {cont = false;}
                    if (movementTime.seconds() > stallTimeoutTime && currentAngle < angleThreshold) {cont = false;}

                    double deliveredPower = p * direction;

                    double centeringPower = autonParameters.POSITION_GAINS.kP * (-currentPosition); //this will be added to the left side and subtacted from the right
                    double driftPower = autonParameters.POSITION_GAINS.kP * (-currentDrift); //this will counteract lateral drift

                    drivetrain.runMotorsIndividual(
                            - deliveredPower + centeringPower + driftPower,
                              deliveredPower + centeringPower - driftPower,
                            - deliveredPower + centeringPower - driftPower,
                              deliveredPower + centeringPower + driftPower);

                    opMode.telemetry.addData("Delivered Power (%)", deliveredPower);
                    opMode.telemetry.addData("Elapsed Time (sec)", movementTime.seconds());
                    opMode.telemetry.addData("Angle (deg)", currentAngle);
                    opMode.telemetry.addData("Angular Velocity (deg/sec)", currentAngVelocity);
                    opMode.telemetry.update();
                    
                    //Write data to the data file
                    try {
                        fileWriter.write(
                            String.format("%.4f",deliveredPower) + "," + 
                            String.format("%.4f",movementTime.seconds()) + "," + 
                            String.format("%.4f",currentAngle) + "," + 
                            String.format("%.4f",currentAngVelocity) + "," + 
                            "\n");
                    }
                    catch (IOException e) {
                        opMode.telemetry.addLine("Writing to the characterization file failed during execution.");
                        opMode.telemetry.update();
                        opMode.sleep(3000);
                        return;
                    }
                }
            }
            
            //Close the data file
            try {
                fileWriter.close();
            }
            catch (IOException e) {
                opMode.telemetry.addLine("Characterization file failed to close.");
                opMode.telemetry.update();
                opMode.sleep(3000);
            }

            //Switch to brake mode
            drivetrain.setZeroPowerBehavior(true);

            // Stop all motion;
            drivetrain.stopMotors();
        }
    }

}
