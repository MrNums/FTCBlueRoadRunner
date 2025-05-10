package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robochargers.MecanumAuton;
import org.firstinspires.ftc.teamcode.robochargers.MecanumDrivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
* The robot will start at the edge of the basket / Net Zone
* The robot will wait 24 seconds, then proceed to park on the right side of the observation zone.
 */

@Autonomous(name="Blue Park Far", group="Linear Opmode")

public class BlueParkFar extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();
    private MecanumDrivetrain drivetrain;
    private MecanumAuton drivetrainAuto;

    @Override
    public void runOpMode() {
        
        drivetrain = new MecanumDrivetrain(hardwareMap, DriveParametersTemplate.class);
        drivetrainAuto = new MecanumAuton(this, drivetrain, AutonParametersTemplate.class);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
        drivetrainAuto.profiledDriveStraight(1.5);      FOWARD  1.5 METERS
        drivetrainAuto.profiledDriveStraight(-1.5);     REVERSE 1.5 METERS
        drivetrainAuto.profiledTurnHeading(90.0);       TURN LEFT 90 DEGRESS    (Postive = CounterClockWise, Negative = ClockWise)    
        drivetrainAuto.profiledDriveSideways(-1.5);     STRAFE LEFT 1.5 METERS  (Negative = Left, Positive = Right?)
        sleep(1000);                                    WAITING/STOPED 1 SECOND (Formula: TimeInSeconds * 1000 = TimeInMilliseconds)
        
        */

        sleep(24000);
        drivetrainAuto.profiledDriveStraight(0.2);
        drivetrainAuto.profiledDriveSideways(2.0);

    }
}
