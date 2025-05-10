package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous(name = "Spec Auto", group = "Autonomous")
public class Spec3 extends LinearOpMode {

    private MecanumDrive drive; // Declare drive as an instance variable
    private DcMotorEx specimen;
    private CRServo specLServo, specRServo;
    private SpecMech specMech;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, -62, Math.toRadians(90));
        drive = new SparkFunOTOSDrive(hardwareMap, initialPose); // Initialize drive
        specimen = hardwareMap.get(DcMotorEx.class, "spec");
        specMech = new SpecMech(hardwareMap); // Initialize SpecMech
        specLServo = hardwareMap.get(CRServo.class, "specLServo");
        specRServo = hardwareMap.get(CRServo.class, "specRServo");

        waitForStart();

        if (isStopRequested()) return; // Check for stop request

        executeAutonomous();
    }

    private void executeAutonomous() {
        // Define trajectories
        TrajectoryActionBuilder path1 = Clipping1();
        TrajectoryActionBuilder path2 = PushingIn(path1);
        //   TrajectoryActionBuilder path3 = Clipping2(path2);
        // TrajectoryActionBuilder path4 = pickingup2(path3);

        // Run the actions in sequence
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                specMech.moveToHigh(), // Score high after the second position
                                path1.build()
                        ),
                        specMech.clipAndRelease(),
                        new ParallelAction(
                                specMech.prepIntake(),
                                path2.build()
                        ),
                        path2.build()
                        //               path3.build(),
                        ///             path4.build(),
                )
        );
    }


     TrajectoryActionBuilder Clipping1() {
        return drive.actionBuilder(new Pose2d(0, -62, Math.toRadians(90)))
                .strafeTo(new Vector2d(0, -30)); // Clipped the first one
    }

     TrajectoryActionBuilder PushingIn(TrajectoryActionBuilder previousPath) {
        return previousPath.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, -40)) // Started pushing 2
                .waitSeconds(.5)
                .strafeTo(new Vector2d(35, -40));
    }
}
       /*         .strafeTo(new Vector2d(35, -9))
                .strafeTo(new Vector2d(45, -9))
                .strafeTo(new Vector2d(45, -48))
                .splineToLinearHeading(new Pose2d(56, -9, Math.toRadians(0)), 0)
                .strafeTo(new Vector2d(56, -48))
                .strafeTo(new Vector2d(40, -48))
                .turn(Math.toRadians(-90)); // Finished pushing 2 in and ready to start clipping
    }

    private TrajectoryActionBuilder Clipping2(TrajectoryActionBuilder previousPath) {
        return previousPath.endTrajectory().fresh()
                .strafeTo(new Vector2d(40, -62)) // Got the first one
                .strafeTo(new Vector2d(0, -50)) // Leaving after getting the specimen
                .turn(Math.toRadians(180))
                .strafeTo(new Vector2d(0, -32)); // At the bar to score the specimen
    }

    private TrajectoryActionBuilder pickingup2(TrajectoryActionBuilder previousPath) {
        return previousPath.endTrajectory().fresh()
                .strafeTo(new Vector2d(40, -62)); // Picking up a second one
    }
}
*/