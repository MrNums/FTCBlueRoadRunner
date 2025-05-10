package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous(name = "4 Basket Auto", group = "Autonomous")
public class Basket extends LinearOpMode {

    private SparkFunOTOSDrive drive; // Declare drive as an instance variable
    private ArmMech armMech;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-39, -62, Math.toRadians(-90));
        drive = new SparkFunOTOSDrive(hardwareMap, initialPose); // Initialize drive
        armMech = new ArmMech(hardwareMap); // Initialize armMech

        waitForStart();

        executeAutonomous();
    }

    private void executeAutonomous() {
        // Create initial trajectory
        TrajectoryActionBuilder path1 = basket1();
        // Create second trajectory starting where path1 ended
        TrajectoryActionBuilder path2 = score2get(path1);

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                path1.build(),
                                armMech.prepareToScore()
                        ),
                        armMech.score(),
                        new ParallelAction(
                                armMech.prepareToScore(),
                                path2.build()
                                ),
                        armMech.Home()
                )
        );
    }

    Pose2d path1End = new Pose2d(-50.9, -45.4, Math.toRadians(-45));

    private TrajectoryActionBuilder basket1() {
        return drive.actionBuilder(new Pose2d(-39, -62, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-50.9, -45.4))
                .turn(Math.toRadians(45));
    }

    private TrajectoryActionBuilder score2get(TrajectoryActionBuilder previousPath) {
        return previousPath.endTrajectory().fresh()
                .strafeTo(new Vector2d(-34, -35))
                .turn(Math.toRadians(100));
    }
}


//   private TrajectoryActionBuilder basket2score(TrajectoryActionBuilder previousPath) {
 //       return previousPath.endTrajectory().fresh()
 //               .strafeTo(new Vector2d(-60, -58))
 //               .turn(Math.toRadians(-100));
 //   }

  //  private TrajectoryActionBuilder basket3get(TrajectoryActionBuilder previousPath) {
   //     return previousPath.endTrajectory().fresh()
        //        .strafeTo(new Vector2d(-43, -31))
  //              .turn(Math.toRadians(100));
 //   }

   // private TrajectoryActionBuilder basket3score(TrajectoryActionBuilder previousPath) {
     //   return previousPath.endTrajectory().fresh()
       //         .strafeTo(new Vector2d(-60, -58))
         //       .turn(Math.toRadians(-100));
   // }

    //private TrajectoryActionBuilder basket4get(TrajectoryActionBuilder previousPath) {
      //  return previousPath.endTrajectory().fresh()
        //        .strafeTo(new Vector2d(-47, -26))
          //      .turn(Math.toRadians(130));
    //}

    //private TrajectoryActionBuilder basket4score(TrajectoryActionBuilder previousPath) {
      //  return previousPath.endTrajectory().fresh()
        //        .strafeTo(new Vector2d(-60, -58))
          //      .turn(Math.toRadians(-130));
    //}
//}
