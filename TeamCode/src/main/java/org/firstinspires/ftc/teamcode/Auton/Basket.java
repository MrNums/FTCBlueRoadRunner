package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name = "4 Basket Auto", group = "Autonomous")
public class Basket extends LinearOpMode {

    private MecanumDrive drive; // Declare drive as an instance variable

    private ArmMech armMech;




    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-39, -62, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, initialPose); // Initialize drive

        ArmMech arm = new ArmMech(hardwareMap);

        armMech = new ArmMech(hardwareMap); // Change this line




        waitForStart();

        executeAutonomous();
    }

    private void executeAutonomous() {
        // Define trajectories
        TrajectoryActionBuilder path1 = basket1();
  //      TrajectoryActionBuilder path2 = basket2get(path1);
    //    TrajectoryActionBuilder path3 = basket2score(path2);
      //  TrajectoryActionBuilder path4 = basket3get(path3);
        //TrajectoryActionBuilder path5 = basket3score(path4);
        //TrajectoryActionBuilder path6 = basket4get(path5);
        //TrajectoryActionBuilder path7 = basket4score(path6);

        Actions.runBlocking(
                new SequentialAction(
                        path1.build(),
                        new ParallelAction(
                        armMech.prepareToScore(),
                        armMech.score()
                        )

          //              path2.build(),
            //            path3.build(),
              //          path4.build(),
                //        path5.build(),
                  //      path6.build(),
                    //    path7.build()
                )
        );
    }

    private TrajectoryActionBuilder basket1() {
        return drive.actionBuilder(new Pose2d(-39, -62, Math.toRadians(90)))
                .strafeTo(new Vector2d(-50.9, -51.4))
                .turn(Math.toRadians(-45));
    }

  //  private TrajectoryActionBuilder basket2get(TrajectoryActionBuilder previousPath) {
  //      return previousPath.endTrajectory().fresh()
  //              .strafeTo(new Vector2d(-34, -35))
  //              .turn(Math.toRadians(100));
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
