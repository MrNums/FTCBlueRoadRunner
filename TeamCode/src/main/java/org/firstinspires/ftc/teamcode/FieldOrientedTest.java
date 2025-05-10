package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robochargers.MecanumDrivetrain;

@TeleOp(name="Field Oriented Test", group="Iterative Opmode")

public class FieldOrientedTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain drivetrain;

    private DcMotorEx shoulder, elbow, specimen, intake;
    private CRServo specLServo, specRServo;

    private boolean lastLeftButton = false;
    private boolean lastRightButton = false;
    private boolean lastXButton = false;
    private boolean lastOptionButton = false;

    int shoulderTarget = 0;
    int elbowTarget = 0;

    enum SpecimenState {
        HOME,
        WALL,
        WALL_LIFT,
        LOW_CLIP,
        LOW_EJECT,
        HIGH,
        HIGH_CLIP,
        HIGH_EJECT
    }

    SpecimenState specimenState = SpecimenState.HOME;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        drivetrain = new MecanumDrivetrain(hardwareMap, DriveParametersTemplate.class);
        
        // shoulder init
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        shoulder.setDirection(TeleOpParameters.SHOULDER_DIRECTION);
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setTargetPosition(0);
        shoulder.setPower(1.0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // elbow init
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow.setDirection(TeleOpParameters.ELBOW_DIRECTION);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(0);
        elbow.setPower(1.0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // specimen init
        specimen = hardwareMap.get(DcMotorEx.class, "spec");
        specimen.setDirection(TeleOpParameters.SPECIMEN_DIRECTION);
        specimen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimen.setTargetPosition(0);
        specimen.setPower(1.0);
        specimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        // intake init
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(TeleOpParameters.INTAKE_DIRECTION);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
  
          // Servo Initialization
          specLServo = hardwareMap.get(CRServo.class, "specLServo");
          specRServo = hardwareMap.get(CRServo.class, "specRServo");

          // TeleOpParameters to Configure Motors & Servos

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
    @Override
    public void loop() {
        drivetrain.updateHeading();

         drivetrain.runMotorsFieldOriented(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
        // drivetrain.runMotorsMixed(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);

        if (gamepad1.right_stick_button){
            drivetrain.zeroHeading();
        }
        
        //Arm Target Postions
        int shoulderStartPos = 0;
        int shoulderIntakePos = 450;
        int shoulderScoreHighPos = 2000;
        int shoulderHoverPos = 450;
        int elbowStartPos = 0;
        int elbowHoverPos = 1050;
        int elbowScoreHighPos = 0;
        int elbowIntakePos = 1300;

        //Test
        int elbowOuttakePos = -1500;
        int shoulderOuttakePos = 2200;

        int specimenHome = 0;
        int specimenWall = 50;
        int specimenWallLift = 200;
        int specimenLowClip = 0;
        int specimenLowEject = 0;
        int specimenHigh = 1100;
        int specimenHighClip = 820;
        int specimenHighEject = 820;
        int specimenTolerance = 10;

        // Shoulder & Elbow Controll
        if (gamepad1.a){
            shoulderTarget = shoulderStartPos;
            elbowTarget = elbowStartPos;
        }
        
        if (shoulderTarget == shoulderHoverPos
        && elbowTarget == elbowHoverPos
        && gamepad1.right_trigger > 0.5){
            shoulderTarget = shoulderIntakePos;
            elbowTarget = elbowIntakePos; 
        }
        if (shoulderTarget == shoulderIntakePos
        && elbowTarget == elbowIntakePos
        && gamepad1.right_trigger < 0.5){
            shoulderTarget = shoulderHoverPos;
            elbowTarget = elbowHoverPos;
        }
        
        if (gamepad1.y){
            shoulderTarget = shoulderScoreHighPos;
            elbowTarget = elbowScoreHighPos;
        }
        
        if (gamepad1.b){
            shoulderTarget = shoulderHoverPos;
            elbowTarget = elbowHoverPos;
        }
        if (gamepad1.touchpad && !lastOptionButton){
            elbowTarget = elbowOuttakePos;
            shoulderTarget = shoulderOuttakePos;
        }
        
        // Servo Intake
        if (gamepad1.right_trigger > 0.5){  /// Intake
            intake.setPower(1.0);
        }
        else if (gamepad1.left_trigger > 0.5){  // Outtake
            intake.setPower(-1.0);
        }
        else if(shoulderTarget == shoulderStartPos &&
        elbowTarget == elbowStartPos){
            intake.setPower(0.0);
        }
        else {
            intake.setPower(0.2);
        }

        // Specimen State Controls 
        if (specimenState == SpecimenState.HOME) {
            specimen.setTargetPosition(specimenHome);
            specLServo.setPower(0.0);
            specRServo.setPower(0.0);

            if (gamepad1.right_bumper && !lastRightButton){
                specimenState = SpecimenState.WALL;
            }
        }    
        else if (specimenState == SpecimenState.WALL){
            specimen.setTargetPosition(specimenWall);
            specLServo.setPower(1.0);
            specRServo.setPower(-1.0);

            if (gamepad1.left_stick_button){
                specimenState = SpecimenState.HOME;
            }
            if (gamepad1.right_bumper && !lastRightButton){
                specimenState = SpecimenState.WALL_LIFT;
            }
        }
        else if (specimenState == SpecimenState.WALL_LIFT){
            specimen.setTargetPosition(specimenWallLift);
            specLServo.setPower(0.0);
            specRServo.setPower(0.0);

            if (gamepad1.left_stick_button){
                specimenState = SpecimenState.HOME;
            }
            if (gamepad1.right_bumper && !lastRightButton){
                specimenState = SpecimenState.HIGH;
            }
            if (gamepad1.left_bumper && !lastLeftButton){
                specimenState = SpecimenState.WALL;
            }
            if (gamepad1.x && !lastXButton){
                specimenState = SpecimenState.LOW_CLIP;
            }
        }
        else if (specimenState == SpecimenState.LOW_CLIP){
            specimen.setTargetPosition(specimenLowClip);
            specLServo.setPower(0.0);
            specRServo.setPower(0.0);

            if (Math.abs(specimen.getCurrentPosition() - specimenLowClip) <= specimenTolerance){
                specimenState = SpecimenState.LOW_EJECT;
            }
            if (gamepad1.left_stick_button){
                specimenState = SpecimenState.HOME;
            }
        }
        else if (specimenState == SpecimenState.LOW_EJECT){
            specimen.setTargetPosition(specimenLowEject);
            specLServo.setPower(-1.0);
            specRServo.setPower(1.0);

            if (gamepad1.left_stick_button){
                specimenState = SpecimenState.HOME;
            }
            if (gamepad1.left_bumper && !lastLeftButton){
                specimenState = SpecimenState.WALL;
            }
        }
        else if (specimenState == SpecimenState.HIGH){
            specimen.setTargetPosition(specimenHigh);
            specLServo.setPower(0.0);
            specRServo.setPower(0.0);

            if (gamepad1.left_stick_button){
                specimenState = SpecimenState.HOME;
            }
            if (gamepad1.x && !lastXButton){
                specimenState = SpecimenState.HIGH_CLIP;
            }
            if (gamepad1.left_bumper && !lastLeftButton){
                specimenState = SpecimenState.WALL_LIFT;
            }
        }
        else if (specimenState == SpecimenState.HIGH_CLIP){
            specimen.setTargetPosition(specimenHighClip);
            specLServo.setPower(0.0);
            specRServo.setPower(0.0);

            if (Math.abs(specimen.getCurrentPosition() - specimenHighClip) <= specimenTolerance){
                specimenState = SpecimenState.HIGH_EJECT;
            }
            if (gamepad1.left_stick_button){
                specimenState = SpecimenState.HOME;
            }
        }
        else if (specimenState == SpecimenState.HIGH_EJECT){
            specLServo.setPower(-1.0);
            specRServo.setPower(1.0);
            specimen.setTargetPosition(specimenHighEject);
            specLServo.setPower(-1.0);
            specRServo.setPower(1.0);

            if (gamepad1.left_stick_button){
                specimenState = SpecimenState.HOME;
            }
            if (gamepad1.left_bumper && !lastLeftButton){
                specimenState = SpecimenState.WALL;
            }
        }

        shoulder.setTargetPosition(shoulderTarget);
        elbow.setTargetPosition(elbowTarget);
        
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gyro Orientation", drivetrain.getHeading());
        telemetry.addData("Drive Joystick Raw", gamepad1.left_stick_y);
        telemetry.addData("Strafe Joystick Raw", gamepad1.left_stick_x);
        telemetry.addData("Rotate Joystick Raw", gamepad1.right_stick_x);
        telemetry.addData("Shoulder Pos", shoulder.getCurrentPosition());
        telemetry.addData("Elbow Pos", elbow.getCurrentPosition());
        telemetry.addData("Specimen Pos", specimen.getCurrentPosition());

        telemetry.update();

        lastLeftButton = gamepad1.left_bumper; 
        lastRightButton = gamepad1.right_bumper;
        lastXButton = gamepad1.x;
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

}