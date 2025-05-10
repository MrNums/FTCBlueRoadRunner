package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robochargers.MecanumAuton;
import org.firstinspires.ftc.teamcode.robochargers.MecanumDrivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Advance V2", group="Linear Opmode")      // This is the Displat Name on the Driver Hub 

public class BlueAdvanceV2 extends LinearOpMode {              // Make sure to rename the file to match the file name.

/* Declare OpMode members. */
  
private ElapsedTime     runtime = new ElapsedTime();
    private MecanumDrivetrain drivetrain;
    private MecanumAuton drivetrainAuto;
    private DcMotorEx shoulder = null;
    private DcMotorEx elbow = null;
    private DcMotorEx specimen = null;
    private DcMotorEx intake = null;
    private CRServo specLServo =  null;
    private CRServo specRServo = null;

    public enum intakeMode {
        INTAKING,
        OUTAKING
    }    

    public enum specMode {
        HIGH,
        LOW,
        INTAKE,
        HOME
    }

    public enum armMode {
        HOME,
        HOVER,
        INTAKE,
        HIGH
    }

        // Global current modes
        private intakeMode currentIntakeMode = intakeMode.INTAKING;
        private specMode currentSpecMode = specMode.HOME;
        private armMode currentArmMode = armMode.HOME;
    
        // Setters for Current Modes
        public void setIntakeMode(intakeMode mode) {
            currentIntakeMode = mode;
        }
    
        public void setSpecMode(specMode mode) {
            currentSpecMode = mode;
        }
    
        public void setArmMode(armMode mode) {
            currentArmMode = mode;
        }

    // INTAKE FUNCTIONS 
    public void runIntake(long timeInMillis) {
        intake.setPower(1.0);
        try {
            Thread.sleep(timeInMillis); // Pause the program for the given time
        } catch (InterruptedException e) {
            e.printStackTrace(); // Handle potential interruptions
        }
        intake.setPower(0); // Stop the intake
    }
    public void runOuttake(long timeInMillis) {
        intake.setPower(-0.5); // Start the intake
        try {
            Thread.sleep(timeInMillis); // Pause the program for the given time
        } catch (InterruptedException e) {
            e.printStackTrace(); // Handle potential interruptions
        }
        intake.setPower(0); // Stop the intake
    }


    public void runIntakeMovement(double distance, long timeInMillis, intakeMode mode) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
    
        // Set intake or outtake power based on the mode
        if (mode == intakeMode.INTAKING) {
            intake.setPower(1.0); // Intake power
            runArmToPos(armMode.INTAKE);
        } else if (mode == intakeMode.OUTAKING) {
            intake.setPower(-0.5); // Outtake power
            runArmToPos(armMode.HOVER);
        }
    
        // Start drivetrain movement
        drivetrainAuto.profiledDriveStraight(distance);
    
        // Allow the intake/outtake to run for the specified time
        while (opModeIsActive() && timer.milliseconds() < timeInMillis) {
            // Allow drivetrain and other operations to continue
        }
    
        // Stop the intake or outtake after the specified time
        intake.setPower(0.2);
        runArmToPos(armMode.HOVER);
    }
    
    

    public void runHighClip(double drivelenght) {
        specimen.setTargetPosition(specimenHighClip);
        specLServo.setPower(-1.0);   // OUTTAKE
        specRServo.setPower(1.0);
        sleep(750);
        drivetrainAuto.profiledDriveStraight(-drivelenght); 
        specLServo.setPower(0.0); 
        specRServo.setPower(0.0);
    }
    public void runLowClip(double drivelenght) {
        specimen.setTargetPosition(specimenLowClip);
        specLServo.setPower(-1.0);   // OUTTAKE
        specRServo.setPower(1.0);
        sleep(500);
        drivetrainAuto.profiledDriveStraight(-drivelenght); 
        specLServo.setPower(0.0); 
        specRServo.setPower(0.0);
    }

    public void runSpec(specMode mode, double drivelenght) {
        if (mode == specMode.HOME) {
            specimen.setTargetPosition(specimenHome);
            specLServo.setPower(0.0);
            specRServo.setPower(0.0);
        } 
        else if (mode == specMode.INTAKE) {
            specimen.setTargetPosition(specimenWall);
            specLServo.setPower(1.0); // INTAKE
            specRServo.setPower(-1.0);
            drivetrainAuto.profiledDriveStraight(drivelenght);
            sleep(500);
            specLServo.setPower(0.0);
            specRServo.setPower(0.0);
            specimen.setTargetPosition(specimenWallLift);
            drivetrainAuto.profiledDriveStraight(-drivelenght);
        } 
        else if (mode == specMode.LOW) {
            specimen.setTargetPosition(specimenWallLift);
            drivetrainAuto.profiledDriveStraight(drivelenght);
            runLowClip(drivelenght);
            specimen.setTargetPosition(specimenWall);
        }
        else if (mode == specMode.HIGH) {
            specimen.setTargetPosition(specimenHigh);
            drivetrainAuto.profiledDriveStraight(drivelenght);
            runHighClip(drivelenght);
            specimen.setTargetPosition(specimenWall);
        }
    }

    public void runArmToPos(armMode mode) {
        int shoulderTarget;
        int elbowTarget;

        switch (mode) {
            case HOME:
                shoulderTarget = shoulderStartPos;
                elbowTarget = elbowStartPos;
                break;
            case HOVER:
                shoulderTarget = shoulderHoverPos;
                elbowTarget = elbowHoverPos;
                break;
            case INTAKE:
                shoulderTarget = shoulderIntakePos;
                elbowTarget = elbowIntakePos;
                break;
            case HIGH:
                shoulderTarget = shoulderScoreHighPos;
                elbowTarget = elbowScoreHighPos;
                break;
            default:
                throw new IllegalArgumentException("Invalid arm mode");
        }

        shoulder.setTargetPosition(shoulderTarget);
        elbow.setTargetPosition(elbowTarget);
    }
    
    public void driveDiagonal(double distanceForward, double distanceSideways) {
        drivetrainAuto.profiledDriveStraight(distanceForward);
        drivetrainAuto.profiledDriveSideways(distanceSideways);
    }
    
    

        //Arm Target Postions
        int shoulderStartPos = 0;
        int shoulderIntakePos = 400;
        int shoulderScoreHighPos = 2000;
        int shoulderHoverPos = 450;
        int elbowStartPos = 0;
        int elbowHoverPos = 1050;
        int elbowScoreHighPos = 0;
        int elbowIntakePos = 1260;

        int specimenHome = 0;
        int specimenWall = 50;
        int specimenWallLift = 200;
        int specimenLowClip = 0;
        int specimenLowEject = 0;
        int specimenHigh = 1150;
        int specimenHighClip = 820;
        int specimenHighEject = 820;
        int specimenTolerance = 10;

    @Override
    public void runOpMode() {
        
        drivetrain = new MecanumDrivetrain(hardwareMap, DriveParametersTemplate.class);
        drivetrainAuto = new MecanumAuton(this, drivetrain, AutonParametersTemplate.class);
        
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

        // Spec Servos
        specLServo = hardwareMap.get(CRServo.class, "specLServo");
        specRServo = hardwareMap.get(CRServo.class, "specRServo");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*
        drivetrainAuto.profiledDriveStraight(1.5);           // Move FORWARD 1.5 METERS (Negative = REVERSE, Positive = FORWARD)
        drivetrainAuto.profiledTurnHeading(90.0);            // TURN LEFT 90 DEGREES (Positive = CounterClockWise, Negative = ClockWise)
        drivetrainAuto.profiledDriveSideways(-1.5);          // STRAFE LEFT 1.5 METERS (Negative = Left, Positive = Right)
        driveDiagonal(1.0, 0.5);                             // DIAGONAL DRIVE: FORWARD 1.0 METERS, LEFT 0.5 METERS, ABOVE RULES APPLY
   
        sleep(1000);                                         // WAIT/PAUSE for 1 SECOND (Formula: TimeInSeconds * 1000 = TimeInMilliseconds)
        runIntake(2500);                                     // RUNS INTAKE FOR 2.5 SECONDS
        runOuttake(2500);                                    // RUNS OUTTAKE FOR 2.5 SECONDS
        runSpec(specMode.HOME, 1.0);                         // SPECIMEN HOME: Moves forward 1.0 meters and returns to the origin. SPEC MODES: "HIGH, LOW, INTAKE, HOME"
        runIntakeMovement(1.0, 1000, intakeMode.INTAKING);   // Moves to INTAKE POSITION, drives FORWARD 1.0 meters while intake runs for 1 second, returns to hover.
                                                             // INTAKE MODES: "INTAKING, OUTAKING"
        runArmToPos(armMode.HOME);                           // SETS ARM (shoulder & elbow) TO GIVEN POSITION. ARM MODES: "HOME, HOVER, INTAKE, HIGH"

        // AUTONOMOUS INSTRUCTIONS BELOW:
        */        

        driveDiagonal(0.2, -0.55);
        runSpec(specMode.HIGH, 0.6); // CLIP FIRST
        drivetrainAuto.profiledTurnHeading(-58.0); // TOWARDS FLOOR SAMPLE
        runIntakeMovement(0.80, 1000, intakeMode.INTAKING); // INTAKE FLOOR SAMPLE
        drivetrainAuto.profiledTurnHeading(-80.0); // TOWARDS OBV
        runOuttake(800); // OUTTAKES
        runArmToPos(armMode.HOME); // ARM RESET
        drivetrainAuto.profiledTurnHeading(-45.0); // CORRECTS ANGLE
        drivetrainAuto.profiledDriveSideways(-0.4); // ALIGHNS TOWARDS 2ND SPEC
        drivetrainAuto.profiledDriveStraight(0.2); 
        sleep(1000);
        runSpec(specMode.INTAKE, 0.60); //  INTAKE 2ND SPEC
        drivetrainAuto.profiledTurnHeading(-180.0); // TURNS AROUND
        specimen.setTargetPosition(specimenHigh); // GETS SPEC READY
        driveDiagonal(-0.15, -1.35); // GO TO HIGH
        runSpec(specMode.HIGH, 0.4); // SCORES HIGH
        specimen.setTargetPosition(specimenHome); // SPEC HOME RESET
        driveDiagonal(-0.2, 1.80); // PARK HOME


    }
}
