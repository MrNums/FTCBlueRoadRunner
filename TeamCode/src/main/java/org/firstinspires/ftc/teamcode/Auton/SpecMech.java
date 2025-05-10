package org.firstinspires.ftc.teamcode.Auton;

/*
    How to use Spec Mech:

    SpecMech spec = new SpecMech(hardwareMap);
    sequence.add(spec.prepIntake());
*/

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOpParameters;
import org.firstinspires.ftc.teamcode.Auton.AutonSettings;

public class SpecMech {
    private DcMotorEx specimen;
    private CRServo specLServo, specRServo;

    public SpecMech(HardwareMap hardwareMap) {
        // Initialize the specimen motor and servos
        specimen = hardwareMap.get(DcMotorEx.class, "spec");
        specimen.setDirection(TeleOpParameters.SPECIMEN_DIRECTION);
        specimen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        specimen.setTargetPosition(0);
        specimen.setPower(1.0);
        specimen.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        specLServo = hardwareMap.get(CRServo.class, "specLServo");
        specRServo = hardwareMap.get(CRServo.class, "specRServo");
    }

    // Move the specimen to a specified state
    public Action moveToState(String state) {
        int specimenTarget;

        switch (state.toLowerCase()) {
            case "home":
                specimenTarget = AutonSettings.SPECIMEN_HOME;
                break;
            case "wall":
                specimenTarget = AutonSettings.SPECIMEN_WALL;
                break;
            case "high":
                specimenTarget = AutonSettings.SPECIMEN_HIGH;
                break;
            default:
                throw new IllegalArgumentException("Unknown specimen state: " + state);
        }

        return moveToSpecimenPosition(specimenTarget);
    }

    // Move the specimen to a specific position
    public Action moveToSpecimenPosition(int target) {
        return packet -> {
            specimen.setTargetPosition(target);
            specimen.setPower(1.0);

            packet.put("Specimen Pos", specimen.getCurrentPosition());

            if (!specimen.isBusy()) {
                specimen.setPower(0);
                return false; // Action complete
            }

            return true; // Action still in progress
        };
    }

    // Prepare the intake mechanism
    public Action prepIntake() {
        return new SequentialAction(
                moveToSpecimenPosition(AutonSettings.SPECIMEN_WALL),
                (TelemetryPacket packet) -> {
                    specLServo.setPower(1.0);
                    specRServo.setPower(-1.0);
                    return false; // Action still in progress
                },
                moveToSpecimenPosition(AutonSettings.SPECIMEN_WALL_LIFT),
                (TelemetryPacket packet) -> {
                    specLServo.setPower(0);
                    specRServo.setPower(0);
                    return false; // Action complete
                }
        );
    }

    // Move to the high position
    public Action moveToHigh() {
        return moveToSpecimenPosition(AutonSettings.SPECIMEN_HIGH);
    }

    // Move down to clip and release, keeping servos running during sleep
    public Action clipAndRelease() {
        return new SequentialAction(
                moveToSpecimenPosition(AutonSettings.SPECIMEN_HIGH_CLIP),
                packet -> {
                    specLServo.setPower(-1.0);
                    specRServo.setPower(1.0);
                    return false; // step done: servos powered on
                },
                new SleepAction(1.5), // this waits 1.5 seconds while servos run
                packet -> {
                    specLServo.setPower(0);
                    specRServo.setPower(0);
                    return false; // stop servos, step done
                }
        );
    }


    // Return to home position
    public Action returnHome() {
        return moveToSpecimenPosition(AutonSettings.SPECIMEN_HOME);
    }

    // Stop the specimen servos
    private Action stopSpecimenServos() {
        return packet -> {
            specLServo.setPower(0);
            specRServo.setPower(0);
            return false; // Action complete
        };
    }
}

