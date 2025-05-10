package org.firstinspires.ftc.teamcode.Auton;

/*
    How to use Arm Mech

    ArmMech arm = new ArmMech(hardwareMap);
    sequence.add(arm.moveToState("intake"));

 */

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOpParameters;
import org.firstinspires.ftc.teamcode.Auton.AutonSettings;

public class ArmMech {
    private DcMotorEx elbow;
    private DcMotorEx shoulder;
    private DcMotorEx intake;

    public ArmMech(HardwareMap hardwareMap) {
        // Elbow setup
        elbow = hardwareMap.get(DcMotorEx.class, "elbow");
        elbow.setDirection(TeleOpParameters.ELBOW_DIRECTION); // Double check this
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setTargetPosition(0);
        elbow.setPower(1.0);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Shoulder setup
        shoulder = hardwareMap.get(DcMotorEx.class, "shoulder");
        shoulder.setDirection(TeleOpParameters.SHOULDER_DIRECTION); // Double check this
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulder.setTargetPosition(0);
        shoulder.setPower(1.0);
        shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Intake setup
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(TeleOpParameters.INTAKE_DIRECTION);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public Action moveToState(String state) {
        int elbowTarget = 0;
        int shoulderTarget = 0;

        switch (state.toLowerCase()) {
            case "intake":
                elbowTarget = AutonSettings.ELBOW_INTAKE_POS;
                shoulderTarget = AutonSettings.SHOULDER_INTAKE_POS;
                intake.setPower(1.0); // Intake
                break;
            case "prep_score_high": // Ensure this matches your intended state
                elbowTarget = AutonSettings.ELBOW_SCORE_HIGH_POS;
                shoulderTarget = AutonSettings.SHOULDER_SCORE_HIGH_POS;
                intake.setPower(0.2); // Holding
                break;
            case "hover":
                elbowTarget = AutonSettings.ELBOW_HOVER_POS;
                shoulderTarget = AutonSettings.SHOULDER_HOVER_POS;
                intake.setPower(0.2); // Holding
                break;
            case "outtake":
                elbowTarget = AutonSettings.ELBOW_OUTTAKE_POS;
                shoulderTarget = AutonSettings.SHOULDER_OUTTAKE_POS;
                intake.setPower(0.2); // Holding
                break;
            case "home":
                elbowTarget = AutonSettings.ELBOW_START_POS;
                shoulderTarget = AutonSettings.SHOULDER_START_POS;
                intake.setPower(0.2); // Holding
                break;
            case "scorehigh": // Add this case
                elbowTarget = AutonSettings.ELBOW_SCORE_HIGH_POS; // Set the appropriate position
                shoulderTarget = AutonSettings.SHOULDER_SCORE_HIGH_POS; // Set the appropriate position
                intake.setPower(0.2); // Adjust power as needed
                break;
            default:
                throw new IllegalArgumentException("Unknown arm state: " + state);
        }

        final int finalElbow = elbowTarget;
        final int finalShoulder = shoulderTarget;

        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                elbow.setTargetPosition(finalElbow);
                shoulder.setTargetPosition(finalShoulder);
                elbow.setPower(1.0);
                shoulder.setPower(1.0);

                packet.put("Elbow Pos", elbow.getCurrentPosition());
                packet.put("Shoulder Pos", shoulder.getCurrentPosition());

                boolean busy = elbow.isBusy() || shoulder.isBusy();
                if (!busy) {
                    elbow.setPower(0);
                    shoulder.setPower(0);
                }
                return busy;
            }
        };
    }

    public Action prepareToScore() {
        return moveToState("scorehigh");
    }

    public Action score() {
        return new Action() {
            private long startTime = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == -1) {
                    intake.setPower(-1.0); // Outtake
                    startTime = System.currentTimeMillis();
                }

                long elapsed = System.currentTimeMillis() - startTime;

                if (elapsed >= 500) { // 0.5 seconds
                    intake.setPower(0);
                    moveToState("home").run(packet);
                    return false;
                }
                return true;
            }
        };
    }

}
