package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;

public class AutonSettings {
    public static final DcMotor.Direction SHOULDER_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotor.Direction ELBOW_DIRECTION = DcMotor.Direction.REVERSE;

    public static final int SHOULDER_INTAKE_POS = 400;
    public static final int SHOULDER_SCORE_HIGH_POS = 2000;
    public static final int SHOULDER_HOVER_POS = 450;
    public static final int SHOULDER_OUTTAKE_POS = 3000;
    public static final int SHOULDER_START_POS = 0;

    public static final int ELBOW_INTAKE_POS = 1260;
    public static final int ELBOW_SCORE_HIGH_POS = 0;
    public static final int ELBOW_HOVER_POS = 1050;
    public static final int ELBOW_OUTTAKE_POS = -1500;
    public static final int ELBOW_START_POS = 0;

    public static final int SPECIMEN_HOME = 0;
    public static final int SPECIMEN_WALL = 50;
    public static final int SPECIMEN_WALL_LIFT = 200;
    public static final int SPECIMEN_LOW_CLIP = 0;
    public static final int SPECIMEN_LOW_EJECT = 0;
    public static final int SPECIMEN_HIGH = 1300;
    public static final int SPECIMEN_HIGH_CLIP = 820;
    public static final int SPECIMEN_HIGH_EJECT = 820;
    public static final int SPECIMEN_TOLERANCE = 10;
}
