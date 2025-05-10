package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robochargers.DrivetrainParametersBase;
import org.firstinspires.ftc.teamcode.robochargers.MecanumDrivetrain.GyroUpDirection;
import org.firstinspires.ftc.teamcode.robochargers.Constants;

public class DriveParametersTemplate extends DrivetrainParametersBase{

    public DriveParametersTemplate() {
        FRONT_LEFT_NAME = "left_front";
        FRONT_RIGHT_NAME = "right_front";
        REAR_LEFT_NAME = "left_back";
        REAR_RIGHT_NAME = "right_back";
        
        GYRO_UP_DIRECTION = GyroUpDirection.POSITIVE_Z;
        
        COUNTS_PER_MOTOR_REV = 28.0;

        DRIVE_GEAR_REDUCTION = Constants.UltraPlanetary.THREE_TO_ONE_REDUCTION * Constants.UltraPlanetary.THREE_TO_ONE_REDUCTION;

        WHEEL_RADIUS = 0.0375;

        FRONT_LEFT_INVERTED = true;
        FRONT_RIGHT_INVERTED = false;
        REAR_LEFT_INVERTED = true;
        REAR_RIGHT_INVERTED = false;

    }
}