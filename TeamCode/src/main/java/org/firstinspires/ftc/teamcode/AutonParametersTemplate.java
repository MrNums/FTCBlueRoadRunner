package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robochargers.AutonParametersBase;
import org.firstinspires.ftc.teamcode.robochargers.PIDGains;

public class AutonParametersTemplate extends AutonParametersBase {
    
    public AutonParametersTemplate() {
        MOTION_PROFILE_LOGGING = false;
        HEADING_GAINS = new PIDGains(0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0);
        POSITION_GAINS = new PIDGains(0.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0);
        
        STRAIGHT_MAX_SPEED = 1.0;
        STRAIGHT_ACCELERATION_RATE = 0.9;
        STRAIGHT_DECELERATION_RATE = 0.9;
        STRAIGHT_TIMEOUT_MULTIPLIER = 1.5;
        STRAIGHT_GAINS = new PIDGains(0.107, 0.234, 0.0998, 0.15, 0.5, 0.0, 0.0);
        
        STRAFE_MAX_SPEED = 1.0;
        STRAFE_ACCELERATION_RATE = 0.8;
        STRAFE_DECELERATION_RATE = 0.8;
        STRAFE_TIMEOUT_MULTIPLIER = 1.2;
        STRAFE_GAINS = new PIDGains(0.215, 0.249, 0.444, 0.15, 2.0, 0.0, 0.0);
        
        TURNING_MAX_SPEED = 85.0;
        TURNING_ACCELERATION_RATE = 90.0;
        TURNING_DECELERATION_RATE = 90.0;
        TURNING_TIMEOUT_MULTIPLIER = 1.2;
        TURNING_GAINS = new PIDGains(0.138, 1.8e-3, 1.16e-6, 0.0005, 0.005, 0.0, 0.0);
    }
}
