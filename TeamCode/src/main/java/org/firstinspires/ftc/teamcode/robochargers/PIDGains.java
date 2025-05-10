package org.firstinspires.ftc.teamcode.robochargers;

public class PIDGains {
    public final double kS;
    
    public final double kV;

    public final double kE;
    
    public final double kA;
    
    public final double kP;
    
    public final double kI;
    
    public final double kD;

    public PIDGains(double _kS, double _kV, double _kE, double _kA, double _kP, double _kI, double _kD) {
        kS = _kS;
        kV = _kV;
        kE = _kE;
        kA = _kA;
        kP = _kP;
        kI = _kI;
        kD = _kD;
    }
    
}
