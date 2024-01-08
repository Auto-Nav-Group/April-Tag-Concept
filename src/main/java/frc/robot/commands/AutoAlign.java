package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Position2D;

public class AutoAlign extends CommandBase {
    private double m_linearP;
    private double m_linearI;
    private double m_linearD;
    private double m_angularP;
    private double m_angularI;
    private double m_angularD;
    private double m_angularErrorSum;
    private double m_linearErrorSum;
    private double m_lastAngularError;
    private double m_lastLinearError;
    private SwerveDrivePoseEstimator estimator;
    public AutoAlign() {
        m_linearP = 0.0d;
        m_linearI = 0.0d;
        m_linearD = 0.0d;
        m_angularP = 0.0d;
        m_angularI = 0.0d;
        m_angularD = 0.0d;
    }

    @Override
    public void execute() {

    }
}
