package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.Position2D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightInterface;

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
    private LimelightInterface m_limelight;
    private Drivetrain m_drivetrain;
    private SwerveDrivePoseEstimator m_estimator;
    public AutoAlign(LimelightInterface limelight, Drivetrain drivetrain, SwerveDrivePoseEstimator estimator) {
        m_limelight = limelight;
        m_drivetrain = drivetrain;
        m_estimator = estimator;
        addRequirements(limelight, drivetrain);

        m_linearP = 0.0d;
        m_linearI = 0.0d;
        m_linearD = 0.0d;
        m_angularP = 0.0d;
        m_angularI = 0.0d;
        m_angularD = 0.0d;
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_estimator.getEstimatedPosition();
        Pose2d targetPose = m_limelight.getTargetPose();

        double targetHeading = targetPose.getRotation().getDegrees();
        if (targetHeading > Math.PI) {
            targetHeading -= 2 * Math.PI;
        } else if (targetHeading < -Math.PI) {
            targetHeading += 2 * Math.PI;
        }
        double angularError = targetHeading - currentPose.getRotation().getDegrees();
        double linearError = m_limelight.getDistance() - currentPose.getTranslation().getNorm();
        double angularErrorDelta = angularError - m_lastAngularError;
        double linearErrorDelta = linearError - m_lastLinearError;
        m_angularErrorSum += angularError;
        m_linearErrorSum += linearError;
        double angularCommand = (m_angularP * angularError) + (m_angularI * m_angularErrorSum) + (m_angularD * angularErrorDelta);
        double linearCommand = (m_linearP * linearError) + (m_linearI * m_linearErrorSum) + (m_linearD * linearErrorDelta);
        m_lastAngularError = angularError;
        m_lastLinearError = linearError;
        // Make the drivetrain drive towards the target
        Translation2d translation = new Translation2d(linearCommand, 0.0d);
        m_drivetrain.drive(translation, angularCommand, false, true);
    }
}
