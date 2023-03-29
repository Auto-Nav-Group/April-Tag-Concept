// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.
package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 *
 */
public class Targeting extends SubsystemBase {
    public static final int DEFAULT_TARGET_RPM = 2000;
    
    public static final int LIMELIGHT_LED_ON = 3;
    public static final int LIMELIGHT_LED_OFF = 1;
    
    public static final double TARGET_ACQUIRED = 1.0;
    public static final double CONFIRMED_THRESHOLD = 0.2;
    
    private NetworkTable m_limeData; // Data from limelight
    private NetworkTableEntry m_tAcquired; // t stands for target
    private NetworkTableEntry m_targetX; // x value of the target
    private NetworkTableEntry m_targetY; // y value of the target

    private boolean m_isReadyToFire = false;

    public Targeting() {
        // Get limelight data from network table
        m_limeData = NetworkTableInstance.getDefault().getTable("limelight");
        m_tAcquired = m_limeData.getEntry("tv");
        m_targetX = m_limeData.getEntry("tx");
        m_targetY = m_limeData.getEntry("ty");

        // Set default values for shuffleboard
        m_limeData.getEntry("camMode").setNumber(0);
        m_limeData.getEntry("ledMode").setNumber(LIMELIGHT_LED_ON);
        m_limeData.getEntry("pipeline").setNumber(1.0d);

        SmartDashboard.putBoolean("isTargetted", false);

        controlLight(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Tx", m_targetX.getDouble(10000));
        SmartDashboard.putNumber("Ty", m_targetY.getDouble(10000));
        SmartDashboard.putBoolean("isTargetted", isTargeted());
        SmartDashboard.putNumber("Distance from Target", calcDistance());
    }

    public boolean getIsReadyToFire() {
        return m_isReadyToFire;
    }

    // Turns the LED on or off
    public void controlLight(boolean enabled) {
        if (enabled) {
            m_limeData.getEntry("ledMode").setNumber(LIMELIGHT_LED_ON);
        } else {
            m_limeData.getEntry("ledMode").setNumber(LIMELIGHT_LED_OFF);
        }
    }

    public double calcShooterRPM() {
        double retval = DEFAULT_TARGET_RPM;
        double ty = m_targetY.getDouble(0.0);
        if (m_tAcquired.getDouble(0.0) == TARGET_ACQUIRED) {
            //retv  al = (-30.07 * ty) + 1690.42;
            retval = (230 * Math.pow(Math.E, ((-0.237 * ty) - 1.5))) + 1680.48;
            if(retval > 2900.0){
                retval = 2900.0;
            }
            if(retval < 1600.0){
                retval = 1600.0;
            }
        }

        return retval;
    }

    public boolean isTargeted() {
        boolean targeted = false;
        double limeError = m_targetX.getDouble(0.0); // Get the error of the target X

        if (Math.abs(limeError) < CONFIRMED_THRESHOLD) {
            targeted = true;
        }

        return targeted;
    }

    public double calcDistance() {
        double targetOffsetAngle_Vertical = m_targetY.getDouble(0.0);
        double limelightMountAngleDegrees = 36.574;
        double limelightLensHeightInches = 33.5;
        double goalHeightInches = 104.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);

        return distanceFromLimelightToGoalInches;
    }

    public double getAngleToTargetDegrees() {
        return -1 * m_targetX.getDouble(0);
    }

    public boolean hasTarget(){
        if (m_tAcquired.getDouble(0.0) == TARGET_ACQUIRED) {
            return true;
        }
        return false;
    }
}
