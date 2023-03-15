package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    /* Constants */
    /* CAN IDs */
    public final int PIVOT_TALON_FX_CAN_ID = 4;
    public final int EXTENDER_TALON_FX_CAN_ID = 5;

    /* PIDs */
    public final double PIVOT_KF = 0.7d;
    public final double PIVOT_KP = 0.3d;
    public final double PIVOT_KI = 0.001d;
    public final double PIVOT_KD = 3d;

    public final double EXTENDER_KF = 0.08d;
    public final double EXTENDER_KP = 0.1d;
    public final double EXTENDER_KI = 0.0025d;
    public final double EXTENDER_KD = 1d;

    /* Velocity */
    public final double PIVOT_MAX_VELOCITY = 3000.0d;
    public final double PIVOT_MAX_ACCELERATION = 3000.0d;
    public final double EXTENDER_MAX_VELOCITY = 20000.0d;
    public final double EXTENDER_MAX_ACCELERATION = 20000.0d;

    /* Motor constants */
    public final double MOTOR_NEUTRAL_DEADBAND = 0.001d;
    public final int PID_CONFIG_TIMEOUT_MS = 10;
    public final int CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS = 10;
    public final double PIVOT_CALIBRATION_MOTOR_SPEED = 0.1d;
    public final double EXTENDER_CALIBRATION_MOTOR_SPEED = 0.1d;

    /* Conversion Factors */
    public final double ENCODER_UNITS_PER_REVOLUTION = 2048.0d / 1.0d;

    public final double PIVOT_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO = 36.00d / 1.0d;
    public final double PIVOT_PULLEY_MOTOR_TO_PULLEY_ARM_RATIO = 72.0d / 36.0d;

    public final double EXTENDER_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO = 4.0d / 1.0d;
    public final double EXTENDER_PULLEY_ROTATION_TO_INCHES = 3.75d; // One rotation of the final extender pulley moves
                                                                    // the arm 3.75 inches
    public final double RADIANS_PER_REVOLUTION = 2 * Math.PI;
    public final double PIVOT_ENCODER_UNITS_PER_RADIANS = (((ENCODER_UNITS_PER_REVOLUTION)
            * (PIVOT_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO) * (PIVOT_PULLEY_MOTOR_TO_PULLEY_ARM_RATIO))
            / (RADIANS_PER_REVOLUTION));
    public final double EXTENDER_ENCODER_UNITS_PER_INCH = ((ENCODER_UNITS_PER_REVOLUTION
            * EXTENDER_GEARBOX_MOTOR_TO_GEARBOX_ARM_RATIO) / EXTENDER_PULLEY_ROTATION_TO_INCHES);

    /* Physical constants */
    public final double MIN_ANGLE_RADIANS = -90.0d * ((2 * Math.PI) / 360.0d); // radians
    public final double MAX_ANGLE_RADIANS = 20.0d * ((2 * Math.PI) / 360.0d); // radians
    public final double MIN_INCHES = 34.712d;
    public final double MAX_INCHES = 49.6d;

    public final double ARM_THETA_DELTA_MODIFIER = 1.0d * ((2 * Math.PI) / 360.0d); // radians
    public final double ARM_R_DELTA_MODIFIER = 1.0d; // inches

    public final double ZEROED_R_POSITION_RADIANS = 0.0d;
    public final double ZEROED_THETA_POSITION_INCHES = -34.712d;

    public final int ZEROED_PIVOT_ENCODER_LIMIT = (int) (MIN_ANGLE_RADIANS * PIVOT_ENCODER_UNITS_PER_RADIANS);
    public final int ZEROED_EXTENDER_ENCODER_LIMIT = (int) (MIN_INCHES * EXTENDER_ENCODER_UNITS_PER_INCH);

    public final double MIN_RESTRICTED_THETA = -72.0d * ((2 * Math.PI) / 360.0d); // radians
    public final double MAX_RESTRICTED_INCHES = MIN_INCHES;

    /* Members */
    private WPI_TalonFX pivotTalonFX;
    private WPI_TalonFX extenderTalonFX;
    private double theta = MIN_ANGLE_RADIANS;
    private double rInches = MIN_INCHES;
    private boolean isPivotCalibrated = false;
    private boolean isExtenderCalibrated = false;

    // Gear ratios
    public Arm() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        pivotTalonFX = new WPI_TalonFX(PIVOT_TALON_FX_CAN_ID);
        extenderTalonFX = new WPI_TalonFX(EXTENDER_TALON_FX_CAN_ID);

        // Clears motor errors
        pivotTalonFX.clearStickyFaults();
        extenderTalonFX.clearStickyFaults();

        // Set factory defaults for onboard PID
        pivotTalonFX.configFactoryDefault();
        extenderTalonFX.configFactoryDefault();

        pivotTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);
        extenderTalonFX.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0,
                CONFIG_ARM_FEEDBACKSENSOR_TIMEOUT_MS);

        pivotTalonFX.setInverted(true);
        pivotTalonFX.setSensorPhase(true);
        extenderTalonFX.setInverted(false);
        extenderTalonFX.setSensorPhase(false);

        // Configure Position PID
        pivotTalonFX.config_kF(0, PIVOT_KF, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.config_kP(0, PIVOT_KP, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.config_kI(0, PIVOT_KI, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.config_kD(0, PIVOT_KD, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.config_IntegralZone(0, 1000);
        pivotTalonFX.configMotionCruiseVelocity(PIVOT_MAX_VELOCITY, PID_CONFIG_TIMEOUT_MS);
        pivotTalonFX.configMotionAcceleration(PIVOT_MAX_ACCELERATION, PID_CONFIG_TIMEOUT_MS);

        extenderTalonFX.config_kF(0, EXTENDER_KF, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.config_kP(0, EXTENDER_KP, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.config_kI(0, EXTENDER_KI, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.config_kD(0, EXTENDER_KD, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.config_IntegralZone(0, 100);
        extenderTalonFX.configMotionCruiseVelocity(EXTENDER_MAX_VELOCITY, PID_CONFIG_TIMEOUT_MS);
        extenderTalonFX.configMotionAcceleration(EXTENDER_MAX_ACCELERATION, PID_CONFIG_TIMEOUT_MS);

        extenderTalonFX.setNeutralMode(NeutralMode.Brake);
        pivotTalonFX.setNeutralMode(NeutralMode.Brake);

        extenderTalonFX.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);
        pivotTalonFX.configNeutralDeadband(MOTOR_NEUTRAL_DEADBAND);

        // Start the calibration process
        isPivotCalibrated = false;
        isExtenderCalibrated = false;
        startCalibration();
    }

    @Override
    public void periodic() {
        /* Update smart dashboard */
        SmartDashboard.putNumber("theta", theta);
        SmartDashboard.putNumber("R(inches)", rInches);
        SmartDashboard.putNumber("pivotMotor encoder", pivotTalonFX.getSelectedSensorPosition());
        SmartDashboard.putNumber("extender motor position", extenderTalonFX.getSelectedSensorPosition());
        SmartDashboard.putBoolean("Pivot Rev Limit", (1 == pivotTalonFX.isRevLimitSwitchClosed()));
        SmartDashboard.putBoolean("Extender Rev Limit", (1 == extenderTalonFX.isRevLimitSwitchClosed()));

        /* Are we calibrated? */
        if (true == isCalibrated()) {
            /* Clamp the value to the max or min if needed */
            theta = Math.max(Math.min(theta, MAX_ANGLE_RADIANS), MIN_ANGLE_RADIANS);

            /* Clamp the value to the max or min if needed */
            rInches = Math.max(Math.min(rInches, MAX_INCHES), MIN_INCHES);

            moveToPosition(theta, rInches);
        } else {
            /* Not calibrated, do calibration! */
            if (false == isExtenderCalibrated) {
                extenderTalonFX.set(-EXTENDER_CALIBRATION_MOTOR_SPEED);
            } else if (false == isPivotCalibrated) {
                pivotTalonFX.set(-PIVOT_CALIBRATION_MOTOR_SPEED);
            }

            /* Update calibration status */
            checkCalibration();
        }
    }

    /**
     * Rotate the arm to a specific angle.
     * 
     * @param angle - the angle in radians
     */
    private void rotateTo(double radians) {
        double minClamp = MIN_ANGLE_RADIANS;
        double encoderUnits = 0.0d;

        if (rInches >= (MIN_INCHES + 1)) {
            minClamp = MIN_RESTRICTED_THETA;
        }

        /* Clamp the value to the max or min if needed */
        radians = Math.max(Math.min(radians, MAX_ANGLE_RADIANS), minClamp);

        /* Convert radians to encoder units */
        encoderUnits = radians * PIVOT_ENCODER_UNITS_PER_RADIANS;

        SmartDashboard.putNumber("Pivot commanded", encoderUnits);

        pivotTalonFX.set(TalonFXControlMode.MotionMagic, encoderUnits, DemandType.ArbitraryFeedForward,
                PIVOT_KF * Math.abs((Math.cos(radians))));
    }

    /**
     * Extends arm to a given amount of inches.
     * 
     * @param inches - the length to extend/retract to in inches
     */
    private void extendTo(double inches) {
        double maxClamp = MAX_INCHES;
        double encoderUnits;

        /* Prevent a collision with the robot bumpers */
        if (theta <= MIN_RESTRICTED_THETA) {
            maxClamp = MAX_RESTRICTED_INCHES;
        }

        // Convert from inches to encoder units
        encoderUnits = inches * EXTENDER_ENCODER_UNITS_PER_INCH;

        /* Clamp the value to the max or min if needed */
        inches = Math.max(Math.min(inches, maxClamp), MIN_INCHES);

        SmartDashboard.putNumber("extender encoder", encoderUnits);
        extenderTalonFX.set(TalonFXControlMode.MotionMagic, encoderUnits, DemandType.ArbitraryFeedForward, 0.0d);
    }

    /**
     * Moves the arm to a given position in space with the arm pivot as the origin.
     * 
     * @param targetTheta   - target position of arm in radians
     * @param targetRInches - target position of the extension in inches
     */
    public void moveToPosition(double targetTheta, double targetRInches) {
        rotateTo(targetTheta);
        extendTo(targetRInches);
    }

    /**
     * Sets the internal position of the arm.
     * 
     * @param targetTheta   - target angle of arm in radians
     * @param targetRInches - target position of the extension in inches
     */
    public void setPosition(double targetTheta, double targetRInches) {
        theta = targetTheta;
        rInches = targetRInches;
    }

    public void adjustPosition(double anglePercent, double extendPercent) {
        theta += ARM_THETA_DELTA_MODIFIER * anglePercent;
        rInches += ARM_R_DELTA_MODIFIER * extendPercent;
    }

    public void pivotSetEncoderUnits(int encoderUnits) {
        pivotTalonFX.setSelectedSensorPosition(encoderUnits);
    }

    public void extenderSetEncoderUnits(int encoderUnits) {
        extenderTalonFX.setSelectedSensorPosition(encoderUnits);
    }

    public void startCalibration() {
        isPivotCalibrated = false;
        isExtenderCalibrated = false;
    }

    private boolean checkCalibration() {
        boolean done = true;

        if (extenderTalonFX.isRevLimitSwitchClosed() == 1) {
            extenderTalonFX.set(0);
            extenderSetEncoderUnits(ZEROED_EXTENDER_ENCODER_LIMIT);

            isExtenderCalibrated = true;
        } else {
            done = false;
        }

        if (pivotTalonFX.isRevLimitSwitchClosed() == 1) {
            pivotTalonFX.set(0);
            pivotSetEncoderUnits(ZEROED_PIVOT_ENCODER_LIMIT);
            isPivotCalibrated = true;
        } else {
            done = false;
        }

        return (done);
    }

    public boolean isCalibrated() {
        return ((true == isPivotCalibrated) && (true == isExtenderCalibrated));
    }
}
