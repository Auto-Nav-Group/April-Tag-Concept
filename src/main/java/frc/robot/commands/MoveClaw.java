package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.classes.SPIKE293Utils;
import frc.robot.subsystems.Claw;


public class MoveClaw extends CommandBase {
    public boolean opened = false;
    private final Claw m_claw;
    public final XboxController m_operatorXboxController;
    public double m_triggerDeadband = 0.05;

    public MoveClaw(Claw givenClaw, XboxController givenController) {
        m_claw = givenClaw;
        m_operatorXboxController = givenController;

        addRequirements(m_claw);
        SmartDashboard.putNumber("Claw Trigger Deadband", m_triggerDeadband);    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double rightTrigger = m_operatorXboxController.getRightTriggerAxis(); 
        double leftTrigger = m_operatorXboxController.getLeftTriggerAxis();

        m_triggerDeadband = SmartDashboard.getNumber("Claw Trigger Deadband", m_triggerDeadband);
        m_triggerDeadband = MathUtil.clamp(m_triggerDeadband, 0.0, 1.0);

        rightTrigger = SPIKE293Utils.applyDeadband(rightTrigger, m_triggerDeadband);
        leftTrigger = SPIKE293Utils.applyDeadband(leftTrigger, m_triggerDeadband);

        if (rightTrigger > leftTrigger) {
            m_claw.percentClaw(rightTrigger);
        } else {
            m_claw.percentClaw(-leftTrigger);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
