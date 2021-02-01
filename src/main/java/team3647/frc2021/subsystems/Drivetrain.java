package team3647.frc2021.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpiutil.math.MathUtil;
import team3647.lib.DriveSignal;
import team3647.lib.drivers.ClosedLoopFactory.ClosedLoopConfig;
import team3647.lib.wpi.HALMethods;
import team3647.lib.wpi.Timer;
/**
 * driver boi
 */
public class Drivetrain implements PeriodicSubsystem {
    private static int constructCount = 0;
    private TalonSRX leftMaster = new TalonSRX(9);   // motors 
    private TalonSRX rightMaster = new TalonSRX(4);
    private VictorSPX leftSlave1 = new VictorSPX(12);
    private VictorSPX leftSlave2 = new VictorSPX(13);
    private VictorSPX rightSlave1 = new VictorSPX(5);
    private VictorSPX rightSlave2 = new VictorSPX(6);

    private final ClosedLoopConfig m_leftPIDConfig;
    private final ClosedLoopConfig m_rightPIDConfig;

    private boolean initialized = false;
    private PeriodicIO periodicIO = new PeriodicIO();

    private final SimpleMotorFeedforward feedforward;

    private final double kEncoderVelocityToMetersPerSecond;

    private double m_timeStamp;
    private double m_maxOutput;

    private boolean shifted;

    public Drivetrain(ClosedLoopConfig leftMasterPIDConfig, ClosedLoopConfig rightMasterPIDConfig, double kWheelDiameterMeters, double kS, double kV, double kA) {
        if (constructCount > 0) {
            throw new UnsupportedOperationException("Drivetrain was already initialized once");
        }
      
        m_leftPIDConfig = leftMasterPIDConfig;
        m_rightPIDConfig = rightMasterPIDConfig;

        leftSlave1.follow(leftMaster);
        leftSlave2.follow(leftMaster);
        rightSlave1.follow(rightMaster);
        rightSlave2.follow(rightMaster);

        rightMaster.setInverted(true);
        leftMaster.setInverted(false);
        
        kEncoderVelocityToMetersPerSecond =
                m_leftPIDConfig.kEncoderVelocityToRPM * kWheelDiameterMeters * Math.PI / 60.0;

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        
        constructCount++;
        shifted = false;
    }

    public static class PeriodicIO {
        // inputs
        /** Meters per second */
        public double leftVelocity;
        /** Meters per second */
        public double rightVelocity;

        public boolean climbLimitSwitch;
        public double ypr[] = new double[] {0, 0, 0};

        /** Meters */
        public double leftPosition;
        /** Meters */
        public double rightPosition;
        /** Degrees -180 to 180 */
        public double heading;

        // outputs
        public double leftOutput;
        public double rightOutput;

        /** Meters Per second */
        public double prevLeftDesiredVelocity;
        /** Meters Per Second */
        public double prevRightDesiredVelocity;

        /** Volts */
        public double leftFeedForward;
        /** Volts */
        public double rightFeedForward;
    }

    @Override
    public synchronized void init() {
        initialized = true;
    }

    @Override
    public void periodic() {
        PeriodicSubsystem.super.periodic();
        m_timeStamp = Timer.getFPGATimestamp();
    }

  /* @Override
    public synchronized void end() {
        leftMaster.stopMotor();
        rightMaster.stopMotor();
        leftSlave1.stopMotor();
        leftSlave2.stopMotor();
        rightSlave1.stopMotor();
        rightSlave1.stopMotor();

        periodicIO.leftOutput = 0;
        periodicIO.rightOutput = 0;
        periodicIO.leftFeedForward = 0;
        periodicIO.rightFeedForward = 0;

    }
*/
    public synchronized void setOpenLoop(DriveSignal driveSignal) {
        if (driveSignal != null) {
            periodicIO.leftOutput = driveSignal.getLeft();
            periodicIO.rightOutput = driveSignal.getRight();
            periodicIO.leftFeedForward = 0;
            periodicIO.rightFeedForward = 0;
        } else {
            end();
            HALMethods.sendDSError("DriveSignal in setOpenLoop was null");
        }
    }

    public void setVelocityMpS(double leftMPS, double rightMPS) {
        setVelocity(new DriveSignal(leftMPS, rightMPS));
    }

    /**
     * @param driveSignal in meters per second
     */
    public synchronized void setVelocity(DriveSignal driveSignal) {
        if (driveSignal != null) {
            if (!isShifted()) {
                periodicIO.leftFeedForward = feedforward.calculate(driveSignal.getLeft());
                periodicIO.rightFeedForward = feedforward.calculate(driveSignal.getRight());

                periodicIO.leftOutput = driveSignal.getLeft() / kEncoderVelocityToMetersPerSecond;
                periodicIO.rightOutput = driveSignal.getRight() / kEncoderVelocityToMetersPerSecond;
            }
        } else {
            HALMethods.sendDSError("Drive signal in setVelocity(DriveSignal) was null");
            end();
        }
    }

    public void arcadeDriveVelocity(double xSpeed, double zRotation, boolean scaleInputs) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, .09);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, .09);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        if (scaleInputs) {
            m_maxOutput = .7;
        }

        setVelocity(new DriveSignal(leftMotorOutput * m_leftPIDConfig.maxVelocity,
                rightMotorOutput * m_rightPIDConfig.maxVelocity));

    }

    public void arcadeDrive(double xSpeed, double zRotation, boolean scaleInputs) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, .09);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, .09);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }
        if (scaleInputs) {
            m_maxOutput = .7;
            leftMotorOutput *= m_maxOutput;
            rightMotorOutput *= m_maxOutput;
        }
        // double currentLeftDesiredVelocity = MathUtil.clamp(leftMotorOutput, -1.0,
        // 1.0) *
        // m_maxOutput
        // * m_leftPIDConfig.maxVelocity;
        // double currentRightDesiredVelocity = MathUtil.clamp(rightMotorOutput, -1.0,
        // 1.0)
        // * m_maxOutput * m_rightPIDConfig.maxVelocity;

        // double leftVoltage = feedforward.calculate(currentLeftDesiredVelocity,
        // (currentLeftDesiredVelocity - periodicIO.prevLeftDesiredVelocity) / kDt);
        // double rightVoltage = feedforward.calculate(currentRightDesiredVelocity,
        // (currentRightDesiredVelocity - periodicIO.prevRightDesiredVelocity) / kDt);

        // if (shifted) {
        // setOpenLoop(new DriveSignal(xSpeed, xSpeed));
        // } else {
        // setOpenLoop(new DriveSignal(leftVoltage / m_leftMasterConfig.nominalVoltage,
        // rightVoltage / m_rightMasterConfig.nominalVoltage));
        // }
        setOpenLoop(new DriveSignal(leftMotorOutput, rightMotorOutput));

        periodicIO.prevLeftDesiredVelocity = leftMotorOutput * m_leftPIDConfig.maxVelocity;
        periodicIO.prevRightDesiredVelocity = rightMotorOutput * m_rightPIDConfig.maxVelocity;
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    protected double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    /**
     * @return left position in meters
     */
    public double getLeftPosition() {
        return periodicIO.leftPosition;
    }

    /**
     * @return right encoder position in meters
     */
    public double getRightPosition() {
        return periodicIO.rightPosition;
    }

    public double getLeftVelocity() {
        return periodicIO.leftVelocity;
    }

    public double getRightVelocity() {
        return periodicIO.rightVelocity;
    }

    public boolean hasInitialized() {
        return initialized;
    }

    public boolean getClimbLimitSwitch() {
        return periodicIO.climbLimitSwitch;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(periodicIO.leftVelocity, periodicIO.rightVelocity);
    }

    public boolean isShifted() {
        return shifted;
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (periodicIO.leftPosition + periodicIO.rightPosition) / 2.0;
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
