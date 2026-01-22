package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config


import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance

import kotlin.math.PI

/**
 * Centralized robot configuration.
 * All tunable values, constants, and hardware names in one place.
 */
@Configurable
object RobotConfig {

    // ==================== ALLIANCE & FIELD ====================
    @JvmField var alliance = Alliance.RED

    // Field dimensions (inches)
    const val FIELD_SIZE = 144.0

    // Goal positions (will be adjusted based on alliance)
    const val GOAL_Y = FIELD_SIZE - 8.0  // 136.0 inches
    const val RED_GOAL_X = FIELD_SIZE - 6.0  // 138.0 inches
    const val BLUE_GOAL_X = 6.0

    // Robot dimensions
    const val ROBOT_WIDTH = 16.0
    const val ROBOT_LENGTH = 16.0

    // ==================== HARDWARE NAMES ====================
    object Hardware {
        // Drive motors (configured in Pedro)
        const val IMU = "imu"

        // Shooter hardware
        const val FLYWHEEL_1 = "fly1"
        const val FLYWHEEL_2 = "fly2"
        const val TURRET_MOTOR = "turret"
        const val TURRET_ENCODER = "encoder"
        const val HOOD_SERVO = "hood"

        // Intake hardware
        const val INTAKE = "intake"

        const val GATE_SERVO = "gate"

        // Vision
        const val LIMELIGHT = "limelight"
    }

    // ==================== FLYWHEEL CONFIG ====================
    object FlywheelConfig {
        @JvmField var pid = PIDCoefficients(0.0033, 0.0, 0.0)
        @JvmField var feedforward = BasicFeedforwardParameters(1.0 / 2400.0, 0.0, 0.03)

        const val MOTOR_TICKS_PER_REV = 28.0
        const val WHEEL_DIAMETER_IN = 6.0
        const val WHEEL_RADIUS_IN = WHEEL_DIAMETER_IN / 2.0

        @JvmField var velocityTolerance = 50.0  // ticks/sec tolerance for "at speed"
    }

    // ==================== TURRET CONFIG ====================
    object TurretConfig {
        @JvmField var pid = PIDCoefficients(0.011, 0.0, 0.2)

        const val GEAR_RATIO = 3.62068965517  // 105/29
        const val MOTOR_TICKS_PER_REV = 537.7
        val RADIANS_PER_TICK = 2.0 * PI / (MOTOR_TICKS_PER_REV * GEAR_RATIO)

        // Turret limits (radians)
        const val MIN_ANGLE = -PI
        const val MAX_ANGLE = PI

        @JvmField var manualPowerSlow = 0.2
        @JvmField var manualPowerFast = 0.6
    }

    // ==================== HOOD CONFIG ====================
    object HoodConfig {
        const val MIN_POSITION = 0.0
        const val MAX_POSITION = 1.0
        @JvmField var adjustStep = 0.05
    }

    // ==================== LIMELIGHT CONFIG ====================
    object LimelightConfig {
        @JvmField var mountAngleDeg = 10.0           // Tilt angle from horizontal
        @JvmField var lensHeightIn = 12.976          // Height of lens from ground
        @JvmField var goalHeightIn = 39.5            // Height of scoring target

        const val POLL_RATE_HZ = 100
        const val DEFAULT_PIPELINE = 0

        @JvmField var alignmentToleranceDeg = 1.0    // TX tolerance for "aligned"
        @JvmField var maxTurretOffsetDeg = 90.0      // Max turret correction
    }

    // ==================== PHYSICS CONFIG ====================
    object Physics {
        const val GRAVITY_IN_PER_S2 = 386.0
        @JvmField var launchAngleDeg = 34.36
        @JvmField var shooterHeightIn = 12.446
    }

    // ==================== INTAKE CONFIG ====================
    object IntakeConfig {
        @JvmField var intakePower = 1.0
        @JvmField var ejectPower = -1.0
        @JvmField var ejectSlowPower = -0.5
        @JvmField var feedPower = 0.8
    }

    // ==================== GATE CONFIG ====================
    object GateConfig {
        @JvmField var openPosition = 0.0
        @JvmField var closedPosition = 1.0
    }

    // ==================== TIMING CONFIG ====================
    object Timing {
        const val FLYWHEEL_SPINUP_TIMEOUT_MS = 2000L
        const val FEED_DURATION_MS = 500L
        const val SHOOT_SEQUENCE_DELAY_MS = 400L
    }
}