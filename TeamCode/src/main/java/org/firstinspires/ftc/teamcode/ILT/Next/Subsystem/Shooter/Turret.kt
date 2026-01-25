package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

/**
 * Turret subsystem for aiming left/right.
 *
 * ============================================================
 * SETUP REQUIREMENTS
 * ============================================================
 *
 * Before pressing INIT:
 * 1. Physically align turret to face straight forward
 * 2. Use a mechanical stop, jig, or visual marker
 * 3. On initialize(), encoder resets to 0
 *
 * If turret drifts during match:
 * - Call resetEncoderToZero() when turret is physically forward
 * - Or use the zeroTurret command
 *
 * ============================================================
 * COORDINATE SYSTEM
 * ============================================================
 *
 * Turret angle of 0 = facing straight forward on robot
 * Positive angle = rotated LEFT (counter-clockwise from above)
 * Negative angle = rotated RIGHT (clockwise from above)
 */
object Turret : Subsystem {
    enum class State { IDLE, MANUAL, LIMELIGHT, ODOMETRY }

    // ==================== HARDWARE ====================
    private var motor = MotorEx(RobotConfig.Hardware.TURRET_MOTOR)

    // ==================== CONTROL ====================
    var controller = controlSystem {
        posPid(RobotConfig.TurretConfig.pid)
    }

    // Manual control power (set from TeleOp)
    var manualPower = 0.0
    var currentState = State.IDLE

    // ==================== TUNING PARAMETERS ====================
    // Limelight tracking gains - TUNED FOR STABILITY
    @JvmField var kP_limelight: Double = 0.015   // Reduced from 0.03 to reduce oscillation
    @JvmField var kD_limelight: Double = 0.008   // Added derivative to dampen oscillation
    @JvmField var minPower: Double = 0.08        // Reduced minimum power
    @JvmField var maxPower: Double = 0.1         // Reduced max power for smoother tracking

    // Deadband and tolerance
    @JvmField var limelightDeadband: Double = 1.0     // Ignore small TX values (degrees)
    @JvmField var alignmentTolerance: Double = 2.0    // Consider aligned within this (degrees)

    // Lost target behavior
    @JvmField var searchPower: Double = 0.15          // Power when searching for lost target
    @JvmField var lostTargetTimeout: Long = 500       // ms before starting search

    // ==================== STATE TRACKING ====================
    private var lastTx: Double = 0.0                   // For derivative calculation
    private var lastTargetSeenTime: Long = 0           // Track when we last saw target
    private var searchDirection: Double = 1.0          // Which way to search (-1 or 1)

    // ==================== CONSTANTS ====================
    private const val RADIANS_PER_TICK = 2.0 * PI /
            (RobotConfig.TurretConfig.MOTOR_TICKS_PER_REV * RobotConfig.TurretConfig.GEAR_RATIO)

    // ==================== INITIALIZATION ====================
    override fun initialize() {
        // IMPORTANT: Reset encoder to 0
        // Assumes turret is physically pointing forward!
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        lastTargetSeenTime = System.currentTimeMillis()

        ActiveOpMode.telemetry.addData("Turret", "Initialized - Encoder zeroed")
        ActiveOpMode.telemetry.addData("WARNING", "Make sure turret is facing FORWARD!")
        ActiveOpMode.telemetry.update()
    }

    // ==================== PERIODIC ====================
    override fun periodic() {
        // Update RobotState
        RobotState.turretYaw = getYaw()

        when (currentState) {
            State.IDLE -> motor.power = 0.0

            State.MANUAL -> {
                motor.power = manualPower.coerceIn(-maxPower, maxPower)
            }

            State.LIMELIGHT -> {
                aimWithLimelightImproved()
            }

            State.ODOMETRY -> {
                if (RobotState.poseValid) {
                    val angleToGoal = atan2(
                        RobotState.goalY - RobotState.currentY,
                        RobotState.goalX - RobotState.currentX
                    )

                    // Calculate robot-relative turret angle
                    val targetYaw = normalizeAngle(angleToGoal - RobotState.currentHeading)

                    // Clamp to turret limits
                    val clampedTarget = targetYaw.coerceIn(
                        RobotConfig.TurretConfig.MIN_ANGLE,
                        RobotConfig.TurretConfig.MAX_ANGLE
                    )

                    controller.goal = KineticState(clampedTarget, 0.0)
                    motor.power = controller.calculate(KineticState(getYaw(), 0.0))
                    RobotState.turretAligned = abs(getYaw() - clampedTarget) < Math.toRadians(2.0)
                }
            }
        }

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("=== TURRET ===", "")
            addData("Yaw", "%.1f°".format(Math.toDegrees(RobotState.turretYaw)))
            addData("Raw Ticks", motor.currentPosition)
            addData("Goal", "%.1f°".format(Math.toDegrees(controller.goal.position)))
            addData("Power", "%.2f".format(motor.power))
            addData("Aligned", RobotState.turretAligned)
            addData("State", currentState.name)
            if (currentState == State.LIMELIGHT) {
                addData("LL Target", RobotState.limelightHasTarget)
                addData("LL TX", "%.2f°".format(RobotState.limelightTx))
            }
        }
    }

    // ==================== IMPROVED LIMELIGHT TRACKING ====================

    /**
     * Improved Limelight tracking with PD control, deadband, and lost-target recovery
     */
    private fun aimWithLimelightImproved() {
        if (!RobotState.limelightHasTarget) {
            handleLostTarget()
            return
        }

        // We have a target - update last seen time
        lastTargetSeenTime = System.currentTimeMillis()

        val tx = RobotState.limelightTx

        // Apply deadband - ignore tiny errors
        if (abs(tx) < limelightDeadband) {
            motor.power = 0.0
            RobotState.turretAligned = true
            lastTx = tx
            return
        }

        // Check if aligned within tolerance
        if (abs(tx) < alignmentTolerance) {
            motor.power = 0.0
            RobotState.turretAligned = true
            lastTx = tx
            return
        }

        // PD Control for smooth tracking
        // P term: proportional to error
        val proportional = -kP_limelight * tx

        // D term: resist rapid changes (derivative of error)
        val derivative = -kD_limelight * (tx - lastTx)
        lastTx = tx

        // Combine P and D
        var power = proportional + derivative

        // Add minimum power to overcome static friction (only if moving)
        if (abs(power) > 0.01) {
            if (power > 0 && power < minPower) power = minPower
            else if (power < 0 && power > -minPower) power = -minPower
        }

        // Clamp to max power
        motor.power = power.coerceIn(-maxPower, maxPower)
        RobotState.turretAligned = false
    }

    /**
     * Handle lost target - search in last known direction
     */
    private fun handleLostTarget() {
        val timeSinceLost = System.currentTimeMillis() - lastTargetSeenTime

        if (timeSinceLost < lostTargetTimeout) {
            // Just lost it - hold position briefly
            motor.power = 0.0
        } else {
            // Been lost for a while - slowly search
            // Determine search direction based on last known TX
            if (lastTx > 0) {
                searchDirection = 1.0  // Target was right, keep searching right
            } else {
                searchDirection = -1.0  // Target was left, keep searching left
            }

            // Check if we're at limits
            val currentYaw = getYaw()
            if (currentYaw >= RobotConfig.TurretConfig.MAX_ANGLE - Math.toRadians(5.0)) {
                searchDirection = -1.0  // Reverse search if at right limit
            } else if (currentYaw <= RobotConfig.TurretConfig.MIN_ANGLE + Math.toRadians(5.0)) {
                searchDirection = 1.0   // Reverse search if at left limit
            }

            motor.power = searchPower * searchDirection
        }

        RobotState.turretAligned = false
    }

    // ==================== POSITION FUNCTIONS ====================

    /**
     * Get turret yaw in radians.
     * 0 = forward, positive = left, negative = right
     * NORMALIZED to [-π, π]
     */
    fun getYaw(): Double {
        return normalizeAngle(motor.currentPosition * RADIANS_PER_TICK)
    }

    /**
     * Get turret yaw in degrees (convenience).
     */
    fun getYawDegrees(): Double = Math.toDegrees(getYaw())

    /**
     * Get raw encoder ticks (for debugging).
     */
    fun getRawTicks(): Double {
        return motor.currentPosition
    }

    /**
     * Reset encoder to zero.
     * CALL THIS WHEN TURRET IS PHYSICALLY FACING FORWARD!
     */
    fun resetEncoderToZero() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        ActiveOpMode.telemetry.addData("Turret", "ENCODER RESET TO ZERO")
    }

    /**
     * Normalize angle to [-π, π].
     * This ensures turret always takes shortest path!
     */
    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    // ==================== AIMING FUNCTIONS ====================

    /**
     * Aim using Limelight TX (horizontal offset).
     * Uses improved PD control with deadband and lost-target recovery.
     */
    fun aimWithLimelight() {
        currentState = State.LIMELIGHT
    }

    /**
     * Aim using odometry (robot position).
     * Calculates angle to goal and rotates turret.
     * Uses shortest path via normalizeAngle!
     */
    fun aimWithOdometry() {
        currentState = State.ODOMETRY
    }

    /**
     * Aim using BOTH systems.
     * Uses Limelight when target visible, odometry otherwise.
     */
    fun aimWithBoth() {
        if (RobotState.limelightHasTarget) {
            currentState = State.LIMELIGHT
        } else if (RobotState.poseValid) {
            currentState = State.ODOMETRY
        } else {
            currentState = State.IDLE
        }
    }

    /**
     * Go to a specific angle (radians).
     * USES SHORTEST PATH via normalizeAngle!
     */
    fun goToYaw(yawRadians: Double) {
        // Normalize the target to ensure shortest path
        val normalizedTarget = normalizeAngle(yawRadians)

        controller.goal = KineticState(normalizedTarget, 0.0)
        motor.power = controller.calculate(KineticState(getYaw(), 0.0))
    }

    /**
     * Go to a specific angle (degrees) - convenience.
     */
    fun goToYawDegrees(yawDegrees: Double) {
        goToYaw(Math.toRadians(yawDegrees))
    }

    /**
     * Set manual power directly.
     */
    fun setManualPowerTurret(power: Double) {
        currentState = State.MANUAL
        manualPower = power.coerceIn(-maxPower, maxPower)
    }

    /**
     * Stop the turret.
     */
    fun stop() {
        currentState = State.IDLE
        motor.power = 0.0
    }

    // ==================== COMMANDS ====================

    // Zero the encoder (call when turret is physically forward)
    val zeroTurret = InstantCommand { resetEncoderToZero() }

    // Manual control
    val spinLeft = InstantCommand { setManualPowerTurret(RobotConfig.TurretConfig.manualPowerFast) }
    val spinRight = InstantCommand { setManualPowerTurret(-RobotConfig.TurretConfig.manualPowerFast) }
    val nudgeLeft = InstantCommand { setManualPowerTurret(RobotConfig.TurretConfig.manualPowerSlow) }
    val nudgeRight = InstantCommand { setManualPowerTurret(-RobotConfig.TurretConfig.manualPowerSlow) }
    val stopTurret = InstantCommand { stop() }

    // Go to preset positions (uses shortest path!)
    val goToCenter = InstantCommand { goToYawDegrees(0.0) }
    val goToLeft45 = InstantCommand { goToYawDegrees(45.0) }
    val goToRight45 = InstantCommand { goToYawDegrees(-45.0) }

    // Aiming commands
    val startLimelightTracking = InstantCommand { aimWithLimelight() }
    val startOdometryTracking = InstantCommand { aimWithOdometry() }
    val startHybridTracking = InstantCommand { aimWithBoth() }
}