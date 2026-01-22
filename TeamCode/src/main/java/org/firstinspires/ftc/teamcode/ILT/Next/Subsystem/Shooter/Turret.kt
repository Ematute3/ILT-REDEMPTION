package org.firstinspires.ftc.teamcode.robot.subsystems.shooter

import com.qualcomm.robotcore.hardware.AnalogInput
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
 * Turret subsystem for aiming.
 * Supports both relative (motor encoder) and absolute encoder feedback.
 */
object Turret : Subsystem {

    private lateinit var motor: MotorEx
    private var absoluteEncoder: AnalogInput? = null

    private var encoderOffset = 0.0
    private var isInitialized = false
    private var hasAbsoluteEncoder = false

    var controller = controlSystem {
        posPid(RobotConfig.TurretConfig.pid)
    }

    // Manual control power (set by gamepad)
    var manualPower = 0.0

    override fun initialize() {
        try {
            motor = MotorEx(RobotConfig.Hardware.TURRET_MOTOR)
            isInitialized = true

            // Try to initialize absolute encoder (optional)
            try {
                absoluteEncoder = ActiveOpMode.hardwareMap.get(
                    AnalogInput::class.java,
                    RobotConfig.Hardware.TURRET_ENCODER
                )
                hasAbsoluteEncoder = true
                ActiveOpMode.telemetry.addData("Turret Encoder", "Found")
            } catch (e: Exception) {
                hasAbsoluteEncoder = false
                ActiveOpMode.telemetry.addData("Turret Encoder", "Not found (using motor encoder)")
            }
        } catch (e: Exception) {
            ActiveOpMode.telemetry.addData("Turret Error", e.message)
            isInitialized = false
        }
    }

    override fun periodic() {
        if (!isInitialized) return

        // Update state
        RobotState.turretYaw = getRelativeYaw()
        if (hasAbsoluteEncoder) {
            RobotState.turretAbsoluteYaw = getAbsoluteYaw()
        }

        // Telemetry
        ActiveOpMode.telemetry.run {
            addData("Turret Yaw", "%.1f°".format(Math.toDegrees(RobotState.turretYaw)))
            addData("Turret Goal", "%.1f°".format(Math.toDegrees(controller.goal.position)))
            if (hasAbsoluteEncoder) {
                addData("Absolute Yaw", "%.1f°".format(Math.toDegrees(RobotState.turretAbsoluteYaw)))
            }
            addData("Turret Aligned", RobotState.turretAligned)
        }
    }

    // ==================== YAW FUNCTIONS ====================

    /**
     * Get yaw from motor encoder (relative).
     */
    fun getRelativeYaw(): Double {
        if (!isInitialized) return 0.0
        return normalizeAngle(motor.currentPosition * RobotConfig.TurretConfig.RADIANS_PER_TICK)
    }

    /**
     * Get yaw from absolute encoder.
     */
    fun getAbsoluteYaw(): Double {
        if (!hasAbsoluteEncoder || absoluteEncoder == null) return getRelativeYaw()

        val ratio = absoluteEncoder!!.voltage / absoluteEncoder!!.maxVoltage
        val rawRadians = ratio * 2.0 * PI
        return normalizeAngle(rawRadians - encoderOffset)
    }

    /**
     * Get the best available yaw (absolute if available, else relative).
     */
    fun getYaw(useAbsolute: Boolean = true): Double {
        return if (useAbsolute && hasAbsoluteEncoder) getAbsoluteYaw() else getRelativeYaw()
    }

    // ==================== AIMING FUNCTIONS ====================

    /**
     * Auto-aim toward goal using odometry.
     * @param useAbsolute Whether to use absolute encoder for feedback.
     */
    fun autoAim(useAbsolute: Boolean = false) {
        if (!isInitialized || !RobotState.poseValid) return

        // Calculate angle to goal in field coordinates
        val angleToGoal = atan2(
            RobotState.goalY - RobotState.currentY,
            RobotState.goalX - RobotState.currentX
        )

        // Convert to robot-relative angle
        val robotRelativeAngle = normalizeAngle(angleToGoal - RobotState.currentHeading)

        // Clamp to turret limits
        val targetYaw = robotRelativeAngle.coerceIn(
            RobotConfig.TurretConfig.MIN_ANGLE,
            RobotConfig.TurretConfig.MAX_ANGLE
        )

        // Set goal and calculate output
        controller.goal = KineticState(targetYaw, 0.0)
        val currentYaw = getYaw(useAbsolute)
        motor.power = controller.calculate(KineticState(currentYaw, 0.0))

        // Update alignment status
        RobotState.turretAligned = abs(currentYaw - targetYaw) < Math.toRadians(2.0)
    }

    /**
     * Auto-aim using Limelight TX (horizontal offset).
     */
    fun autoAimLimelight() {
        if (!isInitialized || !RobotState.limelightHasTarget) return

        // Convert TX to radians (negative because TX is positive when target is to the right)
        val targetOffset = -Math.toRadians(RobotState.limelightTx)

        // Clamp
        val clampedOffset = targetOffset.coerceIn(
            -Math.toRadians(RobotConfig.LimelightConfig.maxTurretOffsetDeg),
            Math.toRadians(RobotConfig.LimelightConfig.maxTurretOffsetDeg)
        )

        // Current yaw + offset = new target
        val currentYaw = getYaw()
        val targetYaw = normalizeAngle(currentYaw + clampedOffset)

        controller.goal = KineticState(targetYaw, 0.0)
        motor.power = controller.calculate(KineticState(currentYaw, 0.0))

        // Update alignment status
        RobotState.turretAligned = abs(RobotState.limelightTx) < RobotConfig.LimelightConfig.alignmentToleranceDeg
    }

    /**
     * Go to a specific yaw angle.
     */
    fun goToYaw(yaw: Double, useAbsolute: Boolean = false) {
        if (!isInitialized) return

        val currentYaw = getYaw(useAbsolute)
        controller.goal = KineticState(yaw, 0.0)
        motor.power = controller.calculate(KineticState(currentYaw, 0.0))
    }

    /**
     * Set motor power directly (for manual control).
     */
    fun setManualPower(power: Double) {
        if (!isInitialized) return
        motor.power = power
    }

    // ==================== UTILITY FUNCTIONS ====================

    /**
     * Normalize angle to [-PI, PI].
     */
    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    /**
     * Calibrate absolute encoder (set current position as zero).
     */
    fun calibrateAbsoluteEncoder() {
        if (hasAbsoluteEncoder && absoluteEncoder != null) {
            val ratio = absoluteEncoder!!.voltage / absoluteEncoder!!.maxVoltage
            encoderOffset = ratio * 2.0 * PI
            ActiveOpMode.telemetry.addData("Encoder Calibrated", "Offset: %.3f".format(encoderOffset))
        }
    }

    /**
     * Check if absolute encoder is available.
     */
    fun hasAbsoluteEncoder(): Boolean = hasAbsoluteEncoder

    // ==================== COMMANDS ====================

    val zeroMotorEncoder = InstantCommand {
        if (isInitialized) {
            motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

    val calibrateEncoder = InstantCommand {
        calibrateAbsoluteEncoder()
    }

    val spinLeft = InstantCommand { manualPower = RobotConfig.TurretConfig.manualPowerFast }
    val spinRight = InstantCommand { manualPower = -RobotConfig.TurretConfig.manualPowerFast }
    val nudgeLeft = InstantCommand { manualPower = RobotConfig.TurretConfig.manualPowerSlow }
    val nudgeRight = InstantCommand { manualPower = -RobotConfig.TurretConfig.manualPowerSlow }
    val stopTurret = InstantCommand { manualPower = 0.0 }
}