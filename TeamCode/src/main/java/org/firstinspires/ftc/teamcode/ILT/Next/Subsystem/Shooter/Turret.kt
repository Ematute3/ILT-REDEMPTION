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
    private  var motor = MotorEx(RobotConfig.Hardware.TURRET_MOTOR)



    // ==================== CONTROL ====================
    var controller = controlSystem {
        posPid(RobotConfig.TurretConfig.pid)
    }

    // Manual control power (set from TeleOp)
    var manualPower = 0.0
    var currentState = State.IDLE

    // ==================== TUNING PARAMETERS ====================
    // These can be adjusted from FTC Dashboard

    @JvmField var kP_limelight: Double = 0.03    // Proportional gain for LL aiming
    @JvmField var minPower: Double = 0.1         // Min power to overcome friction
    @JvmField var maxPower: Double = 0.6         // Max power limit

    // ==================== CONSTANTS ====================
    private const val RADIANS_PER_TICK = 2.0 * PI /
            (RobotConfig.TurretConfig.MOTOR_TICKS_PER_REV * RobotConfig.TurretConfig.GEAR_RATIO)

    // ==================== INITIALIZATION ====================
    override fun initialize() {


            // IMPORTANT: Reset encoder to 0
            // Assumes turret is physically pointing forward!
            motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER


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
                if (RobotState.limelightHasTarget) {
                    val tx = RobotState.limelightTx
                    if (abs(tx) < RobotConfig.LimelightConfig.alignmentToleranceDeg) {
                        motor.power = 0.0
                        RobotState.turretAligned = true
                    } else {
                        var power = -kP_limelight * tx
                        if (abs(power) < minPower) power = Math.signum(power) * minPower
                        motor.power = power.coerceIn(-maxPower, maxPower)
                        RobotState.turretAligned = false
                    }
                } else {
                    motor.power = 0.0
                }
            }

            State.ODOMETRY -> {
                if (RobotState.poseValid) {
                    val angleToGoal = atan2(
                        RobotState.goalY - RobotState.currentY,
                        RobotState.goalX - RobotState.currentX
                    )
                    val targetYaw = normalizeAngle(angleToGoal - RobotState.currentHeading)
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
        }
    }

    // ==================== POSITION FUNCTIONS ====================

    /**
     * Get turret yaw in radians.
     * 0 = forward, positive = left, negative = right
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
    /**
     * Get raw encoder ticks (for debugging).
     */
    fun getRawTicks(): Double {
        return  motor.currentPosition
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
     * Drives turret until TX ≈ 0 (aligned with target).
     */
    fun aimWithLimelight() {

        if (!RobotState.limelightHasTarget) {
            motor.power = 0.0
            RobotState.turretAligned = false
            return
        }

        val tx = RobotState.limelightTx

        // Check if aligned
        if (abs(tx) < RobotConfig.LimelightConfig.alignmentToleranceDeg) {
            motor.power = 0.0
            RobotState.turretAligned = true
            return
        }

        // Proportional control: power = -kP * tx
        // Negative because positive TX means target is right, so turn right (negative)
        var power = -kP_limelight * tx

        // Add minimum power to overcome static friction
        if (power > 0 && power < minPower) power = minPower
        else if (power < 0 && power > -minPower) power = -minPower

        motor.power = power.coerceIn(-maxPower, maxPower)
        RobotState.turretAligned = false
    }

    /**
     * Aim using odometry (robot position).
     * Calculates angle to goal and rotates turret.
     */
    fun aimWithOdometry() {
        if (!RobotState.poseValid) {
            motor.power = 0.0
            RobotState.turretAligned = false
            return
        }

        // Calculate field angle to goal
        val angleToGoal = atan2(
            RobotState.goalY - RobotState.currentY,
            RobotState.goalX - RobotState.currentX
        )

        // Convert to robot-relative (turret) angle
        val targetYaw = normalizeAngle(angleToGoal - RobotState.currentHeading)

        // Clamp to turret limits
        val clampedTarget = targetYaw.coerceIn(
            RobotConfig.TurretConfig.MIN_ANGLE,
            RobotConfig.TurretConfig.MAX_ANGLE
        )

        // PID control to target
        controller.goal = KineticState(clampedTarget, 0.0)
        val currentYaw = getYaw()
        motor.power = controller.calculate(KineticState(currentYaw, 0.0))

        // Update alignment status
        RobotState.turretAligned = abs(currentYaw - clampedTarget) < Math.toRadians(2.0)
    }

    /**
     * Aim using BOTH systems.
     * Uses Limelight when target visible, odometry otherwise.
     */
    fun aimWithBoth() {


        if (RobotState.limelightHasTarget) {
            aimWithLimelight()
        } else if (RobotState.poseValid) {
            aimWithOdometry()
        } else {
            motor.power = 0.0
            RobotState.turretAligned = false
        }
    }

    /**
     * Go to a specific angle (radians).
     */
    fun goToYaw(yawRadians: Double) {


        controller.goal = KineticState(yawRadians, 0.0)
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

        motor.power = power.coerceIn(-maxPower, maxPower)
    }

    /**
     * Stop the turret.
     */
    fun stop() {

        motor.power = 0.0
    }

    // ==================== COMMANDS ====================

    // Zero the encoder (call when turret is physically forward)
    val zeroTurret = InstantCommand { resetEncoderToZero() }

    // Manual control
    val spinLeft = InstantCommand { manualPower = RobotConfig.TurretConfig.manualPowerFast }
    val spinRight = InstantCommand { manualPower = -RobotConfig.TurretConfig.manualPowerFast }
    val nudgeLeft = InstantCommand { manualPower = RobotConfig.TurretConfig.manualPowerSlow }
    val nudgeRight = InstantCommand { manualPower = -RobotConfig.TurretConfig.manualPowerSlow }
    val stopTurret = InstantCommand {
        manualPower = 0.0
        stop()
    }

    // Go to preset positions
    val goToCenter = InstantCommand { goToYawDegrees(0.0) }
    val goToLeft45 = InstantCommand { goToYawDegrees(45.0) }
    val goToRight45 = InstantCommand { goToYawDegrees(-45.0) }
}