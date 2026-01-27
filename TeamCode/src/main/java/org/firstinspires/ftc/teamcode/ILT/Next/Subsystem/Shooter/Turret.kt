package org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter

import com.qualcomm.robotcore.hardware.DcMotor
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.*

object Turret : Subsystem{
    enum class State { IDLE, MANUAL, LIMELIGHT, ODOMETRY, FUSED }

    var motor = MotorEx(RobotConfig.Hardware.TURRET_MOTOR)

    // Using your high-strength PID for the base Odo movement
    var controller = controlSystem {
        posPid(RobotConfig.TurretConfig.pid)
    }

    var manualPower = 0.0
    var currentState = State.IDLE

    // ==================== TUNING PARAMETERS ====================
    @JvmField var minPower: Double = 0.15
    @JvmField var maxPower: Double = 0.75
    @JvmField var alignmentTolerance: Double = 1.0

    // NEW: Limelight Influence Gain (0.0 to 1.0)
    // Lower this if it still oscillates! 0.3 means Limelight moves the turret gently.
    @JvmField var visionGain: Double = 0.4

    private const val RADIANS_PER_TICK = 2.0 * PI /
            (RobotConfig.TurretConfig.MOTOR_TICKS_PER_REV * RobotConfig.TurretConfig.GEAR_RATIO)

    override fun initialize() {
        motor.motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    override fun periodic() {
        RobotState.turretYaw = getYaw()

        when (currentState) {
            State.IDLE -> motor.power = 0.0
            State.MANUAL -> motor.power = manualPower.coerceIn(-maxPower, maxPower)
            State.LIMELIGHT -> aimWithLimelightOnly()
            State.ODOMETRY -> aimWithOdometryOnly()
            State.FUSED -> fusedAimingLogic()
        }
    }
    @JvmField val threshold = 0.1

    private fun fusedAimingLogic() {
        if (!RobotState.poseValid) {
            motor.power = 0.0
            return
        }

        // 1. BASE ODO TARGET
        val deltaX = RobotState.goalX - RobotState.currentX
        val deltaY = RobotState.goalY - RobotState.currentY
        val fieldAngle = atan2(deltaY, deltaX)

        // Ensure headings are consistent
        val robotHeading = normalizeAngle(RobotState.currentHeading)
        var targetYaw = normalizeAngle(fieldAngle - robotHeading)

        // 2. LIMELIGHT CORRECTION
        if (RobotState.limelightHasTarget) {
            val tx = RobotState.limelightTx

            // REDUCED THRESHOLD: 0.1 degrees is a much safer "effective zero"


            if (abs(tx) > threshold) {
                // Apply a P-loop style correction
                val visionCorrection = Math.toRadians(tx) * visionGain
                targetYaw = normalizeAngle(targetYaw + visionCorrection)
            }
        }

        applyControl(targetYaw)
    }

    private fun aimWithOdometryOnly() {
        if (!RobotState.poseValid) return
        val deltaX = RobotState.goalX - RobotState.currentX
        val deltaY = RobotState.goalY - RobotState.currentY
        val fieldAngle = atan2(deltaY, deltaX)
        val robotHeading = if (abs(RobotState.currentHeading) > 2.0 * PI)
            Math.toRadians(RobotState.currentHeading) else RobotState.currentHeading

        applyControl(normalizeAngle(fieldAngle - robotHeading))
    }

    private fun aimWithLimelightOnly() {
        if (!RobotState.limelightHasTarget) {
            motor.power = 0.0
            return
        }
        val tx = RobotState.limelightTx
        val currentYaw = getYaw()
        // FIXED: Using (+) for the math to ensure it moves TOWARD target
        val targetYaw = normalizeAngle(currentYaw + Math.toRadians(tx))

        applyControl(targetYaw)
    }

    private fun applyControl(targetYaw: Double) {
        val clampedTarget = targetYaw.coerceIn(-3 * PI / 4, 3 * PI / 4)
        val currentYaw = getYaw()

        controller.goal = KineticState(clampedTarget, 0.0)
        var power = controller.calculate(KineticState(currentYaw, 0.0))

        // Friction Kick
        val errorDeg = Math.toDegrees(abs(clampedTarget - currentYaw))
        if (errorDeg > 0.25) {
            power += (if (power >= 0) 1.0 else -1.0) * minPower
        } else {
            power = 0.0
        }

        motor.power = power.coerceIn(-maxPower, maxPower)

        val visionCentered = !RobotState.limelightHasTarget || abs(RobotState.limelightTx) < 1.5
        RobotState.turretAligned = errorDeg < alignmentTolerance && visionCentered
    }

    fun getYaw(): Double = normalizeAngle(motor.currentPosition * RADIANS_PER_TICK)
    fun getYawDegrees(): Double = Math.toDegrees(getYaw())

    fun normalizeAngle(radians: Double): Double {
        var angle = radians % (2.0 * PI)
        if (angle <= -PI) angle += 2.0 * PI
        if (angle > PI) angle -= 2.0 * PI
        return angle
    }

    fun aimWithLimelight() { currentState = State.LIMELIGHT }
    fun aimWithOdometry() { currentState = State.ODOMETRY }
    fun aimWithBoth() { currentState = State.FUSED }
    fun stop() { currentState = State.IDLE; motor.power = 0.0 }
}