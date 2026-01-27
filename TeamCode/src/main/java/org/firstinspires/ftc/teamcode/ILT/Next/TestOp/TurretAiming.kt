package org.firstinspires.ftc.teamcode.ILT.Next.TestOp

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.ActiveOpMode
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig.alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Vision.Limelight
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2

@TeleOp(name = "Test - Turret Aiming (V2)", group = "Test")
class TurretAimingTestOpMode : NextFTCOpMode() {

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(Turret, Limelight),
            BulkReadComponent,
            BindingsComponent
        )
    }

    private enum class AimMode { OFF, FUSED, LIMELIGHT_ONLY, ODOMETRY_ONLY }
    private var currentMode = AimMode.OFF

    override fun onInit() {
        RobotState.poseValid = false
        telemetry.addLine("=== TURRET AIMING TEST ===")
        telemetry.addLine("X - FUSED | Circle - LL | Triangle - ODO | Square - STOP")
        telemetry.update()
    }

    override fun onStartButtonPressed() {
        currentMode = AimMode.OFF
        alliance = Alliance.RED

        // Initialize Pose at (12,12) to ensure a valid vector to goal
        follower.pose = Pose(0.0, 0.0, 0.0)



        // Setup driver control
        PedroDriverControlled(
            { -gamepad1.left_stick_y.toDouble() },
            { -gamepad1.left_stick_x.toDouble() },
            { -gamepad1.right_stick_x.toDouble() },
            true // Field Centric
        ).schedule()
    }

    override fun onUpdate() {
        // --- BRIDGE DATA ---
        RobotState.currentX = follower.pose.x
        RobotState.currentY = follower.pose.y

        // Ensure heading is in Radians
        val rawHeading = follower.pose.heading
        RobotState.currentHeading = if (abs(rawHeading) > 2.0 * PI) Math.toRadians(rawHeading) else rawHeading

        RobotState.poseValid = true

        // Reset Heading
        if (gamepad1.options) {
            follower.pose = Pose(follower.pose.x, follower.pose.y, 0.0)
        }

        // --- CONTROLS ---
        if (gamepad1.x) currentMode = AimMode.LIMELIGHT_ONLY
        if(gamepad1.circle) currentMode = AimMode.FUSED
        if (gamepad1.triangle) currentMode = AimMode.ODOMETRY_ONLY
        if (gamepad1.square) currentMode = AimMode.OFF

        // --- EXECUTE ---
        when (currentMode) {
            AimMode.OFF -> Turret.stop()
            AimMode.FUSED -> Turret.aimWithBoth()
            AimMode.LIMELIGHT_ONLY -> Turret.aimWithLimelight()
            AimMode.ODOMETRY_ONLY -> Turret.aimWithOdometry()
        }

        // --- TELEMETRY ---
        ActiveOpMode.telemetry.run {
            addLine("=== ACTIVE MODE: ${currentMode.name} ===")

            addLine("--- SENSORS ---")
            addData("Odo Pose", "(%.1f, %.1f)".format(RobotState.currentX, RobotState.currentY))
            addData("Heading", "%.1f°".format(Math.toDegrees(RobotState.currentHeading)))
            addData("LL Target", if (RobotState.limelightHasTarget) "LOCKED" else "LOST")
            if (RobotState.limelightHasTarget) {
                addData("LL TX", "%.2f°".format(RobotState.limelightTx))
            }

            addLine("--- MATH ---")
            val dx = RobotState.goalX - RobotState.currentX
            val dy = RobotState.goalY - RobotState.currentY
            val fieldAngle = Math.toDegrees(atan2(dy, dx))
            addData("Goal Field Angle", "%.1f°".format(fieldAngle))

            addLine("--- TURRET ---")
            addData("Current Angle", "%.1f°".format(Turret.getYawDegrees()))
            addData("Target Angle", "%.1f°".format(Math.toDegrees(Turret.controller.goal.position)))
            addData("Motor Power", "%.3f".format(Turret.motor.power))
            addData("Aligned", RobotState.turretAligned)

            update()
        }
    }
}