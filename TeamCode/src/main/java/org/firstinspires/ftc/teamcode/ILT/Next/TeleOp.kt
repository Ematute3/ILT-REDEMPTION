package org.firstinspires.ftc.teamcode.ILT.Next

import com.bylazar.configurables.PanelsConfigurables
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Config.RobotConfig

import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Data.Enums.Alliance
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Drive.DriveTrain
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Gate
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Intake.Intake
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.FlyWheel
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Hood
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.OuttakeController
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Shooter.Turret
import org.firstinspires.ftc.teamcode.ILT.Next.Subsystem.Vision.Limelight
import org.firstinspires.ftc.teamcode.ILT.Next.Commands.ShootCommands
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState
import kotlin.math.PI
import kotlin.math.abs

@TeleOp(name = "Main TeleOp", group = "Competition")
class MainTeleOp : NextFTCOpMode() {

    private val panelsTelemetry = PanelsTelemetry.ftcTelemetry
    private val joinedTelemetry = JoinedTelemetry(telemetry, panelsTelemetry)

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(DriveTrain, FlyWheel, Intake, Turret, Hood, Limelight, OuttakeController, Gate),
            BulkReadComponent, BindingsComponent
        )
    }

    private enum class AimModeTele { OFF, FUSED }
    private var currentMode = AimModeTele.OFF

    override fun onInit() {
        // Register configurables so Panels dashboard syncs: dashboard changes → robot
        PanelsConfigurables.INSTANCE.refreshClass(RobotConfig)
        PanelsConfigurables.INSTANCE.refreshClass(Turret)
        // Don't overwrite alliance here – set it on the Panels dashboard or use gamepad X
        RobotState.reset()
        follower.pose = Pose(0.0, 0.0, 0.0)
    }

    override fun onStartButtonPressed() {
        PedroDriverControlled(
            { -gamepad1.left_stick_y.toDouble() },
            { -gamepad1.left_stick_x.toDouble() },
            { -gamepad1.right_stick_x.toDouble() },
            true // FIXED: Set to true (Matches your smooth Test Op)
        ).schedule()

        bindControls()
    }

    private fun bindControls() {
        // --- DRIVER (GP1) ---
        Gamepads.gamepad1.leftTrigger.greaterThan(0.5) whenBecomesTrue Intake.run whenBecomesFalse Intake.stop
        Gamepads.gamepad1.leftBumper whenBecomesTrue Intake.reverse whenBecomesFalse Intake.stop
        Gamepads.gamepad1.rightTrigger.greaterThan(0.5) whenBecomesTrue FlyWheel.spin whenBecomesFalse FlyWheel.stop

        Gamepads.gamepad1.rightBumper whenBecomesTrue ShootCommands.shoot
        Gamepads.gamepad1.circle whenBecomesTrue ShootCommands.stopAll
        Gamepads.gamepad1.triangle whenBecomesTrue { currentMode = AimModeTele.FUSED }
        Gamepads.gamepad1.square whenBecomesTrue { follower.setPose(Pose(follower.pose.x, follower.pose.y, 0.0)) }

        // --- OPERATOR (GP2) ---
        // Gate Controls
        Gamepads.gamepad1.dpadLeft whenBecomesTrue Gate.open
        Gamepads.gamepad1.dpadRight whenBecomesTrue Gate.close
        Gamepads.gamepad1.x whenBecomesTrue { RobotConfig.alliance = Alliance.BLUE }
    }

    override fun onUpdate() {
        // Sync Odo
        RobotState.currentX = follower.pose.x
        RobotState.currentY = follower.pose.y
        val rawHeading = follower.pose.heading
        RobotState.currentHeading = if (abs(rawHeading) > 2.0 * PI) Math.toRadians(rawHeading) else rawHeading
        RobotState.poseValid = true
        if (gamepad1.dpad_up) {
            RobotState.hoodPosition = (RobotState.hoodPosition + 0.01).coerceAtMost(1.0)
        }
        if (gamepad1.dpad_down) {
            RobotState.hoodPosition = (RobotState.hoodPosition - 0.01).coerceAtLeast(0.0)
        }
        // Turret Logic Execution
        when (currentMode) {
            AimModeTele.OFF -> Turret.stop()
            AimModeTele.FUSED -> Turret.aimWithBoth()
        }

        // --- Manual Overrides ---
        // If the operator touches the sticks, it should probably disable Auto-Aim to prevent fighting


        // Telemetry with Panels
        joinedTelemetry.addData("Mode", currentMode)
        joinedTelemetry.addData("Turret Aligned", RobotState.turretAligned)
        joinedTelemetry.addData("Hood Pos", "%.2f".format(RobotState.hoodPosition))
        joinedTelemetry.addData("Flywheel", if (FlyWheel.isAtTargetVelocity()) "READY" else "SPINNING")
        joinedTelemetry.addData("Pose X", "%.1f".format(RobotState.currentX))
        joinedTelemetry.addData("Pose Y", "%.1f".format(RobotState.currentY))
        joinedTelemetry.addData("Heading", "%.1f°".format(Math.toDegrees(RobotState.currentHeading)))
        joinedTelemetry.addData("Alliance", RobotConfig.alliance.name)
        joinedTelemetry.update()
    }
}