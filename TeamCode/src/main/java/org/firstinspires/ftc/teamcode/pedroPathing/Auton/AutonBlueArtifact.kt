package org.firstinspires.ftc.teamcode.robot.opmodes

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.robot.data.config.RobotState


@Autonomous(name = "Main Autonomous", group = "Competition")
class MainAutonomous : NextFTCOpMode() {

    private lateinit var paths: Paths

    init {
        addComponents(
            PedroComponent(Constants::createFollower),
            SubsystemComponent(DriveTrain,
                FlyWheel, Intake, Turret, Hood, Limelight, OuttakeController, Gate
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        RobotConfig.alliance = Alliance.RED
        RobotState.reset()
        follower.setStartingPose(Pose(56.0, 8.0, Math.toRadians(90.0)))
        paths = Paths(follower)
    }

    override fun onStartButtonPressed() {
        // Build the autonomous sequence
        val autoSequence = SequentialGroup(
            // Cycle 1: Intake 1 -> Score
            intakeSequence(paths.intake1),
            Delay(2.0),
            scoreSequence(paths.scorePose1),
            Delay(2.0),

            // Cycle 2: Pre-Intake 2 -> Intake 2 -> Score
            FollowPath(paths.preintake2),
            Delay(2.0),
            intakeSequence(paths.intake2),
            Delay(2.0),
            scoreSequence(paths.scorePose2),
            Delay(2.0),

            // Cycle 3: Intake 3 -> Score
            intakeSequence(paths.intake3),
            Delay(2.0),
            scoreSequence(paths.scorePose3)
        )

        autoSequence.schedule()
    }

    /**
     * Creates a command sequence for intake paths
     * - Runs intake while following the path
     */
    private fun intakeSequence(path: PathChain): Command {
        return ParallelGroup(
            FollowPath(path),
            Intake.run
        )
    }

    /**
     * Creates a command sequence for scoring
     * - Follows path to score position
     * - Spins up flywheel
     * - Opens gate
     * - Waits briefly for ball to shoot
     * - Stops all systems
     */
    private fun scoreSequence(path: PathChain): Command {
        return SequentialGroup(
            // Follow path while aiming turret
            ParallelGroup(
                FollowPath(path),
                InstantCommand{Turret.aimWithBoth()}
            ),

            // Spin up flywheel and wait for it to reach speed
            FlyWheel.spin,
            Delay(0.5), // Wait for flywheel to reach target velocity

            // Open gate to shoot
            ParallelGroup(
                Gate.open,
                Intake.run
            ),

            // Wait for ball to exit
            Delay(0.3),

            // Stop all systems
            ParallelGroup(
                Gate.close,
                Intake.stop,
                FlyWheel.stop
            )
        )
    }

    override fun onUpdate() {
        // Update robot state from odometry
        RobotState.currentX = follower.pose.x
        RobotState.currentY = follower.pose.y
        RobotState.currentHeading = follower.pose.heading
        RobotState.poseValid = true

        // Telemetry
        telemetry.addData("X", "%.2f".format(follower.pose.x))
        telemetry.addData("Y", "%.2f".format(follower.pose.y))
        telemetry.addData("Heading", "%.2f".format(Math.toDegrees(follower.pose.heading)))
        telemetry.addData("Turret Aligned", RobotState.turretAligned)
        telemetry.addData("Flywheel", if (FlyWheel.isAtTargetVelocity()) "READY" else "SPINNING")
        telemetry.update()
    }

    /**
     * Path definitions for autonomous
     */
    class Paths(follower: Follower) {
        val intake1: PathChain
        val scorePose1: PathChain
        val preintake2: PathChain
        val intake2: PathChain
        val scorePose2: PathChain
        val intake3: PathChain
        val scorePose3: PathChain

        init {
            intake1 = follower
                .pathBuilder()
                .addPath(BezierLine(Pose(56.0, 8.0), Pose(11.0, 7.0)))
                .setLinearHeadingInterpolation(Math.toRadians(90.0), Math.toRadians(180.0))
                .build()

            scorePose1 = follower
                .pathBuilder()
                .addPath(BezierLine(Pose(11.0, 7.0), Pose(56.0, 8.0)))
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(105.0))
                .build()

            preintake2 = follower
                .pathBuilder()
                .addPath(
                    BezierCurve(
                        Pose(56.0, 8.0),
                        Pose(62.0, 35.7),
                        Pose(39.0, 35.7)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(105.0), Math.toRadians(180.0))
                .build()

            intake2 = follower
                .pathBuilder()
                .addPath(
                    BezierLine(Pose(39.0, 35.7), Pose(14.64, 35.7))
                )
                .setTangentHeadingInterpolation()
                .build()

            scorePose2 = follower
                .pathBuilder()
                .addPath(
                    BezierLine(Pose(14.64, 35.7), Pose(56.0, 8.0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180.0), Math.toRadians(105.0))
                .build()

            intake3 = follower
                .pathBuilder()
                .addPath(BezierLine(Pose(56.0, 8.0), Pose(15.2, 7.0)))
                .setLinearHeadingInterpolation(Math.toRadians(105.0), Math.toRadians(180.0))
                .build()

            scorePose3 = follower
                .pathBuilder()
                .addPath(BezierLine(Pose(15.2, 7.0), Pose(56.0, 8.0)))
                .setTangentHeadingInterpolation()
                .build()
        }
    }
}