package org.firstinspires.ftc.teamcode.hal;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Robot donanimi. Tek yerden Follower kuruyor.
 * Motor isimleri ve odometri offsetleri robot yapilinca guncellenecek.
 */
public final class Hardware {

    public final Follower follower;

    public Hardware(HardwareMap hardwareMap, Pose startPose) {
        Mecanum drivetrain = new Mecanum(hardwareMap, new MecanumConfig(cfg -> {
            cfg.frontLeftName.set("fl");
            cfg.backLeftName.set("bl");
            cfg.frontRightName.set("fr");
            cfg.backRightName.set("br");
            cfg.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            cfg.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
            cfg.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
            cfg.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        }));

        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, new PinpointConfig(cfg -> {
            cfg.name.set("pinpoint");
            cfg.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            // Gecen sezonun OLCULMUS degerleri (Pedro 2.x Constants.java'dan tasindi):
            //   forwardPodY(161) strafePodX(0), forward FORWARD, strafe REVERSED, birim MM.
            // Pedro 3 karsiligi: xPod = ileri pod, yPod = yanal pod.
            // YENI ROBOTTA YENIDEN OLCULECEK - su anki sasi degil.
            cfg.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            cfg.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            cfg.offsetUnits.set(DistanceUnit.MM);
            cfg.xPodOffset.set(161.0);
            cfg.yPodOffset.set(0.0);
        }));

        follower = new Follower(localizer, drivetrain, new Foresight(new ForesightConfig(cfg -> {})));
        follower.setPose(startPose);
    }
}
