package org.firstinspires.ftc.teamcode.vidar.runtime;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.vidar.VidarAlliance;
import org.firstinspires.ftc.teamcode.vidar.VidarConfig;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarFusionEngine;
import org.firstinspires.ftc.teamcode.vidar.api.VidarDiagnostics;
import org.firstinspires.ftc.teamcode.vidar.config.VidarRobotConfig;
import org.firstinspires.ftc.teamcode.vidar.config.VidarSeasonConfig;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarCorrectedFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarObservationFrame;
import org.firstinspires.ftc.teamcode.vidar.frame.VidarSpatialSnapshot;
import org.firstinspires.ftc.teamcode.vidar.fusion.FieldPoseContext;
import org.firstinspires.ftc.teamcode.vidar.geometry.VidarCalibrationDiagnostics;
import org.firstinspires.ftc.teamcode.vidar.fusion.VidarVisionFusion;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarGlobalVisionWorker;
import org.firstinspires.ftc.teamcode.vidar.schedule.VidarResourceBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.TagDecodeBudget;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagConfig;
import org.firstinspires.ftc.teamcode.vidar.tag.VidarTagDecodeWorker;
import org.firstinspires.ftc.teamcode.vidar.world.VidarWorldModel;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

/**
 * Process-scoped ViDAR perception runtime — survives camera attach/detach across OpModes.
 *
 * <p>Owns world model, fusion, background workers, and snapshot publication. FTC VisionPortal
 * resources live in {@link VidarVisionAttachment} and are torn down on {@link #detachVision()}.
 */
public final class VidarRuntime {

    private static final Object LOCK = new Object();
    private static volatile VidarRuntime instance;

    private final VidarWorldModel world;
    private final FieldPoseContext fieldPoseContext;
    private final TagDecodeBudget tagDecodeBudget;
    private final VidarTagDecodeWorker tagDecodeWorker;
    private final VidarResourceBudget resourceBudget;
    private final VidarObservationWorker observationWorker;
    private final AtomicReference<VidarSpatialSnapshot> publishedSnapshot =
            new AtomicReference<>(VidarSpatialSnapshot.empty());
    private final AtomicReference<VidarObservationFrame> publishedFrame =
            new AtomicReference<>(VidarObservationFrame.empty());

    private RuntimeBootstrap bootstrap;
    private VidarDiagnostics.ConfigSource configSource;
    private VidarFusionEngine fusionEngine;
    private VidarVisionAttachment attachment;

    private VidarRuntime(RuntimeBootstrap bootstrap) {
        this.bootstrap = bootstrap;
        this.configSource = bootstrap.configSource;
        this.fieldPoseContext = new FieldPoseContext(bootstrap.odomSupplier);
        this.tagDecodeBudget = new TagDecodeBudget();
        this.tagDecodeWorker = createTagDecodeWorker();
        this.resourceBudget = new VidarResourceBudget();
        this.world = new VidarWorldModel(bootstrap.odomSupplier, null);
        world.setFieldPoseSupplier(fieldPoseContext.worldTrackFieldPoseSupplier());
        this.observationWorker = new VidarObservationWorker(this::observationTick);
        this.observationWorker.start();
    }

    public static VidarRuntime getOrCreate(RuntimeBootstrap bootstrap) {
        synchronized (LOCK) {
            if (instance == null) {
                instance = new VidarRuntime(bootstrap);
            } else {
                instance.applyBootstrap(bootstrap);
            }
            return instance;
        }
    }

    public static VidarRuntime getInstance() {
        return instance;
    }

    public static void shutdown() {
        synchronized (LOCK) {
            if (instance != null) {
                instance.shutdownInternal();
                instance = null;
            }
        }
    }

    public synchronized void attachVision(
            HardwareMap hardwareMap,
            VidarRobotConfig robot,
            VidarSeasonConfig season) {
        if (attachment != null) {
            detachVision();
        }
        configSource = bootstrap.configSource;
        tagDecodeBudget.reset();
        VidarVisionAttachment newAttachment = null;
        VidarFusionEngine engine = null;
        try {
            newAttachment = VidarVisionAttachment.attach(
                    hardwareMap,
                    robot,
                    season,
                    bootstrap.odomSupplier,
                    tagDecodeBudget,
                    tagDecodeWorker,
                    resourceBudget);
            engine = VidarFusionEngine.create(
                    newAttachment.cameras(),
                    newAttachment.cameraCount(),
                    newAttachment.robotConfig(),
                    newAttachment.season(),
                    bootstrap.odomSupplier,
                    bootstrap.allianceSupplier,
                    tagDecodeBudget,
                    resourceBudget);
            fieldPoseContext.bindVision(engine);
            attachment = newAttachment;
            fusionEngine = engine;
        } catch (RuntimeException ex) {
            if (engine != null) {
                engine.resetMatchState();
            }
            if (newAttachment != null) {
                newAttachment.close();
            }
            fieldPoseContext.bindVision((VidarVisionFusion) null);
            throw ex;
        }
    }

    public synchronized void detachVision() {
        if (attachment != null) {
            attachment.close();
            attachment = null;
        }
        if (fusionEngine != null) {
            fusionEngine.resetMatchState();
            fusionEngine = null;
        }
        fieldPoseContext.bindVision((VidarVisionFusion) null);
        publishDetachedSnapshot();
    }

    public void resetMatchState() {
        world.resetMatchState();
        if (fusionEngine != null) {
            fusionEngine.resetMatchState();
        }
        tagDecodeBudget.reset();
    }

    public VidarSpatialSnapshot snapshot() {
        return publishedSnapshot.get();
    }

    public VidarObservationFrame lastFrame() {
        return publishedFrame.get();
    }

    /** Pin the latest published snapshot for one robot-loop iteration (legacy {@code update()}). */
    public VidarSpatialSnapshot pinSnapshot() {
        synchronized (this) {
            VidarSpatialSnapshot base = publishedSnapshot.get();
            VidarFusionEngine engine = fusionEngine;
            if (engine == null) {
                return base;
            }
            // Refresh odom + correction fields under the runtime lock so OpMode getters stay
            // consistent for one loop and fuse→now uses the latest follower sample.
            if (fieldPoseContext.odomSupplier() != null) {
                engine.recordOdom(fieldPoseContext.odomSupplier().get());
            }
            return base.withCorrections(
                    engine.getFusedFieldPose(),
                    engine.getGatedTagCorrectedFieldPoseNow(),
                    engine.lastTagCorrectionNanos());
        }
    }

    public VidarCorrectedFrame updateCorrected() {
        synchronized (this) {
            if (fusionEngine == null) {
                return VidarCorrectedFrame.from(VidarObservationFrame.empty(), null, null);
            }
            if (fieldPoseContext.odomSupplier() != null) {
                fusionEngine.recordOdom(fieldPoseContext.odomSupplier().get());
            }
            VidarCorrectedFrame corrected = fusionEngine.updateCorrected();
            world.update(fusionEngine, System.nanoTime());
            publishNow();
            return corrected;
        }
    }

    public void setFieldPoseSupplier(Supplier<Pose2D> supplier) {
        fieldPoseContext.setExternalFieldPoseSupplier(supplier);
    }

    public void setFieldPosePrior(Pose2D prior) {
        if (fusionEngine != null) {
            fusionEngine.setFieldPosePrior(prior);
        }
    }

    public void setMotionTrackingEnabled(boolean enabled) {
        world.setMotionTrackingEnabled(enabled);
    }

    public VidarFusionEngine fusionEngine() {
        return fusionEngine;
    }

    public VidarCalibrationDiagnostics calibrationDiagnostics() {
        return fusionEngine == null ? null : fusionEngine.calibrationDiagnostics();
    }

    public VidarWorldModel world() {
        return world;
    }

    public FieldPoseContext fieldPoseContext() {
        return fieldPoseContext;
    }

    public TagDecodeBudget tagDecodeBudget() {
        return tagDecodeBudget;
    }

    public VidarTagDecodeWorker tagDecodeWorker() {
        return tagDecodeWorker;
    }

    public VidarDiagnostics.ConfigSource configSource() {
        return configSource;
    }

    public VidarObservationWorker observationWorker() {
        return observationWorker;
    }

    public VidarGlobalVisionWorker globalVisionWorker() {
        return attachment == null ? null : attachment.globalVisionWorker();
    }

    public VidarVision camera(int index) {
        return attachment == null ? null : attachment.camera(index);
    }

    public int cameraCount() {
        return attachment == null ? 0 : attachment.cameraCount();
    }

    public boolean isVisionAttached() {
        return attachment != null;
    }

    private void applyBootstrap(RuntimeBootstrap updated) {
        this.bootstrap = updated;
        if (updated.configSource != null) {
            this.configSource = updated.configSource;
        }
        // OpMode recreate must rebind suppliers — FieldPoseContext / WorldModel outlive attachVision.
        fieldPoseContext.setOdomSupplier(updated.odomSupplier);
        world.setOdomSupplier(updated.odomSupplier);
    }

    private void observationTick() {
        VidarFusionEngine engine;
        synchronized (this) {
            engine = fusionEngine;
            if (engine != null) {
                if (fieldPoseContext.odomSupplier() != null) {
                    engine.recordOdom(fieldPoseContext.odomSupplier().get());
                }
                engine.update();
                world.update(engine, System.nanoTime());
            }
        }
        if (engine != null) {
            publishNow();
        } else {
            publishDetachedSnapshot();
        }
    }

    private void publishNow() {
        VidarFusionEngine engine = fusionEngine;
        if (engine == null) {
            publishDetachedSnapshot();
            return;
        }
        publishedFrame.set(engine.getLatestFrame());
        publishedSnapshot.set(VidarSpatialSnapshot.build(
                engine, world, fieldPoseContext::fieldPoseForSnapshot));
    }

    private void publishDetachedSnapshot() {
        publishedFrame.set(VidarObservationFrame.empty());
        publishedSnapshot.set(VidarSpatialSnapshot.build(
                null, world, fieldPoseContext::fieldPoseForSnapshot));
    }

    private void shutdownInternal() {
        detachVision();
        observationWorker.shutdownAndJoin();
        if (tagDecodeWorker != null) {
            tagDecodeWorker.shutdownAndJoin();
        }
    }

    private static VidarTagDecodeWorker createTagDecodeWorker() {
        if (VidarTagConfig.ENABLED && VidarConfig.ASYNC_TAG_DECODE_ENABLED) {
            return new VidarTagDecodeWorker();
        }
        return null;
    }
}
