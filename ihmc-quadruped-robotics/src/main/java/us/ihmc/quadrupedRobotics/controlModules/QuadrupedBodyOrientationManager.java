package us.ihmc.quadrupedRobotics.controlModules;

import us.ihmc.euclid.Axis;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedBodyOrientationController;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.quadrupedRobotics.providers.QuadrupedPostureInputProviderInterface;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.ParameterizedPID3DGains;
import us.ihmc.robotics.referenceFrames.OrientationFrame;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedBodyOrientationManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedBodyOrientationController.Setpoints setpoints = new QuadrupedBodyOrientationController.Setpoints();
   private final QuadrupedBodyOrientationController controller;
   private final YoPID3DGains gains;

   private final ParameterizedPID3DGains bodyOrientationGainsParameter;
   private final RigidBody body;

   private final QuadrupedPostureInputProviderInterface postureProvider;
   private final GroundPlaneEstimator groundPlaneEstimator;

   private final FrameQuaternion bodyOrientationReference;
   private final OrientationFrame bodyOrientationReferenceFrame;

   private final QuadrupedForceControllerToolbox controllerToolbox;

   private final FrameVector3D desiredBodyAngularAcceleration = new FrameVector3D();
   private final OrientationFeedbackControlCommand orientationFeedbackControlCommand = new OrientationFeedbackControlCommand();

   private final Wrench wrenchCommand;
   private final VirtualWrenchCommand virtualWrenchCommand = new VirtualWrenchCommand();

   private final YoFrameVector bodyAngularWeight = new YoFrameVector("bodyAngularWeight", worldFrame, registry);

   public QuadrupedBodyOrientationManager(QuadrupedForceControllerToolbox controllerToolbox, QuadrupedPostureInputProviderInterface postureProvider,
                                          YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.postureProvider = postureProvider;

      DefaultPID3DGains bodyOrientationDefaultGains = new DefaultPID3DGains();
      bodyOrientationDefaultGains.setProportionalGains(5000.0, 5000.0, 5000.0);
      bodyOrientationDefaultGains.setDerivativeGains(750.0, 750.0, 750.0);
      bodyOrientationDefaultGains.setIntegralGains(0.0, 0.0, 0.0, 0.0);
      bodyOrientationGainsParameter = new ParameterizedPID3DGains("_bodyOrientation", GainCoupling.NONE, false, bodyOrientationDefaultGains, registry);

      controller = new QuadrupedBodyOrientationController(controllerToolbox, registry);
      groundPlaneEstimator = controllerToolbox.getGroundPlaneEstimator();
      gains = controller.getGains();

      bodyOrientationReference = new FrameQuaternion();
      bodyOrientationReferenceFrame = new OrientationFrame(bodyOrientationReference);

      FullQuadrupedRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      orientationFeedbackControlCommand.set(fullRobotModel.getElevator(), fullRobotModel.getBody());
      bodyAngularWeight.set(5.0, 5.0, 5.0);

      body = fullRobotModel.getBody();
      wrenchCommand = new Wrench(body.getBodyFixedFrame(), controllerToolbox.getReferenceFrames().getCenterOfMassFrame());

      parentRegistry.addChild(registry);
   }

   public void initialize(FrameQuaternionReadOnly bodyOrientationEstimate)
   {
      setpoints.initialize(bodyOrientationEstimate);
      controller.reset();
   }

   public void compute(FrameVector3D angularMomentumRateToPack, FrameQuaternionReadOnly bodyOrientationDesired)
   {
      gains.set(bodyOrientationGainsParameter);

      bodyOrientationReference.setIncludingFrame(bodyOrientationDesired);
      bodyOrientationReference.changeFrame(bodyOrientationReferenceFrame.getParent());
      bodyOrientationReferenceFrame.setOrientationAndUpdate(bodyOrientationReference);

      setpoints.getBodyOrientation().changeFrame(bodyOrientationReferenceFrame);
      setpoints.getBodyOrientation().set(postureProvider.getBodyOrientationInput());
      setpoints.getBodyOrientation().changeFrame(worldFrame);
      double bodyOrientationYaw = setpoints.getBodyOrientation().getYaw();
      double bodyOrientationPitch = setpoints.getBodyOrientation().getPitch() + groundPlaneEstimator.getPitch(bodyOrientationYaw);
      double bodyOrientationRoll = setpoints.getBodyOrientation().getRoll();
      setpoints.getBodyOrientation().setYawPitchRoll(bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll);

      setpoints.getBodyAngularVelocity().set(postureProvider.getBodyAngularRateInput());
      setpoints.getComTorqueFeedforward().setToZero();

      controller.compute(angularMomentumRateToPack, setpoints, controllerToolbox.getTaskSpaceEstimates().getBodyAngularVelocity());

      orientationFeedbackControlCommand.set(setpoints.getBodyOrientation(), setpoints.getBodyAngularVelocity(), desiredBodyAngularAcceleration);
      orientationFeedbackControlCommand.setWeightsForSolver(bodyAngularWeight);
      orientationFeedbackControlCommand.setGains(gains);

      wrenchCommand.setToZero();
      wrenchCommand.setAngularPart(angularMomentumRateToPack);
      virtualWrenchCommand.set(body, wrenchCommand);
   }

   public FeedbackControlCommand<?> createFeedbackControlTemplate()
   {
      return getFeedbackControlCommand();
   }

   public OrientationFeedbackControlCommand getFeedbackControlCommand()
   {
      return orientationFeedbackControlCommand;
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand()
   {
      return virtualWrenchCommand;
   }
}
