package us.ihmc.humanoidBehaviors.behaviors.complexBehaviors;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElement.PickUpBallBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.coactiveElements.PickUpBallBehaviorCoactiveElementBehaviorSide;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.SearchNearForSphereBehavior.SearchNearState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SphereDetectionBehavior;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.WaitForUserValidationBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.geometry.FrameOrientation;

public class SearchNearForSphereBehavior extends StateMachineBehavior<SearchNearState>
{
   public enum SearchNearState
   {
      LOOK_DOWN, ENABLE_LIDAR, SETUP_LIDAR, CLEAR_LIDAR, SEARCHING, VALIDATING
   }

   private final SphereDetectionBehavior initialSphereDetectionBehavior;
   private final WaitForUserValidationBehavior waitForUserValidationBehavior;
   private final PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement;
   private final boolean requireUserValidation;
   private final HumanoidReferenceFrames referenceFrames;
   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final ReferenceFrame chestCoMFrame;

   public SearchNearForSphereBehavior(YoDouble yoTime, PickUpBallBehaviorCoactiveElementBehaviorSide coactiveElement,
         HumanoidReferenceFrames referenceFrames, FullRobotModel fullRobotModel, CommunicationBridge outgoingCommunicationBridge, boolean requireUserValidation,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super("SearchForSpehereNear", SearchNearState.class, yoTime, outgoingCommunicationBridge);
      this.atlasPrimitiveActions = atlasPrimitiveActions;
      this.referenceFrames = referenceFrames;
      this.coactiveElement = coactiveElement;
      this.requireUserValidation = requireUserValidation;

      initialSphereDetectionBehavior = new SphereDetectionBehavior(outgoingCommunicationBridge, referenceFrames);
      waitForUserValidationBehavior = new WaitForUserValidationBehavior(outgoingCommunicationBridge, coactiveElement.validClicked,
            coactiveElement.validAcknowledged);
      setupStateMachine();
      chestCoMFrame = fullRobotModel.getChest().getBodyFixedFrame();;
      
   }

   private void setupStateMachine()
   {

      // BEND OVER *******************************************

      BehaviorAction<SearchNearState> lookDown = new BehaviorAction<SearchNearState>(SearchNearState.LOOK_DOWN, atlasPrimitiveActions.headTrajectoryBehavior,
            atlasPrimitiveActions.chestTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            //MATH
            Vector3D axis = new Vector3D(0, 1, 0);
            double rotationDownAngle = 1.4;
            AxisAngle desiredAxisAngle = new AxisAngle();
            desiredAxisAngle.set(axis, rotationDownAngle);
            Quaternion desiredHeadQuat = new Quaternion();
            desiredHeadQuat.set(desiredAxisAngle);
            //MESSAGE
            HeadTrajectoryMessage message = new HeadTrajectoryMessage(1, desiredHeadQuat, ReferenceFrame.getWorldFrame(), chestCoMFrame);
            atlasPrimitiveActions.headTrajectoryBehavior.setInput(message);
            //MATH
            FrameOrientation desiredChestOrientation = new FrameOrientation(referenceFrames.getPelvisZUpFrame(), Math.toRadians(30), Math.toRadians(20), 0);
            desiredChestOrientation.changeFrame(ReferenceFrame.getWorldFrame());
            Quaternion chestOrientation = new Quaternion();
            desiredChestOrientation.getQuaternion(chestOrientation);
            //MESSAGE
            ChestTrajectoryMessage chestOrientationPacket = new ChestTrajectoryMessage(4.0, chestOrientation, ReferenceFrame.getWorldFrame(), chestCoMFrame);
            atlasPrimitiveActions.chestTrajectoryBehavior.setInput(chestOrientationPacket);

         }
      };

      //ENABLE LIDAR
      BehaviorAction<SearchNearState> enableLidarTask = new BehaviorAction<SearchNearState>(SearchNearState.ENABLE_LIDAR,
            atlasPrimitiveActions.enableLidarBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            atlasPrimitiveActions.enableLidarBehavior.setLidarState(LidarState.ENABLE_BEHAVIOR_ONLY);
         }
      };

      BehaviorAction<SearchNearState> setLidarMediumRangeTask = new BehaviorAction<SearchNearState>(SearchNearState.SETUP_LIDAR,
            atlasPrimitiveActions.setLidarParametersBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {

            DepthDataFilterParameters param = new DepthDataFilterParameters();
            param.nearScanRadius = 0.6f;
            atlasPrimitiveActions.setLidarParametersBehavior.setInput(param);
         }
      };

      //CLEAR LIDAR POINTS FOR CLEAN SCAN *******************************************
      BehaviorAction<SearchNearState> clearLidarTask = new BehaviorAction<SearchNearState>(SearchNearState.CLEAR_LIDAR,
            atlasPrimitiveActions.clearLidarBehavior);

      //SEARCH FOR BALL *******************************************

      BehaviorAction<SearchNearState> findBallTask = new BehaviorAction<SearchNearState>(SearchNearState.SEARCHING, initialSphereDetectionBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            TextToSpeechPacket p1 = new TextToSpeechPacket("LOOKING FOR BALL");
            sendPacket(p1);
            coactiveElement.searchingForBall.set(true);
            coactiveElement.foundBall.set(false);
            coactiveElement.ballX.set(0);
            coactiveElement.ballY.set(0);
            coactiveElement.ballZ.set(0);

         }
      };

      //

      // Confirm from the user that this is the correct ball *******************************************

      BehaviorAction<SearchNearState> validateBallTask = new BehaviorAction<SearchNearState>(SearchNearState.VALIDATING, waitForUserValidationBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            coactiveElement.searchingForBall.set(false);
            coactiveElement.waitingForValidation.set(true);
            coactiveElement.validAcknowledged.set(false);
            coactiveElement.foundBall.set(false);
            coactiveElement.currentState.set(PickUpBallBehaviorState.WAITING_FOR_USER_CONFIRMATION);

            coactiveElement.ballX.set(initialSphereDetectionBehavior.getBallLocation().getX());
            coactiveElement.ballY.set(initialSphereDetectionBehavior.getBallLocation().getY());
            coactiveElement.ballZ.set(initialSphereDetectionBehavior.getBallLocation().getZ());
            coactiveElement.ballRadius.set(initialSphereDetectionBehavior.getSpehereRadius());

         }
      };

      statemachine.addStateWithDoneTransition(lookDown, SearchNearState.ENABLE_LIDAR);
      statemachine.addStateWithDoneTransition(enableLidarTask, SearchNearState.SETUP_LIDAR);
      statemachine.addStateWithDoneTransition(setLidarMediumRangeTask, SearchNearState.CLEAR_LIDAR);
      statemachine.addStateWithDoneTransition(clearLidarTask, SearchNearState.SEARCHING);

      if (requireUserValidation)
      {
         statemachine.addStateWithDoneTransition(findBallTask, SearchNearState.VALIDATING);
         statemachine.addState(validateBallTask);
      }

      else
         statemachine.addState(findBallTask);
      statemachine.setStartState(SearchNearState.LOOK_DOWN);

   }

   public boolean foundBall()
   {
      return initialSphereDetectionBehavior.foundBall();
   }

   public Point3D getBallLocation()
   {
      return initialSphereDetectionBehavior.getBallLocation();
   }

   public double getSphereRadius()
   {
      return initialSphereDetectionBehavior.getSpehereRadius();
   }

   @Override
   public void onBehaviorExited()
   {
   }

}