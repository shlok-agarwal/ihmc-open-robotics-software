package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.commons.MathTools;
import us.ihmc.communication.packets.KinematicsToolboxOutputStatus;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;

public class KinematicsToolboxOutputConverter
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FullHumanoidRobotModel fullRobotModelToUseForConversion;
   private final HumanoidReferenceFrames referenceFrames;
   private final FloatingInverseDynamicsJoint rootJoint;
   private final OneDoFJoint[] oneDoFJoints;
   private final int jointsHashCode;

   public KinematicsToolboxOutputConverter(FullHumanoidRobotModelFactory fullRobotModelFactory)
   {
      this.fullRobotModelToUseForConversion = fullRobotModelFactory.createFullRobotModel();
      rootJoint = fullRobotModelToUseForConversion.getRootJoint();
      oneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModelToUseForConversion);
      jointsHashCode = (int) NameBasedHashCodeTools.computeArrayHashCode(oneDoFJoints);
      referenceFrames = new HumanoidReferenceFrames(fullRobotModelToUseForConversion);
   }

   public void updateFullRobotModel(KinematicsToolboxOutputStatus solution)
   {
      if (jointsHashCode != solution.jointNameHash)
         throw new RuntimeException("Hashes are different.");

      for (int i = 0; i < oneDoFJoints.length; i++)
      {
         float q = solution.getJointAngles()[i];
         OneDoFJoint joint = oneDoFJoints[i];
         joint.setQ(q);
      }
      Vector3D32 translation = solution.getPelvisTranslation();
      rootJoint.setPosition(translation.getX(), translation.getY(), translation.getZ());
      Quaternion32 orientation = solution.getPelvisOrientation();
      rootJoint.setRotation(orientation.getX(), orientation.getY(), orientation.getZ(), orientation.getS());
      fullRobotModelToUseForConversion.updateFrames();
   }

   private WholeBodyTrajectoryMessage output;

   public void setMessageToCreate(WholeBodyTrajectoryMessage message)
   {
      output = message;
   }

   private double trajectoryTime = Double.NaN;

   public void setTrajectoryTime(double trajectoryTime)
   {
      this.trajectoryTime = trajectoryTime;
   }

   public void computeArmTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeArmTrajectoryMessage(robotSide);
   }

   public void computeArmTrajectoryMessage(RobotSide robotSide)
   {
      RigidBody hand = fullRobotModelToUseForConversion.getHand(robotSide);
      RigidBody chest = fullRobotModelToUseForConversion.getChest();
      OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
      int numberOfArmJoints = armJoints.length;
      double[] desiredJointPositions = new double[numberOfArmJoints];
      for (int i = 0; i < numberOfArmJoints; i++)
      {
         OneDoFJoint armJoint = armJoints[i];
         desiredJointPositions[i] = MathTools.clamp(armJoint.getQ(), armJoint.getJointLimitLower(), armJoint.getJointLimitUpper());
      }
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(robotSide, trajectoryTime, desiredJointPositions);
      output.setArmTrajectoryMessage(armTrajectoryMessage);
   }

   public void computeHandTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeHandTrajectoryMessage(robotSide);
   }

   public void computeHandTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      ReferenceFrame trajectoryFrame = worldFrame;
      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      ReferenceFrame handControlFrame = fullRobotModelToUseForConversion.getHandControlFrame(robotSide);
      FramePose3D desiredHandPose = new FramePose3D(handControlFrame);
      desiredHandPose.changeFrame(worldFrame);
      desiredHandPose.get(desiredPosition, desiredOrientation);
      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation, trajectoryFrame);
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrame(worldFrame);
      output.setHandTrajectoryMessage(handTrajectoryMessage);
   }

   public void computeChestTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      ReferenceFrame chestFrame = fullRobotModelToUseForConversion.getChest().getBodyFixedFrame();
      Quaternion desiredQuaternion = new Quaternion();
      FrameQuaternion desiredOrientation = new FrameQuaternion(chestFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredQuaternion.set(desiredOrientation);
      ReferenceFrame pelvisZUpFrame = referenceFrames.getPelvisZUpFrame();
      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime, desiredQuaternion, worldFrame, pelvisZUpFrame);
      output.setChestTrajectoryMessage(chestTrajectoryMessage);
   }

   public void computePelvisTrajectoryMessage()
   {
      checkIfDataHasBeenSet();

      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      ReferenceFrame pelvisFrame = fullRobotModelToUseForConversion.getRootJoint().getFrameAfterJoint();
      FramePose3D desiredPelvisPose = new FramePose3D(pelvisFrame);
      desiredPelvisPose.changeFrame(worldFrame);
      desiredPelvisPose.get(desiredPosition, desiredOrientation);
      PelvisTrajectoryMessage pelvisTrajectoryMessage = HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, desiredPosition, desiredOrientation);
      output.setPelvisTrajectoryMessage(pelvisTrajectoryMessage);
   }

   public void computeFootTrajectoryMessages()
   {
      for (RobotSide robotSide : RobotSide.values)
         computeFootTrajectoryMessage(robotSide);
   }

   public void computeFootTrajectoryMessage(RobotSide robotSide)
   {
      checkIfDataHasBeenSet();

      Point3D desiredPosition = new Point3D();
      Quaternion desiredOrientation = new Quaternion();
      ReferenceFrame footFrame = fullRobotModelToUseForConversion.getEndEffectorFrame(robotSide, LimbName.LEG);
      FramePose3D desiredFootPose = new FramePose3D(footFrame);
      desiredFootPose.changeFrame(worldFrame);
      desiredFootPose.get(desiredPosition, desiredOrientation);
      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(robotSide, trajectoryTime, desiredPosition, desiredOrientation);
      output.setFootTrajectoryMessage(footTrajectoryMessage);
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModelToUseForConversion;
   }

   private void checkIfDataHasBeenSet()
   {
      if (output == null)
         throw new RuntimeException("Need to call setMessageToCreate() first.");
      if (Double.isNaN(trajectoryTime))
         throw new RuntimeException("Need to call setTrajectoryTime() first.");
   }
}
