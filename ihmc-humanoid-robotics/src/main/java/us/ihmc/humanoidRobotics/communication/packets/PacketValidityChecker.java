package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.packets.ObjectValidityChecker;
import us.ihmc.communication.packets.ObjectValidityChecker.ObjectErrorType;
import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.utils.NameBasedHashCodeTools;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.OneDoFJointTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AdjustFootstepMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HumanoidBodyPart;
import us.ihmc.humanoidRobotics.communication.packets.walking.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.NeckTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.SpineTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.idl.TempPreallocatedList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

public abstract class PacketValidityChecker
{
   /**
    * Checks the validity of a {@link FootstepDataMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepDataMessage(FootstepDataMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getLocation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s location field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateQuat4d(message.getOrientation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s orientation field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getPredictedContactPoint2Ds() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < message.getPredictedContactPoint2Ds().size(); arrayListIndex++)
         {
            packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getPredictedContactPoint2Ds().get(arrayListIndex));

            if (packetFieldErrorType != null)
            {
               String messageClassName = message.getClass().getSimpleName();
               String errorMessage = messageClassName + "'s predictedContactPoints field " + packetFieldErrorType.getMessage();
               return errorMessage;
            }
         }
      }

      TrajectoryType trajectoryType = TrajectoryType.fromByte(message.getTrajectoryType());
      packetFieldErrorType = ObjectValidityChecker.validateEnum(trajectoryType);
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s trajectoryType field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      //TODO Check if thats supposed to be checked
      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.getSwingHeight());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s swingHeight field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (trajectoryType == TrajectoryType.WAYPOINTS)
      {
         String messageClassName = message.getClass().getSimpleName();
         TempPreallocatedList<SE3TrajectoryPointMessage> swingTrajectory = message.getSwingTrajectory();

         if (swingTrajectory == null)
         {
            String errorMessage = messageClassName + " has no swing trajectory but trajectory type was set to " + TrajectoryType.WAYPOINTS.toString() + ".";
            return errorMessage;
         }

         if (swingTrajectory.size() > Footstep.maxNumberOfSwingWaypoints)
         {
            String errorMessage = messageClassName + " has " + swingTrajectory.size() + " waypoints. Up to " + Footstep.maxNumberOfSwingWaypoints + " are allowed.";
            return errorMessage;
         }

         double lastTime = 0.0;
         for (int waypointIdx = 0; waypointIdx < swingTrajectory.size(); waypointIdx++)
         {
            double waypointTime = swingTrajectory.get(waypointIdx).getTime();
            if (waypointTime <= lastTime)
            {
               String errorMessage = messageClassName + "'s swing trajectory has non-increasing waypoint times.";
               return errorMessage;
            }
            lastTime = waypointTime;
         }

         if (message.getSwingDuration() > 0.0 && lastTime >= message.getSwingDuration())
         {
            String errorMessage = messageClassName + "'s swing trajectory has waypoints with time larger then the swing time.";
            return errorMessage;
         }

         if (message.getSwingTrajectoryBlendDuration() < 0.0)
         {
            String errorMessage = messageClassName + "'s swing trajectory blend duration is less than zero.";
            return errorMessage;
         }

         if (message.getSwingTrajectoryBlendDuration() > 0.0 && message.getSwingTrajectory().get(0).getTime() > 1.0e-5)
         {
            String errorMessage = messageClassName + "'s swing trajectory blend duration is greater than zero, initial waypoint at t = 0.0 is missing.";
            return errorMessage;
         }
      }

      if (trajectoryType == TrajectoryType.CUSTOM)
      {
         String messageClassName = message.getClass().getSimpleName();
         TempPreallocatedList<Point3D> positionWaypoints = message.getCustomPositionWaypoints();
         if (positionWaypoints == null)
         {
            String errorMessage = messageClassName + "'s type is custom but no position waypoints were specified.";
            return errorMessage;
         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepDataListMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepDataListMessage(FootstepDataListMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.defaultSwingDuration);
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s swingTime field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateDouble(message.defaultTransferDuration);
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s transferTime field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getFootstepDataList() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < message.getFootstepDataList().size(); arrayListIndex++)
         {
            FootstepDataMessage footstepData = message.getFootstepDataList().get(arrayListIndex);
            String footstepDataListErrorMessage = validateFootstepDataMessage(footstepData);

            if (footstepDataListErrorMessage != null)
            {
               String messageClassName = message.getClass().getSimpleName();
               String errorMessage = messageClassName + " field contains a FootstepData in which " + footstepDataListErrorMessage;
               return errorMessage;
            }
         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepDataMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateAdjustFootstepMessage(AdjustFootstepMessage message)
   {
      ObjectErrorType packetFieldErrorType;

      packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getLocation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s location field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateQuat4d(message.getOrientation());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s orientation field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getPredictedContactPoints() != null)
      {
         for (int arrayListIndex = 0; arrayListIndex < message.getPredictedContactPoints().size(); arrayListIndex++)
         {
            packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getPredictedContactPoints().get(arrayListIndex));

            if (packetFieldErrorType != null)
            {
               String messageClassName = message.getClass().getSimpleName();
               String errorMessage = messageClassName + "'s predictedContactPoints field " + packetFieldErrorType.getMessage();
               return errorMessage;
            }
         }
      }

      return null;
   }

   /**
    * Checks the validity of a {@link FootstepStatusMessage}.
    *
    * @param message
    * @return null if the packet is valid, or the error message.
    */
   public static String validateFootstepStatus(FootstepStatusMessage message)
   {
      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateTuple3d(message.getActualFootPositionInWorld());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s actualFootPositionInWorld field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateQuat4d(message.getActualFootOrientationInWorld());
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s actualFootOrientationInWorld field " + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      packetFieldErrorType = ObjectValidityChecker.validateEnum(FootstepStatus.fromByte(message.getFootstepStatus()));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         String errorMessage = messageClassName + "'s status field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      if (message.getFootstepIndex() < 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         return messageClassName + ": footstepIndex field should be non-negative";
      }

      return null;
   }

   public static String validateHandTrajectoryMessage(HandTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSE3TrajectoryMessage(message.se3Trajectory);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (errorType != null)
      {
         errorMessage = "robotSide field " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateArmTrajectoryMessage(ArmTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateJointspaceTrajectoryMessage(message.getJointspaceTrajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.robotSide));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateHeadTrajectoryMessage(HeadTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSO3TrajectoryMessage(message.so3Trajectory);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateChestTrajectoryMessage(ChestTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSO3TrajectoryMessage(message.so3Trajectory);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateNeckTrajectoryMessage(NeckTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateJointspaceTrajectoryMessage(message.getJointspaceTrajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateSpineTrajectoryMessage(SpineTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateJointspaceTrajectoryMessage(message.getJointspaceTrajectory());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateJointspaceTrajectoryMessage(JointspaceTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      if (message.jointTrajectoryMessages == null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s trajectory points are empty.";
         return errorMessage;
      }

      int numberOfJoints = message.jointTrajectoryMessages.size();
      if (numberOfJoints == 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + " is empty.";
         return errorMessage;
      }

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointTrajectoryMessage oneJointTrajectoryMessage = message.getJointTrajectoryMessages().get(jointIndex);
         if(oneJointTrajectoryMessage != null)
         {
            errorMessage = validateOneJointTrajectoryMessage(oneJointTrajectoryMessage, false);
         }
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = messageClassName + " Error with the " + jointIndex + " " + OneDoFJointTrajectoryMessage.class.getSimpleName() + " : " + errorMessage;
            return errorMessage;
         }
      }

      return null;
   }

   public static String validateSE3TrajectoryMessage(SE3TrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      if(message.getFrameInformation().getDataReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Expressed In Reference Frame Id Not Set";
      }

      if(message.getFrameInformation().getTrajectoryReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Trajectory Reference Frame Id Not Set";
      }

      SE3TrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.taskspaceTrajectoryPoints.size() == 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      for (int i = 0; i < message.taskspaceTrajectoryPoints.size(); i++)
      {
         SE3TrajectoryPointMessage waypoint = message.taskspaceTrajectoryPoints.get(i);
         errorMessage = validateSE3TrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }

      if (message.getUseCustomControlFrame() && message.controlFramePose == null)
      {
         String messageClassName = message.getClass().getSimpleName();
         return "The control frame pose for " + messageClassName + " has to be set to be able to use it.";
      }

      return null;
   }

   public static String validateSO3TrajectoryMessage(SO3TrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      SO3TrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.taskspaceTrajectoryPoints.size() == 0)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      if(message.getFrameInformation().getDataReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Expressed In Reference Frame Id Not Set";
      }

      if(message.getFrameInformation().getTrajectoryReferenceFrameId() == NameBasedHashCodeTools.NULL_HASHCODE)
      {
         return message.getClass().getSimpleName() + " Trajectory Reference Frame Id Not Set";
      }

      for (int i = 0; i < message.taskspaceTrajectoryPoints.size(); i++)
      {
         SO3TrajectoryPointMessage waypoint = message.taskspaceTrajectoryPoints.get(i);
         errorMessage = validateSO3TrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }

      if (message.getUseCustomControlFrame() && message.controlFramePose == null)
      {
         String messageClassName = message.getClass().getSimpleName();
         return "The control frame pose for " + messageClassName + " has to be set to be able to use it.";
      }

      return null;
   }

   public static String validatePelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSO3TrajectoryMessage(message.so3Trajectory);
      if (errorMessage != null)
         return PelvisOrientationTrajectoryMessage.class.getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validatePelvisTrajectoryMessage(PelvisTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSE3TrajectoryMessage(message.se3Trajectory);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateFootTrajectoryMessage(FootTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateSE3TrajectoryMessage(message.se3Trajectory);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
      if (errorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s robotSide field " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateFootLoadBearingMessage(FootLoadBearingMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(LoadBearingRequest.fromByte(message.getLoadBearingRequest()));
      if (errorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s request field " + errorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateGoHomeMessage(GoHomeMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateEnum(HumanoidBodyPart.fromByte(message.getHumanoidBodyPart()));
      if (errorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s endEffector field " + errorType.getMessage();
         return errorMessage;
      }

      if (HumanoidBodyPart.fromByte(message.getHumanoidBodyPart()).isRobotSideNeeded())
      {
         errorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.getRobotSide()));
         if (RobotSide.fromByte(message.getRobotSide()) == null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = messageClassName + "'s robotSide field is null. It is required for the bodyPart " + message.getHumanoidBodyPart();
            return errorMessage;
         }
      }

      return null;
   }

   public static String validatePelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      EuclideanTrajectoryPointMessage previousTrajectoryPoint = null;

      if (message.getEuclideanTrajectory().taskspaceTrajectoryPoints.isEmpty())
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = "Received " + messageClassName + " with no waypoint.";
         return errorMessage;
      }

      for (int i = 0; i < message.getEuclideanTrajectory().taskspaceTrajectoryPoints.size(); i++)
      {
         EuclideanTrajectoryPointMessage waypoint = message.getEuclideanTrajectory().taskspaceTrajectoryPoints.get(i);
         errorMessage = validateEuclideanTrajectoryPointMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
         {
            String messageClassName = message.getClass().getSimpleName();
            errorMessage = "The " + messageClassName + "'s " + i + "th waypoint " + errorMessage;
            return errorMessage;
         }
         previousTrajectoryPoint = waypoint;
      }

      return null;
   }

   public static String validateArmDesiredAccelerationsMessage(ArmDesiredAccelerationsMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateDesiredAccelerationsMessage(message.getDesiredAccelerations());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      ObjectErrorType packetFieldErrorType = ObjectValidityChecker.validateEnum(RobotSide.fromByte(message.robotSide));
      if (packetFieldErrorType != null)
      {
         String messageClassName = message.getClass().getSimpleName();
         errorMessage = messageClassName + "'s robotSide field" + packetFieldErrorType.getMessage();
         return errorMessage;
      }

      return null;
   }

   public static String validateNeckDesiredAccelerationsMessage(NeckDesiredAccelerationsMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateDesiredAccelerationsMessage(message.getDesiredAccelerations());
      if (errorMessage != null)
         return NeckDesiredAccelerationsMessage.class.getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateSpineDesiredAccelerationsMessage(SpineDesiredAccelerationsMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage == null)
         errorMessage = validateDesiredAccelerationsMessage(message.getDesiredAccelerations());
      if (errorMessage != null)
         return message.getClass().getSimpleName() + " " + errorMessage;

      return null;
   }

   public static String validateWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage message)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return errorMessage;

      if (!message.leftHandTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = validateHandTrajectoryMessage(message.leftHandTrajectoryMessage)) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.leftHandTrajectoryMessage.getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftHandTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.leftHandTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = validateHandTrajectoryMessage(message.rightHandTrajectoryMessage)) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.rightHandTrajectoryMessage.getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightHandTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.leftArmTrajectoryMessage.jointspaceTrajectory.jointTrajectoryMessages.isEmpty())
      {
         if ((errorMessage = validateArmTrajectoryMessage(message.leftArmTrajectoryMessage)) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.leftArmTrajectoryMessage.getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftArmTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.rightArmTrajectoryMessage.jointspaceTrajectory.jointTrajectoryMessages.isEmpty())
      {
         if ((errorMessage = validateArmTrajectoryMessage(message.rightArmTrajectoryMessage)) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.rightArmTrajectoryMessage.getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightArmTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.chestTrajectoryMessage.so3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = validateChestTrajectoryMessage(message.chestTrajectoryMessage)) != null)
            return errorMessage;
      }
      if (!message.pelvisTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = validatePelvisTrajectoryMessage(message.pelvisTrajectoryMessage)) != null)
            return errorMessage;
      }
      if (!message.headTrajectoryMessage.so3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = validateHeadTrajectoryMessage(message.headTrajectoryMessage)) != null)
            return errorMessage;
      }
      if (!message.leftFootTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = validateFootTrajectoryMessage(message.leftFootTrajectoryMessage)) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.leftFootTrajectoryMessage.getRobotSide()) != RobotSide.LEFT)
            return "The robotSide of leftFootTrajectoryMessage field is inconsistent with its name.";
      }
      if (!message.rightFootTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
      {
         if ((errorMessage = validateFootTrajectoryMessage(message.rightFootTrajectoryMessage)) != null)
            return errorMessage;
         else if (RobotSide.fromByte(message.rightFootTrajectoryMessage.getRobotSide()) != RobotSide.RIGHT)
            return "The robotSide of rightFootTrajectoryMessage field is inconsistent with its name.";
      }

      return null;
   }

   private static String validateSE3TrajectoryPointMessage(SE3TrajectoryPointMessage se3TrajectoryPoint, SE3TrajectoryPointMessage previousSE3TrajectoryPoint,
                                                           boolean checkId)
   {
      String errorMessage = validatePacket(se3TrajectoryPoint);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.position);
      if (errorType != null)
         return "SE3 waypoint position field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateQuat4d(se3TrajectoryPoint.orientation);
      if (errorType != null)
         return "SE3 waypoint orientation field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.linearVelocity);
      if (errorType != null)
         return "SE3 waypoint linear velocity field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.angularVelocity);
      if (errorType != null)
         return "SE3 waypoint angular velocity field " + errorType.getMessage();
      ;

      double subTrajectoryTime = se3TrajectoryPoint.getTime();
      if (previousSE3TrajectoryPoint != null)
         subTrajectoryTime -= previousSE3TrajectoryPoint.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "SE3 waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }

   private static String validateEuclideanTrajectoryPointMessage(EuclideanTrajectoryPointMessage se3TrajectoryPoint,
         EuclideanTrajectoryPointMessage previousTrajectoryPoint, boolean checkId)
   {
      String errorMessage = validatePacket(se3TrajectoryPoint);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.position);
      if (errorType != null)
         return "SE3 waypoint position field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(se3TrajectoryPoint.linearVelocity);
      if (errorType != null)
         return "SE3 waypoint linear velocity field " + errorType.getMessage();

      double subTrajectoryTime = se3TrajectoryPoint.getTime();
      if (previousTrajectoryPoint != null)
         subTrajectoryTime -= previousTrajectoryPoint.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "SE3 waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }

   private static String validateSO3TrajectoryPointMessage(SO3TrajectoryPointMessage so3TrajectoryPoint, SO3TrajectoryPointMessage previousSO3TrajectoryPoint,
                                                           boolean checkId)
   {
      String errorMessage = validatePacket(so3TrajectoryPoint);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateQuat4d(so3TrajectoryPoint.orientation);
      if (errorType != null)
         return "SO3 waypoint orientation field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateTuple3d(so3TrajectoryPoint.angularVelocity);
      if (errorType != null)
         return "SO3 waypoint angular velocity field " + errorType.getMessage();

      double subTrajectoryTime = so3TrajectoryPoint.getTime();
      if (previousSO3TrajectoryPoint != null)
         subTrajectoryTime -= previousSO3TrajectoryPoint.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "SO3 waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }

   private static String validateTrajectoryPoint1DMessage(TrajectoryPoint1DMessage waypoint1D, TrajectoryPoint1DMessage previousTrajectoryPoint1D,
                                                          boolean checkId)
   {
      String errorMessage = validatePacket(waypoint1D);
      if (errorMessage != null)
         return errorMessage;

      ObjectErrorType errorType;

      errorType = ObjectValidityChecker.validateDouble(waypoint1D.getPosition());
      if (errorType != null)
         return "1D waypoint orientation field " + errorType.getMessage();

      errorType = ObjectValidityChecker.validateDouble(waypoint1D.getVelocity());
      if (errorType != null)
         return "1D waypoint angular velocity field " + errorType.getMessage();

      double subTrajectoryTime = waypoint1D.getTime();
      if (previousTrajectoryPoint1D != null)
         subTrajectoryTime -= previousTrajectoryPoint1D.getTime();

      errorType = ObjectValidityChecker.validateTrajectoryTime(subTrajectoryTime);
      if (errorType != null)
         return "1D waypoint time (relative to previous waypoint) " + errorType.getMessage();

      return null;
   }

   private final static double MAX_ACCEPTED_JOINT_VELOCITY = 100.0;

   public static String validateOneJointTrajectoryMessage(OneDoFJointTrajectoryMessage message, boolean checkId)
   {
      String errorMessage = validatePacket(message);
      if (errorMessage != null)
         return errorMessage;

      TrajectoryPoint1DMessage previousTrajectoryPoint = null;

      if (message.trajectoryPoints.size() == 0)
         return "The joint trajectory message has no waypoint.";

      for (int i = 0; i < message.trajectoryPoints.size(); i++)
      {
         TrajectoryPoint1DMessage waypoint = message.trajectoryPoints.get(i);
         errorMessage = validateTrajectoryPoint1DMessage(waypoint, previousTrajectoryPoint, false);
         if (errorMessage != null)
            return "The " + i + "th " + errorMessage;
         previousTrajectoryPoint = waypoint;
      }

      for (int waypointIndex = 0; waypointIndex < message.trajectoryPoints.size(); waypointIndex++)
      {
         TrajectoryPoint1DMessage waypoint = message.trajectoryPoints.get(waypointIndex);
         double waypointPosition = waypoint.getPosition();

         if (Math.abs(waypointPosition) > Math.PI)
            return "The " + waypointIndex + "th waypoint position is unreasonable: " + waypointPosition;

         double waypointVelocity = waypoint.getVelocity();

         if (Math.abs(waypointVelocity) > MAX_ACCEPTED_JOINT_VELOCITY)
            return "The " + waypointIndex + "th waypoint velocity is unreasonable: " + waypointVelocity;
      }

      return null;
   }

   public static String validatePacket(Packet<?> packet)
   {
      if (packet == null)
         return "is null.";
      return null;
   }

   public static String validateDesiredAccelerationsMessage(DesiredAccelerationsMessage message)
   {
      if (message == null)
         return "is null.";
      if(message.getDesiredJointAccelerations() == null)
      {
         return "desired acceleration buffer null";
      }
      if(message.getDesiredJointAccelerations().size()  == 0)
      {
         return "desired acceleration buffer empty";
      }
      return null;
   }
}
