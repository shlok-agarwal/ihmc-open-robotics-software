package us.ihmc.exampleSimulations.genericQuadruped.model;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.partNames.*;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.*;

public class GenericQuadrupedJointNameMapAndContactDefinition implements QuadrupedJointNameMap, ContactPointDefinitionHolder
{
   private final String modelName = "genericQuadruped";
   private final String rootJoint = "body";

   private final LegJointName[] legJointNames = {LegJointName.HIP_ROLL, LegJointName.HIP_PITCH, LegJointName.KNEE_PITCH};
   private final String[] jointNamesBeforeFeet = new String[4];

   private final HashMap<String, QuadrupedJointName> quadrupedJointNameMap = new HashMap<>();
   private final HashMap<QuadrupedJointName, String> sdfJointNameMap = new HashMap<>();
   private final HashMap<String, JointRole> jointRoles = new HashMap<>();

   private final List<ImmutablePair<String, Vector3D>> jointNameGroundContactPointMap = new ArrayList<>();
   private final QuadrantDependentList<HashMap<LegJointName, String>> mapFromLegJointNameToJointId = new QuadrantDependentList<>();

   public GenericQuadrupedJointNameMapAndContactDefinition(GenericQuadrupedPhysicalProperties physicalProperties)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (LegJointName legJointName : legJointNames)
         {
            QuadrupedJointName quadrupedJointName = QuadrupedJointName.getName(robotQuadrant, legJointName);
            quadrupedJointNameMap.put(quadrupedJointName.getUnderBarName(), quadrupedJointName);
         }
      }

      for (Map.Entry<String, QuadrupedJointName> entry : quadrupedJointNameMap.entrySet())
      {
         sdfJointNameMap.put(entry.getValue(), entry.getKey());
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         mapFromLegJointNameToJointId.set(robotQuadrant, new HashMap<LegJointName, String>());
      }

      for (GenericQuadrupedOrderedJointMap joint : GenericQuadrupedOrderedJointMap.values())
      {
         String jointName = joint.getName();
         jointRoles.put(jointName, JointRole.LEG);

         RobotQuadrant robotQuadrant = joint.getRobotQuadrant();
         if (robotQuadrant != null)
         {
            HashMap<LegJointName, String> jointIdToLegNameMap = mapFromLegJointNameToJointId.get(robotQuadrant);
            jointIdToLegNameMap.put(joint.getLegJointName(), joint.getName());
         }
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
      {
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3D>(getJointBeforeFootName(robotQuadrant), physicalProperties.getOffsetFromJointBeforeFootToSole(robotQuadrant)));
         Vector3D shoulderContactOffset = new Vector3D(robotQuadrant.getEnd().negateIfHindEnd(0.08), robotQuadrant.getSide().negateIfRightSide(0.03), 0.02); // // FIXME: 6/2/16
         jointNameGroundContactPointMap.add(new ImmutablePair<String, Vector3D>(getLegJointName(robotQuadrant, LegJointName.HIP_ROLL), shoulderContactOffset));
      }

      jointNamesBeforeFeet[0] = getLegJointName(RobotQuadrant.FRONT_LEFT, LegJointName.KNEE_PITCH);
      jointNamesBeforeFeet[1] = getLegJointName(RobotQuadrant.FRONT_RIGHT, LegJointName.KNEE_PITCH);
      jointNamesBeforeFeet[2] = getLegJointName(RobotQuadrant.HIND_LEFT, LegJointName.KNEE_PITCH);
      jointNamesBeforeFeet[3] = getLegJointName(RobotQuadrant.HIND_RIGHT, LegJointName.KNEE_PITCH);
   }

   public Collection<QuadrupedJointName> getQuadrupedJointNames()
   {
      return quadrupedJointNameMap.values();
   }

   @Override
   public String getModelName()
   {
      return modelName;
   }

   @Override
   public JointRole getJointRole(String jointName)
   {
      return jointRoles.get(jointName);
   }

   @Override
   public NeckJointName getNeckJointName(String jointName)
   {
      return null;
   }

   @Override
   public SpineJointName getSpineJointName(String jointName)
   {
      return null;
   }

   @Override
   public String getPelvisName()
   {
      return rootJoint;
   }

   @Override
   public String getUnsanitizedRootJointInSdf()
   {
      return rootJoint;
   }

   @Override
   public String getChestName()
   {
      return null;
   }

   @Override
   public String getHeadName()
   {
      return null;
   }

   @Override
   public List<ImmutablePair<String, Vector3D>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public boolean isTorqueVelocityLimitsEnabled()
   {
      return false;
   }

   @Override
   public Set<String> getLastSimulatedJoints()
   {
      HashSet<String> lastSimulatedJoints = new HashSet<>();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         lastSimulatedJoints.add(getJointBeforeFootName(robotQuadrant));
      }
      return lastSimulatedJoints;
   }

   @Override
   public String[] getJointNamesBeforeFeet()
   {
      return jointNamesBeforeFeet;
   }

   @Override
   public LegJointName[] getLegJointNames()
   {
      return legJointNames;
   }

   @Override
   public ArmJointName[] getArmJointNames()
   {
      return null;
   }

   @Override
   public SpineJointName[] getSpineJointNames()
   {
      return null;
   }

   @Override
   public NeckJointName[] getNeckJointNames()
   {
      return new NeckJointName[0];
   }

   @Override
   public String getLegJointName(RobotQuadrant robotQuadrant, LegJointName legJointName)
   {
      HashMap<LegJointName, String> legJointMap = mapFromLegJointNameToJointId.get(robotQuadrant);
      return legJointMap.get(legJointName);
   }

   @Override
   public String getJointBeforeFootName(RobotQuadrant robotQuadrant)
   {
      HashMap<LegJointName, String> legJointMap = mapFromLegJointNameToJointId.get(robotQuadrant);
      return legJointMap.get(LegJointName.KNEE_PITCH);
   }

   @Override
   public QuadrupedJointName getJointNameForSDFName(String name)
   {
      return quadrupedJointNameMap.get(name);
   }

   @Override
   public String getSDFNameForJointName(QuadrupedJointName name)
   {
      return sdfJointNameMap.get(name);
   }

   @Override
   public RobotQuadrant getEndEffectorsRobotSegment(String joineNameBeforeEndEffector)
   {
      for(RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if(getJointBeforeFootName(robotQuadrant).equals(joineNameBeforeEndEffector))
         {
            return robotQuadrant;
         }
      }
      throw new IllegalArgumentException(joineNameBeforeEndEffector + " was not listed as an end effector in " + this.getClass().getSimpleName());
   }
}
