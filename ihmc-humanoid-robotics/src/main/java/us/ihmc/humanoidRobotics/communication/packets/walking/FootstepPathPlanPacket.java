package us.ihmc.humanoidRobotics.communication.packets.walking;

import java.util.ArrayList;

import us.ihmc.communication.packets.Packet;

public class FootstepPathPlanPacket extends Packet<FootstepPathPlanPacket>
{

   public boolean goalsValid;
   public FootstepDataMessage start;
   public ArrayList<FootstepDataMessage> originalGoals = new ArrayList<FootstepDataMessage>();
   public ArrayList<FootstepDataMessage> pathPlan = new ArrayList<FootstepDataMessage>();
   public ArrayList<Boolean> footstepUnknown = new ArrayList<Boolean>();
   public double subOptimality;
   public double pathCost = Double.POSITIVE_INFINITY;

   public FootstepPathPlanPacket()
   {
      // empty constructor for serialization
   }

   @Override
   public void set(FootstepPathPlanPacket other)
   {
      goalsValid = other.goalsValid;
      start = new FootstepDataMessage();
      start.set(other.start);
      originalGoals = new ArrayList<>();
      for (int i = 0; i < other.originalGoals.size(); i++)
      {
         FootstepDataMessage footstep = new FootstepDataMessage();
         footstep.set(other.originalGoals.get(i));
         originalGoals.add(footstep);
      }
      pathPlan = new ArrayList<>();
      for (int i = 0; i < other.pathPlan.size(); i++)
      {
         FootstepDataMessage footstep = new FootstepDataMessage();
         footstep.set(other.pathPlan.get(i));
         pathPlan.add(footstep);
      }

      footstepUnknown = new ArrayList<>();
      other.footstepUnknown.forEach(footstepUnknown::add);

      subOptimality = other.subOptimality;
      pathCost = other.pathCost;

      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(FootstepPathPlanPacket other, double epsilon)
   {
      if (goalsValid != other.goalsValid)
         return false;
      if (!start.epsilonEquals(other.start, epsilon))
         return false;
      if (originalGoals.size() != other.originalGoals.size())
         return false;
      if (pathPlan.size() != other.pathPlan.size())
         return false;
      if (footstepUnknown.size() != other.footstepUnknown.size())
         return false;
      if (Math.abs(subOptimality - other.subOptimality) > epsilon)
         return false;
      if (Math.abs(pathCost - other.pathCost) > epsilon)
         return false;
      for (int i = 0; i < originalGoals.size(); i++)
      {
         if (!originalGoals.get(i).epsilonEquals(other.originalGoals.get(i), epsilon))
            return false;
      }
      for (int i = 0; i < pathPlan.size(); i++)
      {
         if (!pathPlan.get(i).epsilonEquals(other.pathPlan.get(i), epsilon))
            return false;
      }
      for (int i = 0; i < footstepUnknown.size(); i++)
      {
         if (footstepUnknown.get(i) != other.footstepUnknown.get(i))
            return false;
      }

      return true;
   }

}
