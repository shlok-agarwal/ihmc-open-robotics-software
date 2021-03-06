package us.ihmc.humanoidRobotics.communication.packets.behaviors;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

public class WalkOverTerrainGoalPacket extends Packet<WalkOverTerrainGoalPacket>
{
   public Point3D position;
   public Quaternion orientation;

   public WalkOverTerrainGoalPacket()
   {
   }

   @Override
   public void set(WalkOverTerrainGoalPacket other)
   {
      position = new Point3D(other.position);
      orientation = new Quaternion(other.orientation);
      setPacketInformation(other);
   }

   @Override
   public boolean epsilonEquals(WalkOverTerrainGoalPacket other, double epsilon)
   {
      if(other == null)
         return false;

      return other.position.epsilonEquals(position, epsilon) && other.orientation.epsilonEquals(orientation, epsilon);
   }
}
