package us.ihmc.humanoidRobotics.communication.packets;

import java.util.Random;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;

@RosMessagePacket(documentation = "This message is used to switch the control scheme between force and position control.\n"
      + "WARNING: When in position control, the IHMC balance algorithms will be disabled and\n" + "it is up to the user to ensure stability.",
      rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE,
      topic = "/control/high_level_state")
public class HighLevelStateMessage extends Packet<HighLevelStateMessage>
{
   @RosExportedField(documentation = "The enum value of the current high level state of the robot.")
   public HighLevelControllerState highLevelControllerState;

   public HighLevelStateMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   public HighLevelStateMessage(HighLevelControllerState highLevelControllerState)
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
      this.highLevelControllerState = highLevelControllerState;
   }

   public HighLevelControllerState getHighLevelControllerState()
   {
      return highLevelControllerState;
   }

   public boolean equals(Object obj)
   {
      return ((obj instanceof HighLevelStateMessage) && this.epsilonEquals((HighLevelStateMessage) obj, 0));
   }

   public String toString()
   {
      return "State= " + highLevelControllerState.toString();
   }

   @Override
   public boolean epsilonEquals(HighLevelStateMessage other, double epsilon)
   {
      return this.getHighLevelControllerState().equals(other.getHighLevelControllerState());
   }

   public HighLevelStateMessage(Random random)
   {
      double value = random.nextInt(3);
      HighLevelControllerState highLevelControllerState = HighLevelControllerState.WALKING;
      if (value == 1)
         highLevelControllerState = HighLevelControllerState.DO_NOTHING_BEHAVIOR;
      else if (value == 2)
         highLevelControllerState = HighLevelControllerState.DIAGNOSTICS;

      this.highLevelControllerState = highLevelControllerState;
   }
}
