package us.ihmc.wholeBodyController.concurrent.controllerCoreCommands;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.*;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;

import java.util.Map;

public class VirtualModelControlCommandDataCopier
{
   private static final int INITIAL_CAPACITY = 20;

   private final VirtualModelControlCommandList virtualModelControlCommandList = new VirtualModelControlCommandList();

   private final RecyclingArrayList<ExternalWrenchCommand> externalWrenchCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, ExternalWrenchCommand.class);
   private final RecyclingArrayList<JointspaceAccelerationCommand> jointspaceAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, JointspaceAccelerationCommand.class);
   private final RecyclingArrayList<MomentumRateCommand> momentumRateCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, MomentumRateCommand.class);
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, PlaneContactStateCommand.class);
   private final RecyclingArrayList<CenterOfPressureCommand> centerOfPressureCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, CenterOfPressureCommand.class);
   private final RecyclingArrayList<SpatialAccelerationCommand> spatialAccelerationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, SpatialAccelerationCommand.class);
   private final RecyclingArrayList<JointAccelerationIntegrationCommand> jointAccelerationIntegrationCommands = new RecyclingArrayList<>(INITIAL_CAPACITY, JointAccelerationIntegrationCommand.class);

   public VirtualModelControlCommandDataCopier()
   {
      clear();
   }

   public void retrieveRigidBodiesFromName(Map<String, RigidBody> nameToRigidBodyMap)
   {
      for (int i = 0; i < externalWrenchCommands.size(); i++)
      {
         ExternalWrenchCommand command = externalWrenchCommands.get(i);
         command.setRigidBody(nameToRigidBodyMap.get(command.getRigidBodyName()));
      }

      for (int i = 0; i < planeContactStateCommands.size(); i++)
      {
         PlaneContactStateCommand command = planeContactStateCommands.get(i);
         command.setContactingRigidBody(nameToRigidBodyMap.get(command.getContactingRigidBodyName()));
      }

      for (int i = 0; i < centerOfPressureCommands.size(); i++)
      {
         CenterOfPressureCommand command = centerOfPressureCommands.get(i);
         command.setContactingRigidBody(nameToRigidBodyMap.get(command.getContactingRigidBodyName()));
      }

      for (int i = 0; i < spatialAccelerationCommands.size(); i++)
      {
         SpatialAccelerationCommand command = spatialAccelerationCommands.get(i);
         RigidBody base = nameToRigidBodyMap.get(command.getBaseName());
         RigidBody endEffector = nameToRigidBodyMap.get(command.getEndEffectorName());
         command.set(base, endEffector);
      }
   }

   public void retrieveJointsFromName(Map<String, OneDoFJoint> nameToJointMap)
   {
      for (int i = 0; i < jointspaceAccelerationCommands.size(); i++)
      {
         JointspaceAccelerationCommand command = jointspaceAccelerationCommands.get(i);
         command.retrieveJointsFromName(nameToJointMap);
      }
   }

   public void copyFromOther(VirtualModelControlCommandList other)
   {
      clear();

      for (int i = 0; i < other.getNumberOfCommands(); i++)
      {
         VirtualModelControlCommand<?> commandToCopy = other.getCommand(i);

         switch (commandToCopy.getCommandType())
         {
         case EXTERNAL_WRENCH:
            copyExternalWrenchCommand((ExternalWrenchCommand) commandToCopy);
            break;
         case MOMENTUM:
            copyMomentumRateCommand((MomentumRateCommand) commandToCopy);
            break;
         case PLANE_CONTACT_STATE:
            copyPlaneContactStateCommand((PlaneContactStateCommand) commandToCopy);
            break;
         case CENTER_OF_PRESSURE:
            copyCenterOfPressureCommand((CenterOfPressureCommand) commandToCopy);
            break;
         case COMMAND_LIST:
            copyFromOther((VirtualModelControlCommandList) commandToCopy);
            break;
         default:
            throw new RuntimeException("The command type: " + commandToCopy.getCommandType() + " is not handled.");
         }
      }
   }

   private void clear()
   {
      virtualModelControlCommandList.clear();
      externalWrenchCommands.clear();
      jointspaceAccelerationCommands.clear();
      momentumRateCommands.clear();
      planeContactStateCommands.clear();
      centerOfPressureCommands.clear();
      spatialAccelerationCommands.clear();
      jointAccelerationIntegrationCommands.clear();
   }

   private void copyExternalWrenchCommand(ExternalWrenchCommand commandToCopy)
   {
      ExternalWrenchCommand localCommand = externalWrenchCommands.add();
      localCommand.set(commandToCopy);
      virtualModelControlCommandList.addCommand(localCommand);
   }

   private void copyMomentumRateCommand(MomentumRateCommand commandToCopy)
   {
      MomentumRateCommand localCommand = momentumRateCommands.add();
      localCommand.set(commandToCopy);
      virtualModelControlCommandList.addCommand(localCommand);
   }

   private void copyPlaneContactStateCommand(PlaneContactStateCommand commandToCopy)
   {
      PlaneContactStateCommand localCommand = planeContactStateCommands.add();
      localCommand.set(commandToCopy);
      virtualModelControlCommandList.addCommand(localCommand);
   }

   private void copyCenterOfPressureCommand(CenterOfPressureCommand commandToCopy)
   {
      CenterOfPressureCommand localCommand = centerOfPressureCommands.add();
      localCommand.set(commandToCopy);
      virtualModelControlCommandList.addCommand(localCommand);
   }

   public VirtualModelControlCommandList getVirtualModelControlCommandList()
   {
      return virtualModelControlCommandList;
   }
}
