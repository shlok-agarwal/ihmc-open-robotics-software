package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.BoxCollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionDataHolder;

public class FallingBoxCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   public FallingBoxCollisionMeshDefinitionDataHolder(Box3D box)
   {
      RigidBodyTransform transformToBody = new RigidBodyTransform();
      CollisionMeshDefinitionData bodyCollisionMeshData = new BoxCollisionMeshDefinitionData("body", box.getLength(), box.getWidth(), box.getHeight());
      bodyCollisionMeshData.setTransformToParentJoint(transformToBody);
      addCollisionMeshDefinitionData(bodyCollisionMeshData);
      
      setVisible(true);
   }
}
