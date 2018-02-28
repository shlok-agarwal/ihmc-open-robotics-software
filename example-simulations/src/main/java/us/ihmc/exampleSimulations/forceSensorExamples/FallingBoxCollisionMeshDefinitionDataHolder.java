package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.BoxCollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionData;
import us.ihmc.simulationconstructionset.collisionMeshDefinition.CollisionMeshDefinitionDataHolder;

public class FallingBoxCollisionMeshDefinitionDataHolder extends CollisionMeshDefinitionDataHolder
{
   public FallingBoxCollisionMeshDefinitionDataHolder(FallingBoxRobotDescription robotDescription)
   {
      Box3D bodyBox = robotDescription.getBodyBox();
      RigidBodyTransform transformToBody = new RigidBodyTransform();
      transformToBody.appendTranslation(0.0, 0.0, -0.5 * bodyBox.getHeight());
      CollisionMeshDefinitionData bodyCollisionMeshData = new BoxCollisionMeshDefinitionData("bodyJoint", bodyBox.getLength(), bodyBox.getWidth(),
                                                                                             bodyBox.getHeight());
      bodyCollisionMeshData.setTransformToParentJoint(transformToBody);
      addCollisionMeshDefinitionData(bodyCollisionMeshData);

      setVisible(true);
   }
}
