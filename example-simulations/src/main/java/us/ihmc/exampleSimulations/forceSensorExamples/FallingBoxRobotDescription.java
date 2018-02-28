package us.ihmc.exampleSimulations.forceSensorExamples;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.Box3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.ForceSensorDescription;
import us.ihmc.robotics.robotDescription.GroundContactPointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;

public class FallingBoxRobotDescription extends RobotDescription
{
   private Box3D rootBox;
   private Box3D bodyBox;

   public FallingBoxRobotDescription(String name, Box3D rootBox, Box3D bodyBox, double mass, boolean useGroundContactPoints)
   {
      super(name);

      this.rootBox = rootBox;
      this.bodyBox = bodyBox;

      FloatingJointDescription rootJoint = new FloatingJointDescription("rootJoint", "bodyjointvariablename");

      LinkDescription rootLink = new LinkDescription("rootLink");
      rootLink.setMassAndRadiiOfGyration(1.0, rootBox.getLength(), rootBox.getWidth(), rootBox.getHeight());

      LinkGraphicsDescription rootLinkGraphics = new LinkGraphicsDescription();
      rootLinkGraphics.translate(0.0, 0.0, -0.5 * rootBox.getHeight());
      rootLinkGraphics.addCube(rootBox.getLength(), rootBox.getWidth(), rootBox.getHeight(), YoAppearance.Blue());
      rootLinkGraphics.identity();
      rootLink.setLinkGraphics(rootLinkGraphics);

      rootJoint.setLink(rootLink);
      
      addRootJoint(rootJoint);

      PinJointDescription bodyJoint = new PinJointDescription("bodyJoint", new Vector3D(0.0, 0.0, -0.5 * rootBox.getHeight()), Axis.Z);

      LinkDescription bodyLink = new LinkDescription("bodyLink");
      bodyLink.setCenterOfMassOffset(new Vector3D(0.0, 0.0, -0.5 * bodyBox.getHeight()));
      bodyLink.setMassAndRadiiOfGyration(mass, bodyBox.getLength(), bodyBox.getWidth(), bodyBox.getHeight());

      LinkGraphicsDescription bodyLinkGraphics = new LinkGraphicsDescription();
      bodyLinkGraphics.translate(0.0, 0.0, -bodyBox.getHeight());
      bodyLinkGraphics.addCube(bodyBox.getLength(), bodyBox.getWidth(), bodyBox.getHeight(), YoAppearance.Red());
      bodyLinkGraphics.identity();
      bodyLink.setLinkGraphics(bodyLinkGraphics);

      bodyJoint.setLink(bodyLink);

      if (useGroundContactPoints)
      {
         int idOfGCP = 0;
         GroundContactPointDescription gcpDescription1 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * bodyBox.getLength(), 0.5 * bodyBox.getWidth(), 0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription1);
         idOfGCP++;

         GroundContactPointDescription gcpDescription2 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * bodyBox.getLength(), 0.5 * bodyBox.getWidth(), -0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription2);
         idOfGCP++;

         GroundContactPointDescription gcpDescription3 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * bodyBox.getLength(), -0.5 * bodyBox.getWidth(), 0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription3);
         idOfGCP++;

         GroundContactPointDescription gcpDescription4 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(0.5 * bodyBox.getLength(), -0.5 * bodyBox.getWidth(), -0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription4);
         idOfGCP++;

         GroundContactPointDescription gcpDescription5 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * bodyBox.getLength(), 0.5 * bodyBox.getWidth(), 0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription5);
         idOfGCP++;

         GroundContactPointDescription gcpDescription6 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * bodyBox.getLength(), 0.5 * bodyBox.getWidth(), -0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription6);
         idOfGCP++;

         GroundContactPointDescription gcpDescription7 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * bodyBox.getLength(), -0.5 * bodyBox.getWidth(), 0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription7);
         idOfGCP++;

         GroundContactPointDescription gcpDescription8 = new GroundContactPointDescription(name + "_gc"
               + idOfGCP, new Vector3D(-0.5 * bodyBox.getLength(), -0.5 * bodyBox.getWidth(), -0.5 * bodyBox.getHeight() - 0.5 * bodyBox.getHeight()));
         bodyJoint.addGroundContactPoint(gcpDescription8);
         idOfGCP++;
      }

      RigidBodyTransform sensorLocation = new RigidBodyTransform();
      ForceSensorDescription ftSensor = new ForceSensorDescription(name + "_ft", sensorLocation);
      ftSensor.setUseGroundContactPoints(useGroundContactPoints);
      bodyJoint.addForceSensor(ftSensor);
      
      rootJoint.addJoint(bodyJoint);
   }

   public Box3D getBodyBox()
   {
      return bodyBox;
   }

   public Box3D getRootBox()
   {
      return rootBox;
   }

}
