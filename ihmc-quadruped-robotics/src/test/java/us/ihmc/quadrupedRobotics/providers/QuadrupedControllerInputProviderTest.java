package us.ihmc.quadrupedRobotics.providers;

import static org.junit.Assert.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import org.junit.Test;

import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetworkedObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.net.TcpNetStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.quadrupedRobotics.communication.packets.BodyAngularRatePacket;
import us.ihmc.quadrupedRobotics.communication.packets.BodyOrientationPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComPositionPacket;
import us.ihmc.quadrupedRobotics.communication.packets.ComVelocityPacket;
import us.ihmc.quadrupedRobotics.communication.packets.PlanarVelocityPacket;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.commons.thread.ThreadTools;

public class QuadrupedControllerInputProviderTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testNoVariablesAreMixedUpWhenSendingTeleOpPacket() throws IOException
   {
      double epsilon = 0.01;
      Random random = new Random(151515);
      Point3D randomComPosition = RandomGeometry.nextPoint3D(random, -1000.0, 1000.0);
      Vector3D randomComVelocity = RandomGeometry.nextVector3D(random, 1000.0); 
      Quaternion randomBodyOrientation = RandomGeometry.nextQuaternion(random);
      Vector3D randomBodyAngularVelocity = RandomGeometry.nextVector3D(random, 1000.0); 
      Vector3D randomPlanarVelocity = RandomGeometry.nextVector3D(random, 1000.0); 

      ComPositionPacket comPositionPacket = new ComPositionPacket(randomComPosition);
      ComVelocityPacket comVelocityPacket = new ComVelocityPacket(randomComVelocity);
      BodyOrientationPacket bodyOrientationPacket = new BodyOrientationPacket(randomBodyOrientation);
      BodyAngularRatePacket bodyAngularRatePacket = new BodyAngularRatePacket(randomBodyAngularVelocity);
      PlanarVelocityPacket planarVelocityPacket = new PlanarVelocityPacket(randomPlanarVelocity);
      
      InputProviderTestNetClassList netClassList = new InputProviderTestNetClassList();
      
      NonThreadedTestingPacketCommunicator objectCommunicator = new NonThreadedTestingPacketCommunicator(netClassList);
      PacketCommunicator mockCommunicator = PacketCommunicator.createCustomPacketCommunicator(objectCommunicator, netClassList);
      
      YoVariableRegistry registry = new YoVariableRegistry("inputProvider");
      GlobalDataProducer dataProducer = new GlobalDataProducer(mockCommunicator);
      TestQuadrupedPhysicalProperties physicalProperties = new TestQuadrupedPhysicalProperties();
      QuadrupedPostureInputProvider postureInputProvider = new QuadrupedPostureInputProvider(physicalProperties, dataProducer, registry);
      QuadrupedPlanarVelocityInputProvider planarVelocityInputProvider = new QuadrupedPlanarVelocityInputProvider(dataProducer, registry);

      // load default parameters
      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);

      mockCommunicator.send(comPositionPacket);
      ThreadTools.sleep(3);
      Point3D comPositionInput = postureInputProvider.getComPositionInput();
      assertTrue(randomComPosition.epsilonEquals(comPositionInput, epsilon));
         
      mockCommunicator.send(comVelocityPacket);
      ThreadTools.sleep(3);
      Vector3D comVelocityInput = postureInputProvider.getComVelocityInput();
      assertTrue(randomComVelocity.epsilonEquals(comVelocityInput, epsilon));
      
      mockCommunicator.send(bodyOrientationPacket);
      ThreadTools.sleep(3);
      Quaternion bodyOrientationInput = postureInputProvider.getBodyOrientationInput();
      assertTrue(randomBodyOrientation.epsilonEquals(bodyOrientationInput, epsilon));
     
      mockCommunicator.send(bodyAngularRatePacket);
      ThreadTools.sleep(3);
      Vector3D bodyAngularRateInput = postureInputProvider.getBodyAngularRateInput();
      assertTrue(randomBodyAngularVelocity.epsilonEquals(bodyAngularRateInput, epsilon));
      
      mockCommunicator.send(planarVelocityPacket);
      ThreadTools.sleep(3);
      Vector3D planarVelocityInput = planarVelocityInputProvider.get();
      assertTrue(randomPlanarVelocity.epsilonEquals(planarVelocityInput, epsilon));
      
      //send everything again
      mockCommunicator.send(comPositionPacket);
      ThreadTools.sleep(3);
      comPositionInput = postureInputProvider.getComPositionInput();
      assertTrue(randomComPosition.epsilonEquals(comPositionInput, epsilon));
      
      mockCommunicator.send(comVelocityPacket);
      ThreadTools.sleep(3);
      comVelocityInput = postureInputProvider.getComVelocityInput();
      assertTrue(randomComVelocity.epsilonEquals(comVelocityInput, epsilon));
      
      mockCommunicator.send(bodyOrientationPacket);
      ThreadTools.sleep(3);
      bodyOrientationInput = postureInputProvider.getBodyOrientationInput();
      assertTrue(randomBodyOrientation.epsilonEquals(bodyOrientationInput, epsilon));
      
      mockCommunicator.send(bodyAngularRatePacket);
      ThreadTools.sleep(3);
      bodyAngularRateInput = postureInputProvider.getBodyAngularRateInput();
      assertTrue( randomBodyAngularVelocity.epsilonEquals(bodyAngularRateInput, epsilon));
      
      mockCommunicator.send(planarVelocityPacket);
      ThreadTools.sleep(3);
      planarVelocityInput = planarVelocityInputProvider.get();
      assertTrue(randomPlanarVelocity.epsilonEquals(planarVelocityInput, epsilon));
      
      //check that nothing was changed by another packet
      comPositionInput = postureInputProvider.getComPositionInput();
      assertTrue(randomComPosition.epsilonEquals(comPositionInput, epsilon));
      
      comVelocityInput = postureInputProvider.getComVelocityInput();
      assertTrue(randomComVelocity.epsilonEquals(comVelocityInput, epsilon));
      
      bodyOrientationInput = postureInputProvider.getBodyOrientationInput();
      assertTrue(randomBodyOrientation.epsilonEquals(bodyOrientationInput, epsilon));
      
      bodyAngularRateInput = postureInputProvider.getBodyAngularRateInput();
      assertTrue(randomBodyAngularVelocity.epsilonEquals(bodyAngularRateInput, epsilon));
      
      planarVelocityInput = planarVelocityInputProvider.get();
      assertTrue(randomPlanarVelocity.epsilonEquals(planarVelocityInput, epsilon));
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testYoVariableNamesAreCorrectAndUpdateInputs() throws IOException
   {
      double epsilon = 0.001;
      Random random = new Random(515151);
      Point3D randomComPosition = RandomGeometry.nextPoint3D(random, 1000.0, 1000.0);
      Vector3D randomComVelocity = RandomGeometry.nextVector3D(random, 1000.0); 
      Quaternion randomBodyOrientation = RandomGeometry.nextQuaternion(random);
      Vector3D randomBodyAngularVelocity = RandomGeometry.nextVector3D(random, 1000.0); 
      Vector3D randomPlanarVelocity = RandomGeometry.nextVector3D(random, 1000.0); 

      YoVariableRegistry registry = new YoVariableRegistry("inputProvider");
      TestQuadrupedPhysicalProperties physicalProperties = new TestQuadrupedPhysicalProperties();
      QuadrupedPostureInputProvider postureInputProvider = new QuadrupedPostureInputProvider(physicalProperties, null, registry);
      QuadrupedPlanarVelocityInputProvider planarVelocityInputProvider = new QuadrupedPlanarVelocityInputProvider(null, registry);

      YoDouble yoComPositionInputX = (YoDouble) registry.getVariable("comPositionInputX");
      YoDouble yoComPositionInputY = (YoDouble) registry.getVariable("comPositionInputY");
      YoDouble yoComPositionInputZ = (YoDouble) registry.getVariable("comPositionInputZ");
      YoDouble yoComVelocityInputX = (YoDouble) registry.getVariable("comVelocityInputX");
      YoDouble yoComVelocityInputY = (YoDouble) registry.getVariable("comVelocityInputY");
      YoDouble yoComVelocityInputZ = (YoDouble) registry.getVariable("comVelocityInputZ");
      YoDouble yoBodyOrientationInputYaw = (YoDouble) registry.getVariable("bodyOrientationInputYaw");
      YoDouble yoBodyOrientationInputPitch = (YoDouble) registry.getVariable("bodyOrientationInputPitch");
      YoDouble yoBodyOrientationInputRoll = (YoDouble) registry.getVariable("bodyOrientationInputRoll");
      YoDouble yoBodyAngularRateInputX = (YoDouble) registry.getVariable("bodyAngularRateInputX");
      YoDouble yoBodyAngularRateInputY = (YoDouble) registry.getVariable("bodyAngularRateInputY");
      YoDouble yoBodyAngularRateInputZ = (YoDouble) registry.getVariable("bodyAngularRateInputZ");
      YoDouble yoPlanarVelocityInputX = (YoDouble) registry.getVariable("planarVelocityInputX");
      YoDouble yoPlanarVelocityInputY = (YoDouble) registry.getVariable("planarVelocityInputY");
      YoDouble yoPlanarVelocityInputZ = (YoDouble) registry.getVariable("planarVelocityInputZ");
      
      
      yoComPositionInputX.set(randomComPosition.getX());
      yoComPositionInputY.set(randomComPosition.getY());
      yoComPositionInputZ.set(randomComPosition.getZ());
      Point3D comPositionInput = postureInputProvider.getComPositionInput();
      assertTrue(randomComPosition.epsilonEquals(comPositionInput, epsilon));
      
      yoComVelocityInputX.set(randomComVelocity.getX());
      yoComVelocityInputY.set(randomComVelocity.getY());
      yoComVelocityInputZ.set(randomComVelocity.getZ());
      Vector3D comVelocityInput = postureInputProvider.getComVelocityInput();
      assertTrue(randomComVelocity.epsilonEquals(comVelocityInput, epsilon));
      
      yoBodyOrientationInputYaw.set(randomBodyOrientation.getYaw());
      yoBodyOrientationInputPitch.set(randomBodyOrientation.getPitch());
      yoBodyOrientationInputRoll.set(randomBodyOrientation.getRoll());
      Quaternion bodyOrientationInput = postureInputProvider.getBodyOrientationInput();
      assertTrue(randomBodyOrientation.epsilonEquals(bodyOrientationInput, epsilon));
      
      yoBodyAngularRateInputX.set(randomBodyAngularVelocity.getX());
      yoBodyAngularRateInputY.set(randomBodyAngularVelocity.getY());
      yoBodyAngularRateInputZ.set(randomBodyAngularVelocity.getZ());
      Vector3D bodyAngularRateInput = postureInputProvider.getBodyAngularRateInput();
      assertTrue(randomBodyAngularVelocity.epsilonEquals(bodyAngularRateInput, epsilon));
      
      yoPlanarVelocityInputX.set(randomPlanarVelocity.getX());
      yoPlanarVelocityInputY.set(randomPlanarVelocity.getY());
      yoPlanarVelocityInputZ.set(randomPlanarVelocity.getZ());
      Vector3D planarVelocityInput = planarVelocityInputProvider.get();
      assertTrue(randomPlanarVelocity.epsilonEquals(planarVelocityInput, epsilon));
      
      //send everything again
      yoComPositionInputX.set(randomComPosition.getX());
      yoComPositionInputY.set(randomComPosition.getY());
      yoComPositionInputZ.set(randomComPosition.getZ());
      comPositionInput = postureInputProvider.getComPositionInput();
      assertTrue(randomComPosition.epsilonEquals(comPositionInput, epsilon));
      
      yoComVelocityInputX.set(randomComVelocity.getX());
      yoComVelocityInputY.set(randomComVelocity.getY());
      yoComVelocityInputZ.set(randomComVelocity.getZ());
      comVelocityInput = postureInputProvider.getComVelocityInput();
      assertTrue(randomComVelocity.epsilonEquals(comVelocityInput, epsilon));
      
      yoBodyOrientationInputYaw.set(randomBodyOrientation.getYaw());
      yoBodyOrientationInputPitch.set(randomBodyOrientation.getPitch());
      yoBodyOrientationInputRoll.set(randomBodyOrientation.getRoll());
      bodyOrientationInput = postureInputProvider.getBodyOrientationInput();
      assertTrue(randomBodyOrientation.epsilonEquals(bodyOrientationInput, epsilon));
      
      yoBodyAngularRateInputX.set(randomBodyAngularVelocity.getX());
      yoBodyAngularRateInputY.set(randomBodyAngularVelocity.getY());
      yoBodyAngularRateInputZ.set(randomBodyAngularVelocity.getZ());
      bodyAngularRateInput = postureInputProvider.getBodyAngularRateInput();
      assertTrue(randomBodyAngularVelocity.epsilonEquals(bodyAngularRateInput, epsilon));
      
      yoPlanarVelocityInputX.set(randomPlanarVelocity.getX());
      yoPlanarVelocityInputY.set(randomPlanarVelocity.getY());
      yoPlanarVelocityInputZ.set(randomPlanarVelocity.getZ());
      planarVelocityInput = planarVelocityInputProvider.get();
      assertTrue(randomPlanarVelocity.epsilonEquals(planarVelocityInput, epsilon));
      
      //check that nothing was changed by another packet
      comPositionInput = postureInputProvider.getComPositionInput();
      assertTrue(randomComPosition.epsilonEquals(comPositionInput, epsilon));
      
      comVelocityInput = postureInputProvider.getComVelocityInput();
      assertTrue(randomComVelocity.epsilonEquals(comVelocityInput, epsilon));
      
      bodyOrientationInput = postureInputProvider.getBodyOrientationInput();
      assertTrue(randomBodyOrientation.epsilonEquals(bodyOrientationInput, epsilon));
      
      bodyAngularRateInput = postureInputProvider.getBodyAngularRateInput();
      assertTrue(randomBodyAngularVelocity.epsilonEquals(bodyAngularRateInput, epsilon));
      
      planarVelocityInput = planarVelocityInputProvider.get();
      assertTrue(randomPlanarVelocity.epsilonEquals(planarVelocityInput, epsilon));
   }
   
   private final class NonThreadedTestingPacketCommunicator implements NetworkedObjectCommunicator
   {
      private final LinkedHashMap<Class<?>, ArrayList<ObjectConsumer<?>>> listeners = new LinkedHashMap<Class<?>, ArrayList<ObjectConsumer<?>>>();
      
      public NonThreadedTestingPacketCommunicator(NetClassList classList)
      {
         for (Class<?> clazz : classList.getPacketClassList())
         {
            listeners.put(clazz, new ArrayList<ObjectConsumer<?>>());
         }  
      }
      
      @Override
      public void consumeObject(Object object)
      {
         
      }

      @Override
      public boolean isConnected()
      {
         return true;
      }

      @Override
      public <T> void detachListener(Class<T> clazz, ObjectConsumer<T> listener)
      {
      }

      @Override
      public void detachGlobalListener(GlobalObjectConsumer listener)
      {
      }

      @Override
      public void connect() throws IOException
      {
      }

      @Override
      public void disconnect()
      {
      }

      @Override
      public void attachStateListener(ConnectionStateListener stateListener)
      {
      }

      @Override
      public <T> void attachListener(Class<T> clazz, ObjectConsumer<T> listener)
      {
         listeners.get(clazz).add(listener);
      }

      @Override
      public void attachGlobalListener(GlobalObjectConsumer listener)
      {
      }

      @Override
      public int send(Object object)
      {
         ArrayList<ObjectConsumer<?>> objectListeners = this.listeners.get(object.getClass());
         
         for (@SuppressWarnings("rawtypes")
         ObjectConsumer listener : objectListeners)
         {
            listener.consumeObject(object);
         }
         return 0;
      }

      @Override
      public void closeConnection()
      {
      }

      @Override
      public void attachStateListener(TcpNetStateListener stateListener)
      {
      }
   }

   private class InputProviderTestNetClassList extends NetClassList
   {
      public InputProviderTestNetClassList()
      {
         registerPacketClass(ComPositionPacket.class);
         registerPacketClass(ComVelocityPacket.class);
         registerPacketClass(BodyOrientationPacket.class);
         registerPacketClass(BodyAngularRatePacket.class);
         registerPacketClass(PlanarVelocityPacket.class);
      }
   }

   private class TestQuadrupedPhysicalProperties implements QuadrupedPhysicalProperties
   {
      @Override
      public Vector3D getOffsetFromJointBeforeFootToSole(RobotQuadrant robotQuadrant)
      {
         return null;
      }

      @Override
      public ArrayList<Point2D> getFootGroundContactPoints(RobotQuadrant robotQuadrant)
      {
         return null;
      }

      @Override
      public QuadrantDependentList<ArrayList<Point2D>> getFeetGroundContactPoints()
      {
         return null;
      }

      @Override
      public double getNominalCoMHeight()
      {
         return 0.0;
      }
   }
}
