package us.ihmc.humanoidRobotics.communication.packets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import org.apache.commons.lang3.StringUtils;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.DisableOnDebug;
import org.junit.rules.Timeout;
import org.reflections.Reflections;

import gnu.trove.list.TByteList;
import gnu.trove.list.array.TByteArrayList;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TIntArrayList;
import gnu.trove.list.array.TLongArrayList;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.ParameterListPacket;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Point2D32;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.Vector2D32;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.communication.packets.walking.SnapFootstepPacket;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.humanoidRobotics.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.idl.TempPreallocatedList;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.AtlasAuxiliaryRobotData;

@ContinuousIntegrationPlan(categories = IntegrationCategory.HEALTH)
public class PacketCodeQualityTest
{
   @Rule
   public DisableOnDebug disableOnDebug = new DisableOnDebug(new Timeout(30, TimeUnit.SECONDS));

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testNoFieldsAreNullAfterPacketCreation()
   { // This test won't fail on Arrays or Lists
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);
      allPacketTypes.removeAll(reaInternalComms);
      allPacketTypes.remove(ParameterListPacket.class);
      allPacketTypes.remove(MessageOfMessages.class);
      allPacketTypes.remove(LocalVideoPacket.class);

      Set<Class<? extends Packet>> packetTypesWithNullFields = new TreeSet<>((o1, o2) -> o1.getSimpleName().compareTo(o2.getSimpleName()));

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Packet packetInstance = packetType.newInstance();
            if (!areAllObjectFieldsNonNull(packetInstance))
            {
               packetTypesWithNullFields.add(packetType);
            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithNullFields.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with null fields:");
            packetTypesWithNullFields.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Found illegal field types in Packet sub-types.", packetTypesWithNullFields.isEmpty());
   }

   private static boolean areAllObjectFieldsNonNull(Object object) throws IllegalArgumentException, IllegalAccessException
   {
      Field[] fields = object.getClass().getDeclaredFields();
      for (Field field : fields)
      {
         if (Modifier.isStatic(field.getModifiers()))
            continue;

         if (field.getType().isPrimitive())
            continue;
         if (field.get(object) == null)
            return false;
         if (thirdPartySerializableClasses.contains(field.getType()))
            continue;
         if (allowedEuclidTypes.contains(field.getType()))
            continue;
         if (!areAllObjectFieldsNonNull(field.get(object)))
            return false;
      }
      return true;
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketsHaveNoConvenienceMethod()
   { // This test won't fail for setUniqueId(long) or validateMessage()
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);
      allPacketTypes.removeAll(reaInternalComms);

      Map<Class<? extends Packet>, List<Method>> packetTypesWithConvenienceMethods = new TreeMap<>((o1, o2) -> o1.getSimpleName()
                                                                                                                 .compareTo(o2.getSimpleName()));

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Method[] methods = packetType.getDeclaredMethods();

            Map<String, Class<?>> setterNames = new HashMap<>();
            Map<String, Class<?>> getterNames = new HashMap<>();

            for (Field field : packetType.getDeclaredFields())
            {
               String methodSuffix = StringUtils.capitalize(field.getName());
               setterNames.put("set" + methodSuffix, field.getType());
               getterNames.put("get" + methodSuffix, field.getType());

               if (field.getType() == StringBuilder.class)
               {
                  setterNames.put("set" + methodSuffix, String.class);
                  getterNames.put("get" + methodSuffix + "AsString", String.class);
               }
            }

            for (Method method : methods)
            {
               String methodName = method.getName();
               if (methodName.equals("toString") && method.getParameterCount() == 0 && method.getReturnType() == String.class)
                  continue;
               if (methodName.equals("validateMessage") && method.getParameterCount() == 0 && method.getReturnType() == String.class)
                  continue;
               if (methodName.equals("setUniqueId") && method.getParameterCount() == 1 && method.getReturnType() == void.class
                     && method.getParameterTypes()[0] == long.class)
                  continue;
               if (methodName.equals("epsilonEquals") && method.getParameterCount() == 2 && method.getReturnType() == boolean.class)
               {
                  if (method.getParameterTypes()[1] == double.class)
                  {
                     Class<?> firstParameterType = method.getParameterTypes()[0];
                     if (firstParameterType == packetType || firstParameterType == Object.class)
                        continue;
                  }
               }

               if (methodName.equals("equals") && method.getParameterCount() == 1 && method.getReturnType() == boolean.class)
               {
                  if (method.getParameterTypes()[0] == Object.class)
                     continue;
               }

               if (methodName.equals("set"))
               {
                  if (method.getParameterCount() == 1 && method.getReturnType() == void.class)
                  {
                     Class<?> firstParameterType = method.getParameterTypes()[0];
                     // This is due to implementing Settable<T>, the generic parameter gets "lost" at compilation and is replaced with Object :/
                     if (firstParameterType == packetType || firstParameterType == Object.class)
                        continue; // The method is an acceptable setter.
                  }
               }

               if (setterNames.containsKey(methodName))
               { // Allowing setters with argument being a super-type of the field.
                  if (method.getParameterCount() == 1 && method.getParameterTypes()[0].isAssignableFrom(setterNames.get(methodName)))
                  {
                     if (method.getReturnType() == void.class)
                        continue; // The method is an acceptable setter.
                  }
               }

               if (getterNames.containsKey(methodName))
               {
                  if (method.getParameterCount() == 0 && method.getReturnType().isAssignableFrom(getterNames.get(methodName)))
                     continue; // The method is an acceptable getter.
               }

               if (!packetTypesWithConvenienceMethods.containsKey(packetType))
                  packetTypesWithConvenienceMethods.put(packetType, new ArrayList<>());
               packetTypesWithConvenienceMethods.get(packetType).add(method);
            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithConvenienceMethods.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with illegal field type:");
            packetTypesWithConvenienceMethods.entrySet().forEach(type -> PrintTools.error(type.getKey().getSimpleName() + ": "
                  + type.getValue().stream().map(Method::getName).collect(Collectors.toList())));
         }
      }

      assertTrue("Found illegal field types in Packet sub-types.", packetTypesWithConvenienceMethods.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketsUseTempPreallocatedListOnly()
   { // This test won't fail on Arrays or Lists
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);
      allPacketTypes.removeAll(reaInternalComms);
      allPacketTypes.remove(ParameterListPacket.class);
      allPacketTypes.remove(MessageOfMessages.class);

      Map<Class<? extends Packet>, List<Class>> packetTypesWithIterableOrArrayField = new HashMap<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Field[] fields = packetType.getDeclaredFields();
            for (Field field : fields)
            {
               if (Modifier.isStatic(field.getModifiers()))
                  continue;

               Class<?> typeToCheck = field.getType();

               if (typeToCheck.isArray() || Iterable.class.isAssignableFrom(typeToCheck))
               {
                  if (!packetTypesWithIterableOrArrayField.containsKey(packetType))
                     packetTypesWithIterableOrArrayField.put(packetType, new ArrayList<>());
                  packetTypesWithIterableOrArrayField.get(packetType).add(typeToCheck);
               }

            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithIterableOrArrayField.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with illegal field type:");
            packetTypesWithIterableOrArrayField.entrySet().forEach(type -> PrintTools.error(type.getKey().getSimpleName() + ": "
                  + type.getValue().stream().map(Class::getSimpleName).collect(Collectors.toList())));
         }
      }

      assertTrue("Found illegal field types in Packet sub-types.", packetTypesWithIterableOrArrayField.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketOnlyExtendPacketClass()
   {
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithAdditionalSuperTypes = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            if (packetType.getSuperclass() != Packet.class)
               packetTypesWithAdditionalSuperTypes.add(packetType);

            if (packetType.getInterfaces().length != 0)
               packetTypesWithAdditionalSuperTypes.add(packetType);
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithAdditionalSuperTypes.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with illegal hierarchy:");
            packetTypesWithAdditionalSuperTypes.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Packet sub-types should only extend Packet class.", packetTypesWithAdditionalSuperTypes.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketDoNotDeclareTypes()
   {
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithNestedType = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Class<?>[] declaredClasses = packetType.getDeclaredClasses();

            if (declaredClasses.length > 0)
               packetTypesWithNestedType.add(packetType);
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithNestedType.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with nested type:");
            packetTypesWithNestedType.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Packet sub-types should not have nested type.", packetTypesWithNestedType.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketsHaveUniqueSimpleNameBasedHashCode()
   { // This test won't fail on Arrays or Lists
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);
      TIntObjectHashMap<Class> allPacketSimpleNameBasedHashCode = new TIntObjectHashMap<>();
      int numberOfCollisions = 0;

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         int simpleNameBasedHashCode = packetType.getSimpleName().hashCode();
         if (allPacketSimpleNameBasedHashCode.containsKey(simpleNameBasedHashCode))
         {
            numberOfCollisions++;
            if (verbose)
            {
               PrintTools.error("Hash-code collision between: " + packetType.getSimpleName() + " and "
                     + allPacketSimpleNameBasedHashCode.get(simpleNameBasedHashCode).getSimpleName());
            }
         }
      }

      assertEquals("Found hash code collisions.", 0, numberOfCollisions);
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketsDeclarePrimitiveOrMessageTypeFields()
   { // This test won't fail on Arrays or Lists
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);
      allPacketTypes.removeAll(reaInternalComms);
      allPacketTypes.remove(LocalVideoPacket.class); // That guy is a packet but does not make it to the network. It will stay on Kryo.
      allPacketTypes.remove(MessageOfMessages.class); // This guy has its own ticket to get converted. 
      allPacketTypes.remove(ParameterListPacket.class); // This guys won't get migrated to DDS/ROS

      Map<Class<? extends Packet>, List<Class>> packetTypesWithIllegalFieldTypes = new HashMap<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Field[] fields = packetType.getDeclaredFields();
            for (Field field : fields)
            {
               if (Modifier.isStatic(field.getModifiers()))
                  continue;

               Class<?> typeToCheck = field.getType();

               if (TempPreallocatedList.class.isAssignableFrom(typeToCheck))
               {
                  Packet packetInstance = packetType.newInstance();
                  Object fieldInstance = field.get(packetInstance);
                  Field clazzField = typeToCheck.getDeclaredField("clazz");
                  clazzField.setAccessible(true);
                  typeToCheck = (Class<?>) clazzField.get(fieldInstance);
               }
               while (typeToCheck.isArray())
                  typeToCheck = typeToCheck.getComponentType();
               if (Packet.class.isAssignableFrom(typeToCheck))
                  continue;
               if (isPrimitive(typeToCheck))
                  continue;
               if (thirdPartySerializableClasses.contains(typeToCheck))
                  continue;
               if (allowedEuclidTypes.contains(typeToCheck))
                  continue;
               if (!packetTypesWithIllegalFieldTypes.containsKey(packetType))
                  packetTypesWithIllegalFieldTypes.put(packetType, new ArrayList<>());
               packetTypesWithIllegalFieldTypes.get(packetType).add(typeToCheck);
            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithIllegalFieldTypes.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with illegal field type:");
            packetTypesWithIllegalFieldTypes.entrySet().forEach(type -> PrintTools.error(type.getKey().getSimpleName() + ": "
                  + type.getValue().stream().map(Class::getSimpleName).collect(Collectors.toList())));
         }
      }

      assertTrue("Found illegal field types in Packet sub-types.", packetTypesWithIllegalFieldTypes.isEmpty());
   }

   private static boolean isPrimitive(Class<?> clazz)
   {
      if (clazz.isPrimitive())
         return true;
      if (clazz.isArray() && clazz.getComponentType().isPrimitive())
         return true;
      return false;
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketStaticFieldsAreFinal()
   {
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithNonFinalStaticFields = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Field[] fields = packetType.getFields();
            for (Field field : fields)
            {
               if (Modifier.isStatic(field.getModifiers()) && !Modifier.isFinal(field.getModifiers()))
                  packetTypesWithNonFinalStaticFields.add(packetType);
            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithNonFinalStaticFields.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with non-final static fields:");
            packetTypesWithNonFinalStaticFields.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Packet sub-types should only declare static fields as final.", packetTypesWithNonFinalStaticFields.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketHaveNoEnum()
   {
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithEnumFields = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Field[] fields = packetType.getFields();
            for (Field field : fields)
            {
               Class<?> typeToCheck = field.getType();
               while (typeToCheck.isArray())
                  typeToCheck = typeToCheck.getComponentType();
               boolean isEnum = Enum.class.isAssignableFrom(typeToCheck);
               if (isEnum)
                  packetTypesWithEnumFields.add(packetType);
            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithEnumFields.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet with enum fields:");
            packetTypesWithEnumFields.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Packet sub-types should note have any enum field.", packetTypesWithEnumFields.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketByteFieldNameRefersToEnumType() throws NoSuchFieldException, SecurityException
   {
      boolean verbose = false;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);
      Set<String> enumLowerCaseNames = reflections.getSubTypesOf(Enum.class).stream().map(Class::getSimpleName).map(name -> name.toLowerCase())
                                                  .collect(Collectors.toSet());

      Set<Class<? extends Packet>> packetTypesWithByteFieldNameNotMatchingEnum = new HashSet<>();

      Set<Field> fieldsToIngore = new HashSet<>();
      fieldsToIngore.add(VideoPacket.class.getField("data"));
      fieldsToIngore.add(SnapFootstepPacket.class.getField("flag"));
      fieldsToIngore.add(AtlasAuxiliaryRobotData.class.getField("electricJointEnabledArray"));

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Field[] fields = packetType.getDeclaredFields();
            for (Field field : fields)
            {
               if (fieldsToIngore.contains(field))
                  continue;
               if (Modifier.isStatic(field.getModifiers()))
                  continue; // Ignore the static fields.

               Class<?> typeToCheck = field.getType();
               while (typeToCheck.isArray())
                  typeToCheck = typeToCheck.getComponentType();

               boolean isEnum = byte.class == typeToCheck || TByteList.class.isAssignableFrom(typeToCheck);

               if (isEnum)
               {
                  String expectedToContainEnumName = field.getName().toLowerCase();
                  boolean foundMatchingEnum = enumLowerCaseNames.stream().filter(enumName -> expectedToContainEnumName.contains(enumName)).findFirst()
                                                                .isPresent();
                  if (!foundMatchingEnum)
                  {
                     packetTypesWithByteFieldNameNotMatchingEnum.add(packetType);
                     break;
                  }
               }
            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithByteFieldNameNotMatchingEnum.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet byte fields not matching enums:");
            packetTypesWithByteFieldNameNotMatchingEnum.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Packet sub-types byte fields' name should contain the name of the enum they referring to.",
                 packetTypesWithByteFieldNameNotMatchingEnum.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testPacketWithByteFieldDeclareEnumValuesAsStaticByteFields() throws NoSuchFieldException, SecurityException
   {
      boolean verbose = true;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);
      Map<String, Class<? extends Enum>> nameToEnumMap = new HashMap<>();
      reflections.getSubTypesOf(Enum.class).forEach(type -> nameToEnumMap.put(type.getSimpleName().toLowerCase(), type));

      Set<Class<? extends Packet>> packetTypesWithByteFieldNameNotMatchingEnum = new HashSet<>();

      Set<Field> fieldsToIngore = new HashSet<>();
      fieldsToIngore.add(VideoPacket.class.getField("data"));
      fieldsToIngore.add(SnapFootstepPacket.class.getField("flag"));
      fieldsToIngore.add(AtlasAuxiliaryRobotData.class.getField("electricJointEnabledArray"));

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            Field[] fields = packetType.getDeclaredFields();

            Field[] staticFields = Arrays.stream(fields).filter(f -> Modifier.isStatic(f.getModifiers())).toArray(Field[]::new);

            for (Field field : fields)
            {
               if (fieldsToIngore.contains(field))
                  continue;
               if (Modifier.isStatic(field.getModifiers()))
                  continue; // Ignore the static fields.
               Class<?> typeToCheck = field.getType();
               while (typeToCheck.isArray())
                  typeToCheck = typeToCheck.getComponentType();
               if (byte.class != typeToCheck && !TByteList.class.isAssignableFrom(typeToCheck))
                  continue;
               String expectedToContainEnumName = field.getName().toLowerCase();
               Set<Entry<String, Class<? extends Enum>>> potentialMatchingEnums = nameToEnumMap.entrySet().stream()
                                                                                               .filter(entry -> expectedToContainEnumName.contains(entry.getKey()))
                                                                                               .collect(Collectors.toSet());

               if (potentialMatchingEnums.isEmpty())
                  new AssertionError("Could not find enum type for: " + packetType.getSimpleName() + "." + field.getName());

               Class<? extends Enum> matchingEnum = null;
               int score = Integer.MAX_VALUE;

               for (Entry<String, Class<? extends Enum>> entry : potentialMatchingEnums)
               { // Simple string matching scoring system to privilege the enum with the longest string match (here smallest residual once removed from the field name).
                  int indexFromStart = expectedToContainEnumName.indexOf(entry.getKey()); // represents prefix length that is different from the enum name
                  int indexFromEnd = StringUtils.reverse(expectedToContainEnumName).indexOf(StringUtils.reverse(entry.getKey())); // represents suffix length that is different from the enum name

                  int newScore = indexFromStart + indexFromEnd; // summing the two indices gives us the matching score, the lower the better.
                  if (newScore < score)
                  {
                     matchingEnum = entry.getValue();
                     score = newScore;
                  }
               }

               Enum[] enumConstants = matchingEnum.getEnumConstants();
               // Now verifies that all the enum constants are declared as statics in the packet type and that they refer to the enum ordinal.

               for (Enum enumConstant : enumConstants)
               {

                  Field staticEnumConstantValue = Arrays.stream(staticFields).filter(f -> f.getName().endsWith(enumConstant.name())).findFirst().orElse(null);
                  if (staticEnumConstantValue == null)
                  {
                     String errorMessage = packetType.getSimpleName() + "." + field.getName() + " refers to the enum: " + matchingEnum.getSimpleName() + " but "
                           + packetType.getSimpleName() + " does not declare the enum constants as static fields.";
                     PrintTools.error(errorMessage);
                     ThreadTools.sleep(10);
                     System.out.println("Example 1:");
                     for (Enum enumConstant2 : enumConstants)
                        System.out.println("public static final byte " + toUpperCaseWithUnderscore(matchingEnum.getSimpleName()) + "_" + enumConstant2.name()
                              + " = " + ((byte) enumConstant2.ordinal()) + ";");
                     System.out.println("");
                     System.out.println("Example 2:");
                     for (Enum enumConstant2 : enumConstants)
                        System.out.println("public static final byte " + enumConstant2.name() + " = " + ((byte) enumConstant2.ordinal()) + ";");

                     fail(errorMessage);
                  }
                  else
                  {
                     if (staticEnumConstantValue.getType() != byte.class)
                        fail("The static field: " + packetType.getSimpleName() + "." + staticEnumConstantValue.getName() + " should be a byte.");

                     byte value = (byte) staticEnumConstantValue.get(null);
                     assertEquals("The static field: " + packetType.getSimpleName() + "." + staticEnumConstantValue.getName() + " should be equal to: "
                           + (byte) enumConstant.ordinal(), (byte) enumConstant.ordinal(), value);
                  }
               }
            }
         }
         catch (Exception e)
         {
            PrintTools.error("Problem with packet: " + packetType.getSimpleName());
            e.printStackTrace();
         }
      }

      if (verbose)
      {
         if (!packetTypesWithByteFieldNameNotMatchingEnum.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet byte fields not matching enums:");
            packetTypesWithByteFieldNameNotMatchingEnum.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Packet sub-types byte fields' name should contain the name of the enum they referring to.",
                 packetTypesWithByteFieldNameNotMatchingEnum.isEmpty());
   }

   private static String toUpperCaseWithUnderscore(String camelCase)
   {
      // For normal camelCase:
      String regex = "([a-z0-9])([A-Z]+)";
      String replacement = "$1_$2";

      String inWithUnderscores = camelCase.replaceAll(regex, replacement);

      regex = "([A-Z])([A-Z])([a-z]+)";
      replacement = "$1_$2$3";

      inWithUnderscores = inWithUnderscores.replaceAll(regex, replacement);

      String ret = inWithUnderscores.toUpperCase();
      return ret;
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 4.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testOnlyEmptyAndCopyConstructor()
   {
      boolean verbose = false;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithoutEmptyConstructor = new HashSet<>();
      Set<Class<? extends Packet>> packetTypesWithNonEmptyConstructors = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            List<Constructor<?>> constructors = Arrays.asList(packetType.getDeclaredConstructors());
            assertFalse("The type: " + packetType.getSimpleName() + " has no constructors?!", constructors.isEmpty());

            boolean hasEmptyConstructor = constructors.stream().filter(constructor -> constructor.getParameterTypes().length == 0).findFirst().isPresent();
            if (!hasEmptyConstructor)
               packetTypesWithoutEmptyConstructor.add(packetType);

            boolean hasNonEmptyConstructors = constructors.stream().filter(constructor -> constructor.getParameterTypes().length != 0)
                                                          .filter(constructor -> constructor.getParameterTypes().length != 1
                                                                || !constructor.getParameterTypes()[0].equals(packetType))
                                                          .findFirst().isPresent();
            if (hasNonEmptyConstructors)
               packetTypesWithNonEmptyConstructors.add(packetType);
         }
         catch (SecurityException e)
         {
         }
      }

      if (verbose)
      {
         if (!packetTypesWithoutEmptyConstructor.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet sub-types without an empty constructor:");
            packetTypesWithoutEmptyConstructor.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
         if (!packetTypesWithNonEmptyConstructors.isEmpty())
         {
            System.out.println();
            System.out.println();
            PrintTools.error("List of packet sub-types with non-empty constructors:");
            packetTypesWithNonEmptyConstructors.forEach(type -> PrintTools.error(type.getSimpleName()));
         }
      }

      assertTrue("Packet sub-types should implement an empty constructor.", packetTypesWithoutEmptyConstructor.isEmpty());
      assertTrue("Packet sub-types should not implement a non-empty constructor.", packetTypesWithNonEmptyConstructors.isEmpty());
   }

   @SuppressWarnings("rawtypes")
   @ContinuousIntegrationTest(estimatedDuration = 1.0, categoriesOverride = IntegrationCategory.FAST)
   @Test(timeout = Integer.MAX_VALUE)
   public void testNoRandomConstructor()
   {
      boolean printPacketTypesWithRandomConstructor = false;

      Reflections reflections = new Reflections("us.ihmc");
      Set<Class<? extends Packet>> allPacketTypes = reflections.getSubTypesOf(Packet.class);

      Set<Class<? extends Packet>> packetTypesWithRandomConstructor = new HashSet<>();

      for (Class<? extends Packet> packetType : allPacketTypes)
      {
         try
         {
            packetType.getConstructor(Random.class);
            // If we get here, that means the type implement a random constructor.
            if (printPacketTypesWithRandomConstructor)
               PrintTools.error("Found type that implements a random constructor: " + packetType.getSimpleName());
            packetTypesWithRandomConstructor.add(packetType);
         }
         catch (NoSuchMethodException | SecurityException e)
         {
         }
      }

      assertTrue("Packet sub-types should not implement a random constructor.", packetTypesWithRandomConstructor.isEmpty());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = Integer.MAX_VALUE)
   public void testAllPacketFieldsArePublic()
   {
      IHMCCommunicationKryoNetClassList classList = new IHMCCommunicationKryoNetClassList();

      for (Class<?> clazz : classList.getPacketClassList())
      {
         checkIfAllFieldsArePublic(clazz);
      }

      for (Class<?> clazz : classList.getPacketFieldList())
      {
         checkIfAllFieldsArePublic(clazz);
      }
   }

   private void checkIfAllFieldsArePublic(Class<?> clazz)
   {
      if (List.class.isAssignableFrom(clazz))
         return;

      if (thirdPartySerializableClasses.contains(clazz) || allowedEuclidTypes.contains(clazz))
         return;

      for (Field field : clazz.getDeclaredFields())
      {
         if (Modifier.isStatic(field.getModifiers()))
            continue;
         if (Modifier.isTransient(field.getModifiers()))
            continue;
         if (field.isSynthetic())
            continue;

         assertTrue("Class " + clazz.getCanonicalName() + " has non-public field " + field.getName() + " declared by "
               + field.getDeclaringClass().getCanonicalName(), Modifier.isPublic(field.getModifiers()));
         assertFalse("Class " + clazz.getCanonicalName() + " has final field " + field.getName() + " declared by "
               + field.getDeclaringClass().getCanonicalName(), Modifier.isFinal(field.getModifiers()));
      }
   }

   @SuppressWarnings("rawtypes")
   private static final Set<Class<? extends Packet>> reaInternalComms;
   static
   {
      Reflections reflections = new Reflections("us.ihmc.robotEnvironmentAwareness");
      reaInternalComms = reflections.getSubTypesOf(Packet.class);
   }

   private static final Set<Class<?>> thirdPartySerializableClasses = new HashSet<>();
   static
   {
      thirdPartySerializableClasses.add(TempPreallocatedList.class);
      thirdPartySerializableClasses.add(TByteArrayList.class);
      thirdPartySerializableClasses.add(TFloatArrayList.class);
      thirdPartySerializableClasses.add(TDoubleArrayList.class);
      thirdPartySerializableClasses.add(TIntArrayList.class);
      thirdPartySerializableClasses.add(TLongArrayList.class);
      thirdPartySerializableClasses.add(StringBuilder.class);
   }

   private static final Set<Class<?>> allowedEuclidTypes = new HashSet<>();
   static
   {
      allowedEuclidTypes.add(Point3D32.class);
      allowedEuclidTypes.add(Vector3D32.class);
      allowedEuclidTypes.add(Point3D.class);
      allowedEuclidTypes.add(Vector3D.class);
      allowedEuclidTypes.add(Point2D32.class);
      allowedEuclidTypes.add(Vector2D32.class);
      allowedEuclidTypes.add(Point2D.class);
      allowedEuclidTypes.add(Vector2D.class);
      allowedEuclidTypes.add(Quaternion.class);
      allowedEuclidTypes.add(Quaternion32.class);
      allowedEuclidTypes.add(Pose2D.class);
      allowedEuclidTypes.add(Pose3D.class);
      allowedEuclidTypes.add(Orientation2D.class);
   }
}
