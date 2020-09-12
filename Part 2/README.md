# Part 2 - Building a Basic Robot

Check out the working file here: [`wheely_boi.xacro`](https://github.com/uwrov/Training-Project/blob/master/src/wb/urdf/wheely_boi.xacro)

A gazebo simulated robot is often defined using a URDF (Unifed Robot Description Format) file.

A URDF file is not too different from an HTML file, we define positions and
relations of objects in a big, ugly file. Defining a complex robot in URDF 
is a pain in the butt, so we use a xacro file to make our lives easier.

Xacro files are a slightly more abstract way of defining our robot, as they allow for variables and code reuse.

In this part we will go through the simple `wheely_boi.xacro` to get an understanding of what a xacro is, and how to write one.

### Declarations
The outermost layer of the xacro tells Gazebo that this is a model in xml format. The robot block has two properties.
  - `xmlns:xacro="https://www.ros.org/wiki/xacro"` tells the robot block that we are using a xacro
  - `name` : the name of the model
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="wheely_boi">
  <xacro:property name="wheel_separation" value="0.070" />
  <xacro:property name="wheel_radius" value="0.034" />
  <xacro:property name="wheel_width" value="0.010" />
  <xacro:property name="body_height" value="0.1" />
  <xacro:property name="body_width" value="0.050" />
  <xacro:property name="body_length" value="0.035" />
```

### Link definitions part I
Each of these three components are pointing at a specific link, and they also tell Gazebo what materials/properties it has.
  - `material` : mostly visual, in this case we are just defining colors
  - `mu1` : friction in the x direction
  - `mu2` : friction in the y direction
```xml
  <gazebo reference="edumip_body">
    <material>Gazebo/White</material>
    <mu1>0.2</mu1>
    <mu2>0.2</mu2>
  </gazebo>

  <gazebo reference="wheelR">
    <material>Gazebo/Red</material>
    <mu1>0.84</mu1>
    <mu2>0.84</mu2>
  </gazebo>

  <gazebo reference="wheelL">
    <material>Gazebo/Blue</material>
    <mu1>0.84</mu1>
    <mu2>0.84</mu2>
  </gazebo>
```

### Link definitions part II
Here, we are defining links, which are each parts of the robot. In order to simulate or robot, each link needs three parts.
  - `visual` : tells Gazebo how to render the part on the screen
  - `collision` : tells Gazebo the size of the hitbox, or should be checking for collisions, usually similar to visual
  - `intertial` : relates to the [inertia](https://en.wikipedia.org/wiki/Inertia) of the object
    - `value` : mass in kg
    - `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz` : intertial components
```xml
  <link name="edumip_body">
    <visual>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </visual>

    <collision>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry> 
    </collision>

    <inertial>
      <mass value="0.18"/>
      <inertia ixx="6.0e-4" ixy="0" ixz="0" iyy="6.0e-4" iyz="0" izz="6.0e-4"/>
    </inertial>
  </link>
```

### Defining a Macro
If our robot has links that are very similar, we can define a macro to hold most of the code. 
This will make our xacro files a lot less redundant.

The key differences from above are:
  - `xacro:macro` : Tells xacro that we want to create a macro
    - `name` : Name we use to refer to the macro
    - `params` : Macros can take in parameters to make them more flexible. 
      - The scope of these parameters is limited to the macro definition.
```xml
  <xacro:macro name="wheel_link" params="name radius width mass">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${width}" radius="${radius}"/>
        </geometry>
      </visual>

      <collision>
        <geometry>
          <cylinder length="${width}" radius="${radius}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="1.75e-5" ixy="0.0" ixz="0.0" iyy="1.75e-5" iyz="0.0" izz="1.75e-5"/>
      </inertial>
    </link>
  </xacro:macro>
```
### Using a Macro
To use our macro, we call it with `xacro:_name_`, then fill in the `params` accordingly.
  - This will simply paste in the code defined in the macro
```xml
<xacro:wheel_link name="wheelR" radius="${wheel_radius}" width="${wheel_width}" mass="0.3"/>
<xacro:wheel_link name="wheelL" radius="${wheel_radius}" width="${wheel_width}" mass="0.3"/>
```

### Defining the Joints
To snap the defined parts together, we have to define the joints. Joints are the connection between the links.

Here are the parameters/tags
  - `type` : Defines what kind of joint we want. Because we're making wheels we use a continuous joint.
  - `parent` and `child` : Defines what objects are connected by the joint. Typically the `parent` is fixed while the `child` moves around it.
  - `axis` : The string "0 0 1" corresponds to which axes the joint is allowed to move in.
  - `origin` : Defines the point in space the joint moves about.
  - `friction` : Defines how much friction the joint has.
```xml
  <joint name="jointL" type="continuous">
    <parent link="edumip_body"/>
    <child link="wheelL"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0 ${body_width/2 + wheel_width/2} -${body_height/2 - 0.01}" rpy="${pi/2} 0 0" /> 
    <dynamics friction="0.1"/>
  </joint>

  <joint name="jointR" type="continuous">
    <parent link="edumip_body"/>
    <child link="wheelR"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0 -${body_width/2 + wheel_width/2} -${body_height/2 - 0.01}" rpy="${pi/2} 0 0" />
    <dynamics friction="0.1"/>
  </joint>
```

### Plugin
The last thing to do tell Gazebo what plugin to load, and the file name of the plugin. We'll talk more about plugins later.
```xml
  <gazebo>
    <plugin name="wb_plugin" filename="libwb_plugin.so"> </plugin>
  </gazebo>
</robot> 
```
