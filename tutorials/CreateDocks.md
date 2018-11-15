# Overview

In this tutorial we will demonstrate how to create new docks with your own dimensions and shapes. 

The `vmrc_gazebo` ROS package contains multiple models related with docks:

* dock_block: The basic floating building block. Basically, a dock is created by combining these blocks together.
* dock_block_2x2: A 2x2 grid of dock blocks.
* dock_block_3x3: A 3x3 grid of dock blocks.
* dock_block_4x4: A 4x4 grid of dock blocks.
* dock_2016_base: A similar dock used during the 2016 RobotX competition with an `m` shape (three bays).
* dock_2016: The complete 2016 floating dock with the three placards and symbols.
* dock_2018_base: A similar dock used during the 2018 RobotX competition with an `H` shape (two bays).
* dock_2018: The complete 2018 floating dock with the two placards and symbols.

# How to create new docks

The docks are created using [erb templates](https://en.wikipedia.org/wiki/ERuby). This way, we can easily automate the creation of custom docks. Follow the next steps to create your own dock.

* Create a new directory for your dock (e.g.: `my_dock_base`):

```
#!bash

   $ mkdir -p ~/vmrc_ws/src/vmrc_gazebo/models/my_dock_base && cd ~/vmrc_ws/src/vmrc_gazebo/models/my_dock_base
```

* Create the `model.config` file inside the `my_dock_base` directory and edit it with your favorite editor:


```
#!bash

    <?xml version="1.0"?>
    <model>
      <name>my_dock_base</name>
      <version>1.0</version>
      <sdf version="1.6">model.sdf</sdf>

      <author>
        <name>Carlos Agüero</name>
        <email>caguero@openrobotics.org</email>
      </author>

      <description>
        My custom dock base.
      </description>
    </model>
```

* Next, create the `model.sdf.erb` file inside the `my_dock_base` directory and edit it with your favorite editor:

```
#!bash

    <?xml version="1.0" ?>
    <sdf version="1.6">

      <!-- Important: This file is generated. DO NOT EDIT! -->

      <model name="robotx_dock_2016_base">
    <%
    layout = [
    'X  X',
    'X  X',
    'X  X',
    'XXXXXXX',
    'XXXX',
    'XXXXXXX',
    'X  X',
    'X  X',
    'X  X',
    ]
    %>
    <%= ERB.new(File.read('dock_generator.erb'),
                      nil, '-', 'dock').result(binding) %>
      </model>
    </sdf>
```

The `layout` variable controls the layout of the dock. Feel free to modify this variable to your own needs. Every `X` will be transformed into a 4x4 grid dock block. Try not to use a large number of blocks, as these will be converted into visuals that might impact the performance of the simulation.

* Open the `vmrc_gazebo/CMakeLists.txt` file and add your model:

```
#!bash

    # Dock base files that need to be processed with erb
    set (dock_base_erb_files
      models/dock_2016_base/model.sdf.erb
      models/dock_2018_base/model.sdf.erb

      # THIS IS THE ONLY LINE THAT NEEDS TO BE ADDED.
      models/my_dock_base/model.sdf.erb
    )
```

* Generate your new dock:

```
#!bash

    $ cd ~/vmrc_ws/
    $ catkin_make
```

* Launch your simulation and insert your new dock by clicking on `Insert->my_dock_base`:

![my_dock.png](https://bitbucket.org/repo/BgXLzgM/images/109597433-my_dock.png)