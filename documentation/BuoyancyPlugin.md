# Buoyancy Plugin

This model plugin is part of the usv_gazebo_plugins; it simulates buoyancy of an object in the fluid. 

![buoyancy.gif](https://bitbucket.org/repo/BgXLzgM/images/1837217943-buoyancy.gif)

## Theory of operation

Archimedes' principle states that the buoyancy force on a body immersed in a fluid is equal to the weight of the fluid displaced by the body:

![$$F_{B} = -\rho V g$$](https://bitbucket.org/repo/BgXLzgM/images/1666239173-eq1.png)

where ρ is the density of the fluid, *V* is the submerged volume of the body and *g* is the acceleration due to gravity. The buoyancy force is applied at the center of the submerged volume of the object. 

![Untitled Diagram.png](https://bitbucket.org/repo/BgXLzgM/images/1306625960-Untitled%20Diagram.png)

The buoyancy forces coupled with gravity can lead to oscillations. In nature, oscillations diminish due to drag forces. We use a simplified force-drag model which approximates drag as a linear function of velocity:

![F_d = \beta_l m \frac{V}{V_T} (\boldsymbol{v_w} - \boldsymbol{v_c})](https://bitbucket.org/repo/BgXLzgM/images/1163409051-eq2.png)

Here *β_l* is the linear drag coefficient, *m* is the mass of the body, *V* is the submerged volume of the body, *V_T* is the total volume of the body, *v_w* is the velocity of the fluid current and *v_c* is the velocity of the body. *F_d* is applied to the center of the submerged volume of the object.

To dissipate angular velocity, we add drag torque as follows:

![T_d = \beta_a m \frac{V}{V_T} L^2 \omega](https://bitbucket.org/repo/BgXLzgM/images/4248525408-eq3.png)

Here *β_a* is the angular drag coefficient, *L* is the average width of the body and *Ω* is the angular velocity of the body. *T_d* is applied to the center of mass of the object.

The buoyancy plugin currently only supports sphere, box and cylindrical shapes. Box and cylinder are modelled as polyhedron and their volume is computationally expensive. Submerged volume for the sphere is calculated by integration and is very efficient. Therefore, sphere approximations should be used unless capturing the higher fidelity physical interactions is warranted.

## Plugin usage
The parameters can be specified within the model definition as SDF parameters. For example:
```
<model>
    ...
    <plugin name="BuoyancyPlugin" filename="libbuoyancy_gazebo_plugin.so">
        <wave_model>ocean_waves</wave_model> <!-- name of wave model object (optional) -->
        <fluid_density>1000</fluid_density>  <!-- density of fluid -->
        <fluid_level>0.0</fluid_level>       <!-- height of fluid / air interface [m] -->
        <linear_drag>25.0</linear_drag>      <!-- linear drag coefficient -->
        <angular_drag>2.0</angular_drag>     <!-- angular drag coefficient -->
        <buoyancy name="buoyancy_sphere">    <!-- describes volume properties -->
            <link_name>link</link_name>
            <pose>0 0 -0.08 0 0 0</pose>
            <geometry>
                <sphere>
                    <radius>0.23</radius>
                </sphere>
            </geometry>
        </buoyancy>
    </plugin>
</model>
```
The volume properties of the body are specified in the `<buoyancy />` tag. Each buoyancy element is associated with a `<link />` in the model. Multiple buoyancy objects can be set up for an object. The `<geometry />` tag supports box, sphere and cylinder shapes. 
You can select drag coefficients through numerical experiments. 

Refer to `usv_gazebo_plugins/worlds/buoyancy_plugin_demo.world.xacro` for more in-depth examples.