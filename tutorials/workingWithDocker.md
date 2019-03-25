Coming soon!

## Working With Your Docker Image ##

### Opening additional terminals into your image ###
When working with the VRX environment, it is often useful to open multiple bash terminals. The join.bash script provides an easy way to do this for users running the VRX Docker container. Simply run this script and pass in the name of the Docker container for which you want to create an extra terminal. For example, assuming your container is named vrx and the join.bash script is in your present working directory, the command:

```
#!bash

./join.bash vrx
```

will open a new bash session inside the container. Note that the name of the container is set by the argument passed to the run.bash when the image was initiated. If in doubt, you can list currently running containers using the `docker container ls` command. 

### Mounting a Mercurial Repository from the Host ###
The VRX Docker image contains a copy of the source files needed to build and run the VRX environment. These files are copied from the host machine at build time. Updating the source on the host machine and re-running the build script will cause Docker to copy and build the updated files. A drawback to this approach, however, is that the container must be restarted for the updates to take effect.

A more streamlined alternative is to mount the mercurial repository from the host at runtime. This approach allows the host and the container to share access to the repository, so changes to the source can be made either from the host or from within the container, and all such changes are immediately available both on the host and within the container. This allows a great deal of flexibility. For example, the user can edit source code in the repository from the host, or switch branches using mercurial commands, then rebuild the environment in the container using standard `catkin_make` commands, and see the impact of changes immediately.

Assuming the mercurial repository has been cloned to the local host, the following steps will cause it to be mounted into the VRX container at runtime:

1. Use an editor to open docker/run.bash and uncomment the line that reads:
    
    ```
    DOCKER_OPTS="--mount type=bind,source=${VRX_PATH},target=/home/developer/vrx_ws/src/vrx"
    ```
 
1. Save and quit.
1. Execute the run script as usual. 

The repository on the host will now be shared within the container at `~/vrx_ws/src/vrx`.