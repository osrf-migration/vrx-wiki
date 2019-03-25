Coming soon!

## Working With Your Docker Image ##

### Opening additional terminals into your image ###
When working with the VRX environment, it is often useful to open multiple bash terminals. The join.bash script provides an easy way to do this for users running the VRX Docker container. Simply run this script and pass in the name of the Docker container for which you want to create an extra terminal. For example, assuming your container is named vrx and the join.bash script is in your present working directory, the command:

.
```
#!bash

/join.bash vrx
```

will open a new bash session inside the container. Note that the name of the container is set by the argument passed to the run.bash when the image was initiated. If in doubt, you can list currently running containers using the following docker command: 

```
#!bash

docker container ls
```


### Mounting a Mercurial Repository from the Host ###