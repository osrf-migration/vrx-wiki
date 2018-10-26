## Install Nvidia Docker ##

This is an optional step, but highly recommended if you have an Nvidia GPU available.

* Remove old versions of [Nvidia Docker](https://github.com/NVIDIA/nvidia-docker):

```
#!bash
    $ docker volume ls -q -f driver=nvidia-docker | xargs -r -I{} -n1 docker ps -q -a -f volume={} | xargs -r docker rm -f
    $ sudo apt-get purge -y nvidia-docker
```

* Setup the Nvidia Docker repository. Choose only one block based on your
```
#!bash
    $ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    $ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    $ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
        sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    $ sudo apt-get update
```

* Next, install Nvidia Docker (version 2):

```
#!bash
    $ sudo apt-get install -y nvidia-docker2
```

* Then, restart the Docker daemon 
```
#!bash
    $ sudo service docker restart
```

* Verify the installation:

```
#!bash
    $ docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
```

* [**Ubuntu 18.04 users**] Verify the installation:

```
#!bash
    $ docker run --runtime=nvidia --rm nvidia/cuda:9.1-devel nvidia-smi
```

This command should print your GPU information, for example...

![Screenshot from 2018-06-20 08-21-43.png](https://bitbucket.org/repo/BgXLzgM/images/403079041-Screenshot%20from%202018-06-20%2008-21-43.png)