# Option 2: Install all software using Docker #

It's possible to use [Docker](https://www.docker.com/) to simplify the installation process or if you prefer to leave your host system untouched. We have created a Docker image that you'll need to build following these instructions.

## Verify System Requirements ##

 * Check the Hardware section of the [System Requirements](https://bitbucket.org/osrf/vmrc/wiki/system_requirements)
 * While it may be possible to use Docker on non-Ubuntu Linux distributions, a Ubuntu-based OS is strongly encouraged.
  * When using Docker you do not need to have ROS/Gazebo installed on the host!

## Install Supporting Tools ##

```
#!bash
    $ sudo apt install curl
```

## Install Docker ##

Docker has two available versions: Community Edition (CE) and Enterprise Edition (EE). In this tutorial, we'll install the CE version.

* Remove old versions of Docker (if installed):

```
#!bash
    $ sudo apt-get remove docker docker-engine docker.io
```

* Install the following dependencies needed to setup an external package repository:

```
#!bash
   $ sudo apt-get install apt-transport-https ca-certificates curl software-properties-common
```

* Add the official GPG key of Docker:

```
#!bash
    $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

* Add the stable package repository:

```
#!bash
    $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```

**Note for Ubuntu Bionic (18.04 LTS) users only **: There's no stable package yet. Instead, run the following command:

```
#!bash
    $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) edge"
```

* Now, we can install Docker:

```
#!bash
    $ sudo apt-get update && sudo apt-get install docker-ce
```
For Ubuntu 16.04, docker-ce should be roughly version 18.03.1 (as of June, 2018).


* Check your Docker installation:

```
#!bash
    $ sudo docker run hello-world
```

You should see the message `Hello from Docker!` confirming that your installation was successfully completed.

## Install Nvidia Docker ##

This is an optional step, but highly recommended if you have an Nvidia GPU available. If not, skip this section and proceed directly to "Build the VMRC image" below.

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

This command should print your GPU information, for example...
![Screenshot from 2018-06-20 08-21-43.png](https://bitbucket.org/repo/BgXLzgM/images/403079041-Screenshot%20from%202018-06-20%2008-21-43.png)

## Build the VMRC image ##
This is an optional step to build a local copy of the Docker image from our Dockerfile. If you just want to use the most recent version available in the osrf DockerHub repository, you can skip directly to "Run the VMRC image," below.

* Create a build directory for the image:

        $ mkdir -p ~/vmrc_docker/vmrc && cd ~/vmrc_docker/vmrc

* Download the appropriate vmrc Dockerfile, depending on whether you are using the Nvidia runtime. 

    * For the Nvidia enabled version:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc_nvidia/Dockerfile

    * Otherwise, if you are running without an Nvidia GPU, use:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc/Dockerfile

* Download the build script to the parent of your build directory:

```
#!bash
    $ cd ..
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/build.bash
    $ chmod u+x build.bash
```

* Build your VMRC Docker image:

        $ ./build.bash vmrc


## Run the VMRC image ##

* Create a vmrc docker directory (if you haven't already) and download the run script:

```
#!bash
    $ mkdir -p ~/vmrc_docker && cd ~/vmrc_docker
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/run.bash
    $ chmod u+x run.bash
```

* Run the VMRC container provided on Docker Hub:

    * To use the default Docker runtime (no Nvidia):

            $ ./run.bash osrf/vmrc

    * For the nvidia runtime, use the -n flag:

            $ ./run.bash -n osrf/vmrc_nvidia

* Alternately, if you built your own local Docker container using the instructions in the previous step, you can run it using either:

            $ ./run.bash vmrc

for the default runtime or

            $ ./run.bash -n vmrc

if you built the nvidia version.

* Note that if this is the first time running the image and you did not build it on your machine, the script will first pull the image from the repository to your machine, which may take a moment.

* If all has gone well, the command will drop you into a bash prompt running as the user "developer" inside the container. 

* Test your VMRC installation:

```
#!bash
    $ roslaunch robotx_gazebo sandisland.launch
```


## Troubleshooting: ##

#### Permission Error ####

If this is the first time you've used docker on this machine, when you run the above command you may get an error similar to...
```
docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post http://%2Fvar%2Frun%2Fdocker.sock/v1.37/containers/create: dial unix /var/run/docker.sock: connect: permission denied. See 'docker run --help'.
```

You will need to add your user account the docker group,
```
sudo usermod -a -G docker $USER
```
and then logout-log-back-in for the changes to take effect.

#### Test Docker Network Access ####

Run the command...
```
docker run busybox nslookup google.com
```

If the system returns something like…
```
$ docker run busybox nslookup google.com
Server:    172.20.20.11
Address 1: 172.20.20.11 lee.ern.nps.edu

Name:      google.com
Address 1: 2607:f8b0:4005:804::200e sfo07s13-in-x0e.1e100.net
Address 2: 216.58.195.78 sfo07s16-in-f78.1e100.net
```
then the system has network access and we can proceed with building the VMRC container.

If the system hangs for a long time (~5 min) and returns something like
```
Server:    8.8.8.8
Address 1: 8.8.8.8

nslookup: can't resolve 'google.com'
```
then we have a network problem. The likely cause is that we can’t use the default DNS.

See the [Troubleshooting](https://bitbucket.org/osrf/vmrc/wiki/Troubleshooting) page Setup and Install page, under the sections Setup->Docker->Networking.