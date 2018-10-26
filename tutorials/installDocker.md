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

* Now, we can install Docker:

```
#!bash
    $ sudo apt-get update && sudo apt-get install docker-ce
```

## Check your Docker installation:

 * First try running a docker container using sudo

```
#!bash
    $ sudo docker run hello-world
```

You should see the message `Hello from Docker!` confirming that your installation was successfully completed.

  * Next try running as a normal user

```
#!bash
    $ docker run hello-world
```

You should see the message `Hello from Docker!` confirming that your installation was successfully completed.

If you see an error that starts with text such as

```
#!bash
docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/dock
```

Then try adding yourself to the docker user group `sudo usermod -a -G docker $USER`.  Unfortunately you'll need to logoff and login for this change to complete.  Then retry running the hello-world test.