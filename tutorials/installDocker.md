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