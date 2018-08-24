## Build the VMRC image ##

* Create a build directory for the image:

        $ mkdir -p ~/vmrc_docker/vmrc && cd ~/vmrc_docker/vmrc

* Download the appropriate vmrc Dockerfile, depending on whether you are using the Nvidia runtime. 

    * For the Nvidia enabled version:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc_nvidia/Dockerfile

    * Otherwise, if you are running without an Nvidia GPU, use:

            $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/vmrc/Dockerfile

* Download the build and run scripts to the parent of your build directory:

```
#!bash
    $ cd ..
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/build.bash
    $ wget https://bitbucket.org/osrf/vmrc/raw/default/docker/run.bash
    $ chmod u+x build.bash run.bash
```

* Build your VMRC Docker image:

        $ ./build.bash vmrc


## Run the VMRC image ##

* To use the default Docker runtime (no Nvidia):

        $ ./run.bash vmrc

* For the nvidia runtime, use the -n flag:

            $ ./run.bash -n vmrc

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