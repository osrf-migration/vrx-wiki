# Contents
[TOC]


# Setup and Install #

## Docker ##

### Permissions ###
If this is the first time you've used docker on this machine, when you run the above command you may get an error similar to...
```
docker: Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock: Post http://%2Fvar%2Frun%2Fdocker.sock/v1.37/containers/create: dial unix /var/run/docker.sock: connect: permission denied. See 'docker run --help'.
```

You will need to add your user account the docker group,
```
sudo usermod -a -G docker $USER
```
and then logout-log-back-in for the changes to take effect.

### Networking ###

#### Docker DNS Trouble ####

**Symptom**
The symptom is that you cannot get network access from within your Docker container.  We can test the container network access with...

```docker run busybox nslookup google.com```

If the system returns something like…
```
$ docker run busybox nslookup google.com
Server:    172.20.20.11
Address 1: 172.20.20.11 lee.ern.nps.edu

Name:      google.com
Address 1: 2607:f8b0:4005:804::200e sfo07s13-in-x0e.1e100.net
Address 2: 216.58.195.78 sfo07s16-in-f78.1e100.net
```
then the system has network access and we can proceed with building the VRX container.

If the system hangs for a long time (~5 min) and returns something like
```
Server:    8.8.8.8
Address 1: 8.8.8.8

nslookup: can't resolve 'google.com'
```
then we have a network problem. The likely cause is that we can’t use the default DNS.

**Temporary Fix**

We can verify that this is the case by temporarily providing an explicit DNS to the docker run command.

 1. Find out what DNS your host is using...
```
$ nmcli dev show | grep 'IP4.DNS'
IP4.DNS[1]:                             172.20.20.11
IP4.DNS[2]:                             172.10.20.12
```
 2. Repeat the nslookup using your DNS
```
$ docker run --dns 172.20.20.11 busybox nslookup google.com
Server:    172.20.20.11
Address 1: 172.20.20.11 lee.ern.nps.edu

Name:      google.com
Address 1: 2607:f8b0:4005:807::200e sfo07s16-in-x0e.1e100.net
Address 2: 216.58.195.78 sfo07s16-in-f78.1e100.net
```

Unfortunately the docker build command doesn't seem to have a ```dns``` command line option to set this explicitly in the build step.

We have tried configuring docker to use a specific DNS in two ways
 1. Add the following line to the /etc/docker/daemon.json file ```{ "dns": ["172.20.20.11", "172.10.20.12"] }```
 2. Add the following lines to the /etc/default/docker file
```
# Use DOCKER_OPTS to modify the daemon startup options.
DOCKER_OPTS="--dns 172.20.20.11 --dns 172.10.20.12"
```

For both attempts you need to restart docker: ```docker run busybox less /etc/resolv.conf``` and then rerun the nslookup test or try ```docker run busybox less /etc/resolv.conf```  If this test returns
```
# Dynamic resolv.conf(5) file for glibc resolver(3) generated by resolvconf(8)
#     DO NOT EDIT THIS FILE BY HAND -- YOUR CHANGES WILL BE OVERWRITTEN

nameserver 8.8.8.8
nameserver 8.8.4.4
```
then we likely still don't have access.

**A Solution**
One way to solve this is...

* Edit the NetworkManager file
```
sudo cp /run/resolvconf/interface/NetworkManager /run/resolvconf/interface/NetworkManager.orig
sudo nano /run/resolvconf/interface/NetworkManager
```

* Add lines for your specific DNS, e.g.,
```
nameserver 172.20.20.11
nameserver 172.10.20.12
```

* Update resolveconf
```
sudo resolvconf -u
```

* Verify that the changes are now visible in the container
```
docker run busybox less /etc/resolv.conf
```
which should list the same DNS IPs, e.g.,
```
# Dynamic resolv.conf(5) file for glibc resolver(3) generated by resolvconf(8)
#     DO NOT EDIT THIS FILE BY HAND -- YOUR CHANGES WILL BE OVERWRITTEN
nameserver 172.20.20.11
nameserver 172.10.20.12
```

References:

 * https://stackoverflow.com/questions/24151129/docker-network-calls-fail-during-image-build-on-corporate-network
 * https://development.robinwinslow.uk/2016/06/23/fix-docker-networking-dns/

# Adding a package  - with network trouble

We want to add a package - say iputils-ping so that we can use the ping command

# Network trouble

As described above, we can't connect to the network because of presumed firewall issues.

 1. Start container - within container...
 2. Manually add the DNS addresses: `vim /etc/resolv.conf` and add nameserver entries as above.
 3. `sudo apt install iputils-ping nano `

# ROS key issues

When we try `sudo apt update` we get errors such as 
```
W: An error occurred during the signature verification. The repository is not updated and the previous index files will be used. GPG error: http://packages.ros.org/ros/ubuntu bionic InRelease: The following signatures couldn't be verified because the public key is not available: NO_PUBKEY F42ED6FBAB17C654
```

Which is a non-docker specific issue described here: https://answers.ros.org/question/325039/apt-update-fails-cannot-install-pkgs-key-not-working/


# How to update existing docker container - perhaps adding some packages #

These instructions are a bit specific  to BSB's setup.

## Network setup on host  ##

Because of NPS network issues.  We need to do the following (from above) on the host computer.

* Edit the NetworkManager file
```
sudo cp /run/resolvconf/interface/NetworkManager /run/resolvconf/interface/NetworkManager.orig
sudo nano /run/resolvconf/interface/NetworkManager
```

* Add lines for your specific DNS, e.g.,
```
nameserver 172.20.20.11
nameserver 172.10.20.12
```

* Update resolveconf
```
sudo resolvconf -u
```

* Verify that the changes are now visible in the container
```
docker run busybox less /etc/resolv.conf
```
which should list the same DNS IPs, e.g.,
```
# Dynamic resolv.conf(5) file for glibc resolver(3) generated by resolvconf(8)
#     DO NOT EDIT THIS FILE BY HAND -- YOUR CHANGES WILL BE OVERWRITTEN
nameserver 172.20.20.11
nameserver 172.10.20.12
```

## Add any additional steps or packages at the end of the docker file ##

I keep my notes at https://github.com/bsb808/nps_robotx/blob/master/docker/bsb_docker_installs.txt

## Re-run the docker build which should incrementally add in any changes  ##

```
cd vrx
docker/build.bash -n .
```