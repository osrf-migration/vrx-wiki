# Run VMRC Using Docker Containers#

You will need to install Docker:

* [Install Docker](https://bitbucket.org/osrf/vmrc/wiki/tutorials/installDocker)
    * [**Optional**: Install Nvidia Docker](https://bitbucket.org/osrf/vmrc/wiki/tutorials/installNvidiaDocker) The simulation will run faster using a GPU.  If you computer has an NVIDIA GPU, and you'd like to make the GPU available to the container running the simulation, follow these directions.

There are two options for running a VMRC Docker Image:

## Option 1: Run our Pre-Built Docker Image ##

This is the quickest way to get up and running with Docker. Use this option if you want a stable setup with minimal steps to get running, but without the latest changes to the source code. (Best if you are a user and don't plan to contribute to the project.)

 * [Download and Run](https://bitbucket.org/osrf/vmrc/wiki/tutorials/runDockerHubImage) current VMRC Docker image from Docker Hub

## Option 2: Build Your Own VMRC Image ##

You can also build your own local Docker image from the latest VMRC repository code. Use this option if you want your own Docker container with the most recent source code. (Best if you want to contribute to the VMRC project.)


 * [Build and Run](https://bitbucket.org/osrf/vmrc/wiki/tutorials/buildRunLocalImage) a local image using the latest source code.