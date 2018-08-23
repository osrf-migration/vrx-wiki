Official Docker images are stored in the [vmrc repository](https://hub.docker.com/u/vmrc/dashboard/) on Dockerhub.

For instructions on using these images or building from their Dockerfiles, please see the [System Setup Docker](https://bitbucket.org/osrf/vmrc/wiki/tutorials/SystemSetupDocker) tutorial.

Members of the vmrc organization with push access can upload new versions of Docker images to the repository as follows:

1. Build the new image.

1. Login to Docker from the command line:

            $docker login

1. Tag the image to be pushed to the repository. For example, if the image name is new_nvidia_docker, run:

            $docker tag new_nvidia_docker osrf/vmrc_nvidia:v4

1. Upload the new image. Continuing the example above, you would run:

            $docker push osrf/vmrc_nvidia:v4