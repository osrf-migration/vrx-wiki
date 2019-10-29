# Summary
This tutorial will allow you to verify that your submission is working as expect.

# Prerequisites
We assume the following:

* You have a local installation of the vrx environment built from source according to the [host-based installation instructions (option 2)](https://bitbucket.org/osrf/vrx/wiki/tutorials/SystemSetupInstall).
* Docker is [installed on your system](https://docs.docker.com/install/linux/docker-ce/ubuntu/).
* You have developed a solution to one or more VRX tasks.
* You have [created a Docker image containing your solution](https://bitbucket.org/osrf/vrx/wiki/tutorials/Creating%20a%20Dockerhub%20image%20for%20submission) and uploaded it to Docker hub.
* You have created the [required files](https://bitbucket.org/osrf/vrx/wiki/events/19/vrx_challenge) for a submission.
* You have [forked and cloned the vrx-events repository](https://bitbucket.org/osrf/vrx/wiki/submission_process) to your home directory and added your files.


# Step 1: Verify your dockerhub_image.txt file.

* Change to the directory containing your `dockerhub_image.txt` file.
* Run
```bash
cat dockerhub_image.txt | xargs docker pull
```
* If the contents of the file are correct, docker should begin to pull your image. Once you have verified this is working, you can exit out of the pull using `ctrl+c`.