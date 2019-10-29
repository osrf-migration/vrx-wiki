# Summary
This tutorial will allow you to verify that your submission is working as expect.

# Prerequisites
We assume the following:

* You have developed a solution to one or more VRX tasks.
* You have created a Docker image containing your solution and uploaded it to Docker hub.
* You have created the required files for a submission.
* You have a local installation of the vrx environment built from source according to the host-based installation instructions, available here.
* Docker is installed on your system.

# Step 1: Verify your dockerhub_image.txt file.

* Change to the directory containing your `dockerhub_image.txt` file.
* Run
```bash
cat dockerhub_image.txt | xargs docker pull
```
* If the contents of the file are correct, docker should begin to pull your image. Once you have verified this is working, you can exit out of the pull using `ctrl+c`.