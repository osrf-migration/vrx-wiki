# Download log files

The [VRX automated evaluation tool](https://bitbucket.org/osrf/vrx-docker/src/default/) runs and scores team submissions for a given event. This tool also generates multiple log files, including Gazebo log files that can be used to playback the simulations.

## Install `aws` tool

All logs are uploaded to the AWS S3 `vrx-events` bucket. You'll need a command line tool to interact with AWS S3 and download your log files:

```
sudo apt-get update
sudo apt install awscli
```

## Download your log files

Create a directory to store your log files:

```
mkdir ~/vrx_logs
```

Now, sync your recently created local folder with the remote directory in the S3 bucket:

```
aws s3 sync s3://vrx-events/<year>/<event> . --no-sign-request --exclude "*" --include "*<teamname>*"
```

**Note:** Substitute <year>, <event>, and <teamname> with the year and event that you are interested.

E.g.:

```
aws s3 sync s3://vrx-events/2019/rehearsal . --no-sign-request --exclude "*" --include "*team_osrf*"
```

**Note:** Be patient as these log files might take a while to download.